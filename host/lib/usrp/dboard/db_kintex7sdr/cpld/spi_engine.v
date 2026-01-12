`timescale 1ns / 1ps

`default_nettype none
// ============================================================================
// spi_engine_clean.v - SPI frame engine (MODE0) + robust SEL latch/error handling
//
// Key properties:
//  - SPI_ADDR is treated as ASYNC (GPIO from FPGA). We latch it only on CS falling.
//  - MISO during CMD phase = immediate echo of MOSI (helps master alignment).
//  - For WRITE payloads: MISO echoes MOSI.
//  - For READ payloads: MISO outputs rd_word_lat bits (prepared on negedge).
//  - evt_frame_end is a ONE-CLOCK PULSE (bitcnt saturates at 31).
//
// "Clean" changes (to reduce harmless warnings and save a couple FFs):
//  - cmd_shift is 7 bits (instead of 8). We only need the previous 7 CMD bits.
//  - data_shift is 23 bits (instead of 24). We only need the previous 23 DATA bits.
// ============================================================================

module spi_engine #(
    parameter [2:0] CPLD_DEST = 3'b000
)(
    input  wire       sclk,
    input  wire       cs_n,       // active-low CS
    input  wire       mosi,

    input  wire [2:0] spi_addr,   // ASYNC from FPGA GPIO (stable around CS edges)

    input  wire [23:0] rd_word_in,

    output wire        cs_active,
    output reg  [5:0]  bitcnt,

    output wire        evt_cmd_end,
    output wire        evt_frame_end,

    output wire [7:0]  cmd_byte_full,

    output reg         is_write_lat,
    output reg  [2:0]  bank_lat,
    output reg  [3:0]  reg_lat,

    output wire [23:0] wr_data_full,

    output reg  [2:0]  sel_latched,
    output wire        sel_is_cpld,
    output reg         err_invalid_sel,

    input  wire        clr_err_invalid_sel_evt,

    output wire        miso_o,
    output wire        miso_oe
);

    assign cs_active   = ~cs_n;
    assign sel_is_cpld = (sel_latched == CPLD_DEST);

    // CMD end is naturally a pulse because bitcnt increments past 7.
    assign evt_cmd_end = cs_active && (bitcnt == 6'd7);

    // evt_frame_end must be a pulse even though bitcnt saturates at 31.
    reg frame_end_seen;
    assign evt_frame_end = cs_active && (bitcnt == 6'd31) && !frame_end_seen;

    // Store only the PREVIOUS bits needed to form {prev_bits, current_mosi}
    reg [6:0]  cmd_shift;   // previous 7 CMD bits
    reg [22:0] data_shift;  // previous 23 DATA bits

    // Full CMD / DATA words "as seen on the wire" at current bit.
    assign cmd_byte_full = {cmd_shift[6:0], mosi};
    assign wr_data_full  = {data_shift[22:0], mosi};

    // READ response word latched at CMD end for READ transactions.
    reg [23:0] rd_word_lat;

    // Prepared read bit for MISO (updated on negedge SCLK).
    reg miso_bit_reg;

    wire in_cmd_phase  = (bitcnt < 6'd8);
    wire in_data_phase = (bitcnt >= 6'd8);

    // MISO data selection:
    //  - CMD phase: immediate echo
    //  - WRITE payload: echo
    //  - READ payload: rd_word_lat bits (prepared on negedge)
    wire miso_data =
        in_cmd_phase ? mosi :
        (is_write_lat && in_data_phase) ? mosi :
        miso_bit_reg;

    // CPLD drives MISO only when selected.
    assign miso_oe = cs_active && sel_is_cpld;
    assign miso_o  = miso_data;

    // Power-up init (helps simulation and gives a defined start state).
    // NOTE: deterministic HW init should be done via external reset if needed.
    initial begin
        bitcnt          = 6'd0;
        cmd_shift       = 7'd0;
        data_shift      = 23'd0;
        is_write_lat    = 1'b0;
        bank_lat        = 3'd0;
        reg_lat         = 4'd0;
        sel_latched     = CPLD_DEST;
        err_invalid_sel = 1'b0;
        rd_word_lat     = 24'd0;
        miso_bit_reg    = 1'b0;
        frame_end_seen  = 1'b0;
    end

    // Latch selector on CS falling edge (start of transaction)
    always @(negedge cs_n) begin
        sel_latched <= spi_addr;
        // err_invalid_sel is sticky (cleared only by soft reset event)
    end

    // Main SPI sampling
    always @(posedge sclk or posedge cs_n) begin
        if (cs_n) begin
            // Transaction ended
            bitcnt         <= 6'd0;
            cmd_shift      <= 7'd0;
            data_shift     <= 23'd0;

            is_write_lat   <= 1'b0;
            bank_lat       <= 3'd0;
            reg_lat        <= 4'd0;

            rd_word_lat    <= 24'd0;
            frame_end_seen <= 1'b0;

            // Detect illegal selector change during CS active
            if (spi_addr != sel_latched) begin
                err_invalid_sel <= 1'b1;
            end

        end else begin
            // CS active

            // Clear selection error on SOFT_RST event (from regmap)
            if (clr_err_invalid_sel_evt) begin
                err_invalid_sel <= 1'b0;
            end

            // Once bitcnt reached 31, remember it (evt_frame_end is one-shot)
            if (bitcnt == 6'd31) begin
                frame_end_seen <= 1'b1;
            end

            // Shift in CMD bits (bitcnt 0..7)
            if (bitcnt < 6'd8) begin
                cmd_shift <= {cmd_shift[5:0], mosi};

                // At the last CMD bit, latch decoded fields.
                if (bitcnt == 6'd7) begin
                    is_write_lat <= cmd_byte_full[7];
                    bank_lat     <= cmd_byte_full[6:4];
                    reg_lat      <= cmd_byte_full[3:0];

                    // Latch READ word for CPLD READ transactions.
                    if (sel_is_cpld && (cmd_byte_full[7] == 1'b0)) begin
                        rd_word_lat <= rd_word_in;
                    end else begin
                        rd_word_lat <= 24'd0;
                    end
                end

            end else begin
                // Shift in DATA bits (bitcnt 8..31)
                data_shift <= {data_shift[21:0], mosi};
            end

            // Advance counter (saturate at 31)
            if (bitcnt < 6'd31) begin
                bitcnt <= bitcnt + 6'd1;
            end else begin
                bitcnt <= 6'd31;
            end
        end
    end

    // Prepare MISO read bit on negedge SCLK (setup before next posedge)
    // For READ payload: bitcnt 8..31 => rd_word_lat[31-bitcnt] => [23..0]
    always @(negedge sclk or posedge cs_n) begin
        if (cs_n) begin
            miso_bit_reg <= 1'b0;
        end else begin
            if (cs_active && sel_is_cpld && !is_write_lat &&
                (bitcnt >= 6'd8) && (bitcnt <= 6'd31)) begin
                miso_bit_reg <= rd_word_lat[31 - bitcnt];
            end else begin
                miso_bit_reg <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire
