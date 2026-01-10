`timescale 1ns / 1ps

`default_nettype none
// ============================================================================
// spi_engine.v - SPI frame engine (MODE0) + robust SEL latch/error handling
//
// Changes vs previous version:
//  - SPI_ADDR is treated as ASYNC (comes from GPIO pins driven by FPGA).
//  - We do NOT sample SPI_ADDR on SCLK edges (avoids metastability/glitches).
//  - We latch sel_latched on CS falling edge.
//  - On CS rising edge we compare current spi_addr with sel_latched:
//      if different -> sticky err_invalid_sel = 1
//    (meaning: SPI_ADDR was changed during the active CS window).
// ============================================================================

module spi_engine #(
    parameter [2:0] CPLD_DEST = 3'b000
)(
    input  wire       sclk,
    input  wire       cs_n,       // active-low CS
    input  wire       mosi,

    input  wire [2:0] spi_addr,   // ASYNC from GPIO pins (stable around CS edges)

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

    assign evt_cmd_end   = cs_active && (bitcnt == 6'd7);
    assign evt_frame_end = cs_active && (bitcnt == 6'd31);

    reg [7:0]  cmd_shift;
    reg [23:0] data_shift;

    assign cmd_byte_full = {cmd_shift[6:0], mosi};
    assign wr_data_full  = {data_shift[22:0], mosi};

    reg [23:0] rd_word_lat;

    reg miso_bit_reg;

    wire in_cmd_phase  = (bitcnt < 6'd8);
    wire in_data_phase = (bitcnt >= 6'd8);

    wire miso_data =
        in_cmd_phase ? mosi :
        (is_write_lat && in_data_phase) ? mosi :
        miso_bit_reg;

    assign miso_oe = cs_active && sel_is_cpld;
    assign miso_o  = miso_data;

    // ------------------------------------------------------------
    // Latch selector on CS falling edge (start of transaction)
    // ------------------------------------------------------------
    always @(negedge cs_n) begin
        sel_latched <= spi_addr;
        // do not clear err_invalid_sel here (sticky until SOFT_RST)
    end

    // ------------------------------------------------------------
    // Main SPI sampling
    // ------------------------------------------------------------
    always @(posedge sclk or posedge cs_n) begin
        if (cs_n) begin
            // End of transaction / idle
            bitcnt       <= 6'd0;
            cmd_shift    <= 8'd0;
            data_shift   <= 24'd0;

            is_write_lat <= 1'b0;
            bank_lat     <= 3'd0;
            reg_lat      <= 4'd0;

            rd_word_lat  <= 24'd0;

            // Robust ERR_INVALID_SEL: if address != latched at CS end -> it was changed during CS active
            if (spi_addr != sel_latched) begin
                err_invalid_sel <= 1'b1;
            end
        end else begin
            // CS active

            // Clear sticky selection error on SOFT_RST event (from regmap)
            if (clr_err_invalid_sel_evt) begin
                err_invalid_sel <= 1'b0;
            end

            // Shift in CMD
            if (bitcnt < 6'd8) begin
                cmd_shift <= {cmd_shift[6:0], mosi};

                if (bitcnt == 6'd7) begin
                    is_write_lat <= cmd_byte_full[7];
                    bank_lat     <= cmd_byte_full[6:4];
                    reg_lat      <= cmd_byte_full[3:0];

                    // Latch read word now for READ transactions to CPLD
                    if (sel_is_cpld && (cmd_byte_full[7] == 1'b0)) begin
                        rd_word_lat <= rd_word_in;
                    end else begin
                        rd_word_lat <= 24'd0;
                    end
                end
            end else begin
                // Shift in DATA
                data_shift <= {data_shift[22:0], mosi};
            end

            // Advance counter
            if (bitcnt < 6'd31) begin
                bitcnt <= bitcnt + 6'd1;
            end else begin
                bitcnt <= 6'd31;
            end
        end
    end

    // ------------------------------------------------------------
    // Prepare MISO read bit on negedge SCLK (setup before next posedge)
    // For READ payload: rd_word_lat[23 - (bitcnt-8)] = rd_word_lat[31 - bitcnt]
    // ------------------------------------------------------------
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

