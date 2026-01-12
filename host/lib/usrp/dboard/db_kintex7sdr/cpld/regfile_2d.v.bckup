`timescale 1ns / 1ps
`default_nettype none
// ============================================================================
// regfile_2d.v - 2-D register storage REGS[bank][reg] + defaults + writes
//
// XST(CPLD) compatible rules applied:
//  - NO always @(*) / @*   -> explicit sensitivity list (Verilog-95 safe)
//  - NO posedge cs_n in sensitivity list (XST:1468 in CPLD flow)
//  - Single clocked always: posedge sclk
//  - "End of transaction" commit happens on (cs_n==0 && bitcnt==31) because
//    your SPI frame is fixed at 32 SCLK edges.
//
// Functionality:
//  - Power-up defaults via initial (if XST later complains, replace by reset)
//  - Masked commit writes (CTRL0/CTRL1/ATT2_CODE)
//  - Soft reset override (defaults + STATUS1.SOFT_RST_OCCURRED)
//  - GPIO snapshots at end-of-frame
//  - STATUS1 sticky set OR-mask + clear-on-read (bits0..3 cleared when armed)
//  - ATT2_CTRL.LAST_OK stored bit2 set/clr pulses
// ============================================================================

module regfile_2d #(
    parameter [7:0] ID0_CHAR   = "K",
    parameter [7:0] ID1_CHAR   = "7",
    parameter [7:0] ID2_CHAR   = "S",
    parameter [7:0] VER_MAJOR  = 8'd1,
    parameter [7:0] VER_MINOR  = 8'd0,
    parameter [7:0] VER_PATCH  = 8'd0
)(
    input  wire        sclk,
    input  wire        cs_n,

    // bit counter from spi_engine (0..31)
    input  wire [5:0]  bitcnt,

    input  wire [7:0]  gpio0_now,
    input  wire [7:0]  gpio1_now,

    input  wire        soft_reset_evt,

    input  wire        commit_we,
    input  wire [2:0]  w_bank,
    input  wire [3:0]  w_reg,
    input  wire [23:0] w_data,
    input  wire [23:0] w_mask,   // 1 = writable bit

    input  wire [23:0] status1_set_mask_pulse, // pulse(s) during frame
    input  wire        clr_status1_on_cs_rise,  // "armed" flag: clear bits0..3 at frame end

    input  wire        att2_last_ok_set_pulse,
    input  wire        att2_last_ok_clr_pulse,

    input  wire [2:0]  rd_bank,
    input  wire [3:0]  rd_reg,
    output reg  [23:0] rd_data,

    output wire [23:0] ctrl0_q,
    output wire [23:0] ctrl1_q,
    output wire [23:0] att2_code_q,
    output wire [23:0] status1_q,
    output wire [23:0] att2_ctrl_q,
    output wire [23:0] gpio_in0_q,
    output wire [23:0] gpio_in1_q
);

    localparam integer NBANK = 8;
    localparam integer NREG  = 16;

    localparam [2:0] BANK0 = 3'd0;
    localparam [2:0] BANK1 = 3'd1;
    localparam [2:0] BANK2 = 3'd2;
    localparam [2:0] BANK3 = 3'd3;

    // BANK0 regs
    localparam [3:0] R_ID0   = 4'h0;
    localparam [3:0] R_ID1   = 4'h1;
    localparam [3:0] R_ID2   = 4'h2;
    localparam [3:0] R_VMAJ  = 4'h3;
    localparam [3:0] R_VMIN  = 4'h4;
    localparam [3:0] R_VPAT  = 4'h5;
    localparam [3:0] R_STAT1 = 4'h7;

    // BANK1 regs
    localparam [3:0] R_CTRL0 = 4'h0;
    localparam [3:0] R_CTRL1 = 4'h1;

    // BANK2 regs
    localparam [3:0] R_ATT2_CODE = 4'h0;
    localparam [3:0] R_ATT2_CTRL = 4'h1;

    // BANK3 regs
    localparam [3:0] R_GPIO_IN0  = 4'h0;
    localparam [3:0] R_GPIO_IN1  = 4'h1;

    // STATUS1: bits 0..3 are sticky and clear-on-read
    localparam [23:0] STATUS1_CLR_MASK = 24'h00000F;

    // ATT2_CTRL stored bits: LAST_OK at bit2
    localparam integer ATT2CTL_LAST_OK = 2;

    // Defaults (soft reset)
    localparam [23:0] CTRL0_DFLT     = 24'h000000;
    localparam [23:0] CTRL1_DFLT     = 24'h000000;
    localparam [23:0] ATT2_CODE_DFLT = 24'h000000;
    localparam [23:0] ATT2_CTRL_DFLT = 24'h000004; // bit2 LAST_OK = 1
    localparam [23:0] STAT1_SOFT_RST = 24'h000002; // bit1 SOFT_RST_OCCURRED = 1

    // 2-D storage
    reg [23:0] REGS [0:NBANK-1][0:NREG-1];

    integer b, r;

    // ---------------------------
    // Power-up defaults
    // ---------------------------
    initial begin
        for (b = 0; b < NBANK; b = b + 1)
            for (r = 0; r < NREG; r = r + 1)
                REGS[b][r] = 24'd0;

        // ID/VER (RO constants)
        REGS[BANK0][R_ID0]  = {16'd0, ID0_CHAR};
        REGS[BANK0][R_ID1]  = {16'd0, ID1_CHAR};
        REGS[BANK0][R_ID2]  = {16'd0, ID2_CHAR};
        REGS[BANK0][R_VMAJ] = {16'd0, VER_MAJOR};
        REGS[BANK0][R_VMIN] = {16'd0, VER_MINOR};
        REGS[BANK0][R_VPAT] = {16'd0, VER_PATCH};

        // STATUS1 sticky starts cleared
        REGS[BANK0][R_STAT1] = 24'd0;

        // CTRL defaults
        REGS[BANK1][R_CTRL0] = CTRL0_DFLT;
        REGS[BANK1][R_CTRL1] = CTRL1_DFLT;

        // ATT2 defaults
        REGS[BANK2][R_ATT2_CODE] = ATT2_CODE_DFLT;
        REGS[BANK2][R_ATT2_CTRL] = ATT2_CTRL_DFLT;

        // GPIO snapshots
        REGS[BANK3][R_GPIO_IN0] = 24'd0;
        REGS[BANK3][R_GPIO_IN1] = 24'd0;
    end

    // ---------------------------
    // Fixed outputs (constant indices)
    // ---------------------------
    assign ctrl0_q     = REGS[BANK1][R_CTRL0];
    assign ctrl1_q     = REGS[BANK1][R_CTRL1];
    assign att2_code_q = REGS[BANK2][R_ATT2_CODE];
    assign status1_q   = REGS[BANK0][R_STAT1];
    assign att2_ctrl_q = REGS[BANK2][R_ATT2_CTRL];
    assign gpio_in0_q  = REGS[BANK3][R_GPIO_IN0];
    assign gpio_in1_q  = REGS[BANK3][R_GPIO_IN1];

    // Read taps for BANK0 constants (avoid variable indexing REGS[BANK0][rd_reg])
    wire [23:0] id0_q   = REGS[BANK0][R_ID0];
    wire [23:0] id1_q   = REGS[BANK0][R_ID1];
    wire [23:0] id2_q   = REGS[BANK0][R_ID2];
    wire [23:0] vmaj_q  = REGS[BANK0][R_VMAJ];
    wire [23:0] vmin_q  = REGS[BANK0][R_VMIN];
    wire [23:0] vpat_q  = REGS[BANK0][R_VPAT];
    wire [23:0] stat1_q = REGS[BANK0][R_STAT1];

    // ---------------------------
    // COMB read mux (Verilog-95 safe): explicit sensitivity list
    // ---------------------------
    always @(rd_bank or rd_reg or
             id0_q or id1_q or id2_q or vmaj_q or vmin_q or vpat_q or stat1_q or
             ctrl0_q or ctrl1_q or att2_code_q or att2_ctrl_q or
             gpio_in0_q or gpio_in1_q) begin
        rd_data = 24'd0;

        case (rd_bank)
            BANK0: begin
                case (rd_reg)
                    R_ID0:   rd_data = id0_q;
                    R_ID1:   rd_data = id1_q;
                    R_ID2:   rd_data = id2_q;
                    R_VMAJ:  rd_data = vmaj_q;
                    R_VMIN:  rd_data = vmin_q;
                    R_VPAT:  rd_data = vpat_q;
                    R_STAT1: rd_data = stat1_q;
                    default: rd_data = 24'd0;
                endcase
            end

            BANK1: begin
                case (rd_reg)
                    R_CTRL0: rd_data = ctrl0_q;
                    R_CTRL1: rd_data = ctrl1_q;
                    default: rd_data = 24'd0;
                endcase
            end

            BANK2: begin
                case (rd_reg)
                    R_ATT2_CODE: rd_data = att2_code_q;
                    R_ATT2_CTRL: rd_data = att2_ctrl_q;
                    default:     rd_data = 24'd0;
                endcase
            end

            BANK3: begin
                case (rd_reg)
                    R_GPIO_IN0: rd_data = gpio_in0_q;
                    R_GPIO_IN1: rd_data = gpio_in1_q;
                    default:    rd_data = 24'd0;
                endcase
            end

            default: rd_data = 24'd0;
        endcase
    end

    // ---------------------------
    // Helpers
    // ---------------------------
    function [23:0] mask_write;
        input [23:0] oldv;
        input [23:0] dat;
        input [23:0] msk;
        begin
            mask_write = (oldv & ~msk) | (dat & msk);
        end
    endfunction

    wire frame_active = (cs_n == 1'b0);
    wire frame_start  = frame_active && (bitcnt == 6'd0);
    wire frame_end    = frame_active && (bitcnt == 6'd31);

    // Pending accumulators inside a frame
    reg [23:0] pend_status1_set;
    reg        pend_last_ok_set;
    reg        pend_last_ok_clr;

    // Temporary scratch for STATUS1 update (declared at module scope for old parsers)
    reg [23:0] stat1_next;

    // ---------------------------
    // Single-writer sequential process (posedge sclk only)
    // ---------------------------
    always @(posedge sclk) begin
        if (frame_start) begin
            pend_status1_set <= 24'd0;
            pend_last_ok_set <= 1'b0;
            pend_last_ok_clr <= 1'b0;
        end

        if (frame_active) begin
            // accumulate sticky SETs over the frame
            if (status1_set_mask_pulse != 24'd0)
                pend_status1_set <= pend_status1_set | status1_set_mask_pulse;

            // accumulate LAST_OK pulses
            if (att2_last_ok_set_pulse)
                pend_last_ok_set <= 1'b1;
            if (att2_last_ok_clr_pulse)
                pend_last_ok_clr <= 1'b1;

            // Commit at end-of-frame
            if (frame_end) begin
                // GPIO snapshot (same semantics as CS^ for fixed 32-bit frames)
                REGS[BANK3][R_GPIO_IN0] <= {16'd0, gpio0_now};
                REGS[BANK3][R_GPIO_IN1] <= {16'd0, gpio1_now};

                // Soft reset has highest priority
                if (soft_reset_evt) begin
                    REGS[BANK1][R_CTRL0]     <= CTRL0_DFLT;
                    REGS[BANK1][R_CTRL1]     <= CTRL1_DFLT;
                    REGS[BANK2][R_ATT2_CODE] <= ATT2_CODE_DFLT;
                    REGS[BANK2][R_ATT2_CTRL] <= ATT2_CTRL_DFLT;
                    REGS[BANK0][R_STAT1]     <= STAT1_SOFT_RST;
                end else begin
                    // STATUS1 clear-on-read (armed) + sticky SET
                    stat1_next = REGS[BANK0][R_STAT1];
                    if (clr_status1_on_cs_rise)
                        stat1_next = stat1_next & ~STATUS1_CLR_MASK;

                    // include both accumulated and current-cycle pulse (safety)
                    stat1_next = stat1_next | (pend_status1_set | status1_set_mask_pulse);
                    REGS[BANK0][R_STAT1] <= stat1_next;

                    // ATT2_CTRL.LAST_OK: apply CLR then SET
                    if (pend_last_ok_clr | att2_last_ok_clr_pulse)
                        REGS[BANK2][R_ATT2_CTRL][ATT2CTL_LAST_OK] <= 1'b0;
                    if (pend_last_ok_set | att2_last_ok_set_pulse)
                        REGS[BANK2][R_ATT2_CTRL][ATT2CTL_LAST_OK] <= 1'b1;

                    // Masked commit write (only allowed RW regs)
                    if (commit_we) begin
                        case (w_bank)
                            BANK1: begin
                                case (w_reg)
                                    R_CTRL0: REGS[BANK1][R_CTRL0] <= mask_write(REGS[BANK1][R_CTRL0], w_data, w_mask);
                                    R_CTRL1: REGS[BANK1][R_CTRL1] <= mask_write(REGS[BANK1][R_CTRL1], w_data, w_mask);
                                    default: ; // ignore
                                endcase
                            end
                            BANK2: begin
                                case (w_reg)
                                    R_ATT2_CODE: REGS[BANK2][R_ATT2_CODE] <= mask_write(REGS[BANK2][R_ATT2_CODE], w_data, w_mask);
                                    default: ; // ignore
                                endcase
                            end
                            default: ; // ignore RO banks
                        endcase
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
