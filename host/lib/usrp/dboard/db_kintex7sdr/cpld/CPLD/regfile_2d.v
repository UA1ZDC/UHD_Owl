`timescale 1ns / 1ps
`default_nettype none
// ============================================================================
// regfile_2d.v - 2-D register storage REGS[bank][reg] + defaults + writes
//
// XST(CPLD) FIX:
//  - Do NOT use (posedge sclk or posedge cs_n) with functional cs_n branch.
//  - Accumulate "pending" events on posedge sclk.
//  - Commit all REGS updates on posedge cs_n (end of SPI transaction).
//
// Implements:
//  - power-up defaults (initial)  [NOTE: if XST later complains, we'll replace by reset]
//  - masked writes (commit_we) -> pending then apply on CS^
//  - soft reset override -> pending then apply on CS^
//  - GPIO snapshots on CS^
//  - STATUS1 sticky bits: pending OR-mask + clear-on-read on CS^ (bits0..3)
//  - ATT2_CTRL.LAST_OK stored bit2 with set/clr pending, applied on CS^
// ============================================================================

module regfile_2d #(
    parameter [7:0] ID0_CHAR   = "K",
    parameter [7:0] ID1_CHAR   = "7",
    parameter [7:0] ID2_CHAR   = "S",
    parameter [7:0] VER_MAJOR  = 8'd1,
    parameter [7:0] VER_MINOR  = 8'd0,
    parameter [7:0] VER_PATCH  = 8'd0
)(
    input  wire sclk,
    input  wire cs_n,

    input  wire [7:0] gpio0_now,
    input  wire [7:0] gpio1_now,

    input  wire soft_reset_evt,

    input  wire        commit_we,
    input  wire [2:0]  w_bank,
    input  wire [3:0]  w_reg,
    input  wire [23:0] w_data,
    input  wire [23:0] w_mask,   // 1 = writable bit

    input  wire [23:0] status1_set_mask_pulse, // pulse on SCLK domain
    input  wire        clr_status1_on_cs_rise,  // armed flag (from status_block)

    input  wire att2_last_ok_set_pulse,
    input  wire att2_last_ok_clr_pulse,

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

    localparam [3:0] R_ID0   = 4'h0;
    localparam [3:0] R_ID1   = 4'h1;
    localparam [3:0] R_ID2   = 4'h2;
    localparam [3:0] R_VMAJ  = 4'h3;
    localparam [3:0] R_VMIN  = 4'h4;
    localparam [3:0] R_VPAT  = 4'h5;
    localparam [3:0] R_STAT1 = 4'h7;

    localparam [3:0] R_CTRL0 = 4'h0;
    localparam [3:0] R_CTRL1 = 4'h1;

    localparam [3:0] R_ATT2_CODE = 4'h0;
    localparam [3:0] R_ATT2_CTRL = 4'h1;

    localparam [3:0] R_GPIO_IN0  = 4'h0;
    localparam [3:0] R_GPIO_IN1  = 4'h1;

    // STATUS1 bits 0..3 are sticky and clear-on-read
    localparam [23:0] STATUS1_CLR_MASK = 24'h00000F; // bits0..3

    // ATT2_CTRL stored bits: only LAST_OK at bit2 is stored here
    localparam integer ATT2CTL_LAST_OK_RO = 2;

    // Handy defaults for soft reset (avoid double-assigning bit slices)
    localparam [23:0] CTRL0_DFLT     = 24'h000000;
    localparam [23:0] CTRL1_DFLT     = 24'h000000;
    localparam [23:0] ATT2_CODE_DFLT = 24'h000000;
    localparam [23:0] ATT2_CTRL_DFLT = 24'h000004; // bit2 LAST_OK = 1
    localparam [23:0] STAT1_SOFT_RST = 24'h000002; // bit1 = SOFT_RST_OCCURRED

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

        // STATUS1 starts cleared
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

    // ---------------------------
    // Safe synthesizable read mux (no variable indexing pitfalls)
    // ---------------------------
    always @* begin
        rd_data = 24'd0;
        case (rd_bank)
            BANK0: begin
                case (rd_reg)
                    R_ID0, R_ID1, R_ID2, R_VMAJ, R_VMIN, R_VPAT, R_STAT1: rd_data = REGS[BANK0][rd_reg];
                    default: rd_data = 24'd0;
                endcase
            end
            BANK1: begin
                case (rd_reg)
                    R_CTRL0: rd_data = REGS[BANK1][R_CTRL0];
                    R_CTRL1: rd_data = REGS[BANK1][R_CTRL1];
                    default: rd_data = 24'd0;
                endcase
            end
            BANK2: begin
                case (rd_reg)
                    R_ATT2_CODE: rd_data = REGS[BANK2][R_ATT2_CODE];
                    R_ATT2_CTRL: rd_data = REGS[BANK2][R_ATT2_CTRL];
                    default: rd_data = 24'd0;
                endcase
            end
            BANK3: begin
                case (rd_reg)
                    R_GPIO_IN0: rd_data = REGS[BANK3][R_GPIO_IN0];
                    R_GPIO_IN1: rd_data = REGS[BANK3][R_GPIO_IN1];
                    default: rd_data = 24'd0;
                endcase
            end
            default: rd_data = 24'd0;
        endcase
    end

    // =========================================================================
    // PENDING LATCHES (captured on SCLK domain during active CS)
    // =========================================================================
    reg        pend_commit;
    reg [2:0]  pend_bank;
    reg [3:0]  pend_reg;
    reg [23:0] pend_data;
    reg [23:0] pend_mask;

    reg        pend_soft_reset;
    reg [23:0] pend_status1_set;

    reg        pend_last_ok_set;
    reg        pend_last_ok_clr;

    // Capture pulses/commits on posedge SCLK while CS active (cs_n=0)
    always @(posedge sclk) begin
        if (!cs_n) begin
            // accumulate STATUS1 sets (sticky OR)
            if (status1_set_mask_pulse != 24'd0)
                pend_status1_set <= pend_status1_set | status1_set_mask_pulse;

            // latch write commit (only one per frame expected)
            if (commit_we) begin
                pend_commit <= 1'b1;
                pend_bank   <= w_bank;
                pend_reg    <= w_reg;
                pend_data   <= w_data;
                pend_mask   <= w_mask;
            end

            // latch soft reset request (frame-based)
            if (soft_reset_evt)
                pend_soft_reset <= 1'b1;

            // latch LAST_OK pulses
            if (att2_last_ok_set_pulse)
                pend_last_ok_set <= 1'b1;
            if (att2_last_ok_clr_pulse)
                pend_last_ok_clr <= 1'b1;
        end
    end

    // =========================================================================
    // COMMIT UPDATES ON CS RISING EDGE (end of transaction)
    // =========================================================================
    function [23:0] mask_write;
        input [23:0] oldv;
        input [23:0] dat;
        input [23:0] msk;
        begin
            mask_write = (oldv & ~msk) | (dat & msk);
        end
    endfunction

    always @(posedge cs_n) begin
        // 1) snapshot GPIO always on CS^
        REGS[BANK3][R_GPIO_IN0] <= {16'd0, gpio0_now};
        REGS[BANK3][R_GPIO_IN1] <= {16'd0, gpio1_now};

        // 2) Soft reset has highest priority for state registers
        if (pend_soft_reset) begin
            REGS[BANK1][R_CTRL0]     <= CTRL0_DFLT;
            REGS[BANK1][R_CTRL1]     <= CTRL1_DFLT;
            REGS[BANK2][R_ATT2_CODE] <= ATT2_CODE_DFLT;
            REGS[BANK2][R_ATT2_CTRL] <= ATT2_CTRL_DFLT;
            REGS[BANK0][R_STAT1]     <= STAT1_SOFT_RST; // sets bit1

        end else begin
            // 3) STATUS1 clear-on-read (armed)
            if (clr_status1_on_cs_rise) begin
                REGS[BANK0][R_STAT1] <= (REGS[BANK0][R_STAT1] & ~STATUS1_CLR_MASK);
            end

            // 4) STATUS1 sticky set from this frame (apply AFTER possible clear)
            if (pend_status1_set != 24'd0) begin
                // If clr_status1_on_cs_rise was true, STAT1 already scheduled to clear.
                // We need to ensure set is applied on top: compute from current and re-assign.
                if (clr_status1_on_cs_rise) begin
                    REGS[BANK0][R_STAT1] <= ((REGS[BANK0][R_STAT1] & ~STATUS1_CLR_MASK) | pend_status1_set);
                end else begin
                    REGS[BANK0][R_STAT1] <= (REGS[BANK0][R_STAT1] | pend_status1_set);
                end
            end

            // 5) ATT2 LAST_OK stored bit2 (apply clr then set)
            if (pend_last_ok_clr)
                REGS[BANK2][R_ATT2_CTRL][ATT2CTL_LAST_OK_RO] <= 1'b0;
            if (pend_last_ok_set)
                REGS[BANK2][R_ATT2_CTRL][ATT2CTL_LAST_OK_RO] <= 1'b1;

            // 6) Apply masked commit write (explicit decode; no variable 2D indexing)
            if (pend_commit) begin
                case (pend_bank)
                    BANK1: begin
                        case (pend_reg)
                            R_CTRL0: REGS[BANK1][R_CTRL0] <= mask_write(REGS[BANK1][R_CTRL0], pend_data, pend_mask);
                            R_CTRL1: REGS[BANK1][R_CTRL1] <= mask_write(REGS[BANK1][R_CTRL1], pend_data, pend_mask);
                            default: ; // ignore
                        endcase
                    end
                    BANK2: begin
                        case (pend_reg)
                            R_ATT2_CODE: REGS[BANK2][R_ATT2_CODE] <= mask_write(REGS[BANK2][R_ATT2_CODE], pend_data, pend_mask);
                            default: ; // ignore
                        endcase
                    end
                    default: ; // ignore RO banks
                endcase
            end
        end

        // 7) Clear pending flags at end of transaction
        pend_commit      <= 1'b0;
        pend_bank        <= 3'd0;
        pend_reg         <= 4'd0;
        pend_data        <= 24'd0;
        pend_mask        <= 24'd0;

        pend_soft_reset  <= 1'b0;
        pend_status1_set <= 24'd0;

        pend_last_ok_set <= 1'b0;
        pend_last_ok_clr <= 1'b0;
    end

endmodule

`default_nettype wire
