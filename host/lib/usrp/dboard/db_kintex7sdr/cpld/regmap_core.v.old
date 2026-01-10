`timescale 1ns / 1ps

`default_nettype none
// ============================================================================
// regmap_core.v - glue + new STATUS1.ERR_WRITE_TO_RO (bit3)
// ============================================================================

module regmap_core #(
    parameter [7:0] ID0_CHAR   = "K",
    parameter [7:0] ID1_CHAR   = "7",
    parameter [7:0] ID2_CHAR   = "S",
    parameter [7:0] VER_MAJOR  = 8'd1,
    parameter [7:0] VER_MINOR  = 8'd0,
    parameter [7:0] VER_PATCH  = 8'd0
)(
    input  wire       sclk,
    input  wire       cs_n,
    input  wire       mosi,

    input  wire       cs_active,
    input  wire [5:0] bitcnt,

    input  wire       evt_cmd_end,
    input  wire       evt_frame_end,
    input  wire [7:0] cmd_byte_full,

    input  wire       is_write_lat,
    input  wire [2:0] bank_lat,
    input  wire [3:0] reg_lat,

    input  wire [23:0] wr_data_full,

    input  wire [2:0] sel_latched,
    input  wire       sel_is_cpld,
    input  wire       err_invalid_sel,

    input  wire       STAT_LTC6948,

    input  wire cpld_io_00, input  wire cpld_io_01, input  wire cpld_io_02, input  wire cpld_io_03,
    input  wire cpld_io_04, input  wire cpld_io_05, input  wire cpld_io_06, input  wire cpld_io_07,
    input  wire cpld_io_08, input  wire cpld_io_09, input  wire cpld_io_10, input  wire cpld_io_11,
    input  wire cpld_io_12, input  wire cpld_io_13, input  wire cpld_io_14, input  wire cpld_io_15,

    output wire TPS_EN,
    output wire LED_RX,
    output wire ATT1_RX_C1,
    output wire ATT1_RX_C2,

    output wire ATT2_RX_LE,
    output wire ATT2_SCLK_RX,
    output wire ATT2_MOSI_RX,

    output wire [23:0] rd_word_for_cmd,
    output wire        clr_err_invalid_sel_evt
);

    wire cmd_is_write = cmd_byte_full[7];
    wire [2:0] cmd_bank = cmd_byte_full[6:4];
    wire [3:0] cmd_reg  = cmd_byte_full[3:0];

    localparam [2:0] BANK0 = 3'd0;
    localparam [2:0] BANK1 = 3'd1;
    localparam [2:0] BANK2 = 3'd2;
    localparam [2:0] BANK3 = 3'd3;

    localparam [3:0] R_STAT0 = 4'h6;
    localparam [3:0] R_STAT1 = 4'h7;

    localparam [3:0] R_CTRL0 = 4'h0;
    localparam [3:0] R_CTRL1 = 4'h1;

    localparam [3:0] R_ATT2_CODE = 4'h0;
    localparam [3:0] R_ATT2_CTRL = 4'h1;

    localparam [3:0] R_GPIO_IN0 = 4'h0;
    localparam [3:0] R_GPIO_IN1 = 4'h1;

    localparam integer CTRL0_AUTOLATCH   = 4;
    localparam integer CTRL0_LE_POL      = 5;
    localparam integer CTRL0_SOFT_RST_W1 = 6;

    // STATUS1 sticky bits
    localparam integer STAT1_APPLY_DONE      = 0;
    localparam integer STAT1_SOFT_RST_OCC    = 1;
    localparam integer STAT1_ERR_INV_ADDR    = 2;
    localparam integer STAT1_ERR_WR_TO_RO    = 3; // <<< NEW

    // ATT2_CTRL readback bits
    localparam integer ATT2CTL_APPLY_W1   = 0;
    localparam integer ATT2CTL_BUSY_RO    = 1;
    localparam integer ATT2CTL_LAST_OK_RO = 2;

    function impl_fn;
        input [2:0] bk;
        input [3:0] rg;
        begin
            impl_fn = 1'b0;
            if (bk == BANK0) begin
                if (rg <= R_STAT1) impl_fn = 1'b1;
            end else if (bk == BANK1) begin
                if (rg == R_CTRL0 || rg == R_CTRL1) impl_fn = 1'b1;
            end else if (bk == BANK2) begin
                if (rg == R_ATT2_CODE || rg == R_ATT2_CTRL) impl_fn = 1'b1;
            end else if (bk == BANK3) begin
                if (rg == R_GPIO_IN0 || rg == R_GPIO_IN1) impl_fn = 1'b1;
            end
        end
    endfunction

    wire cmd_addr_valid = impl_fn(cmd_bank, cmd_reg);

    wire [7:0] gpio0_now = {cpld_io_07,cpld_io_06,cpld_io_05,cpld_io_04,cpld_io_03,cpld_io_02,cpld_io_01,cpld_io_00};
    wire [7:0] gpio1_now = {cpld_io_15,cpld_io_14,cpld_io_13,cpld_io_12,cpld_io_11,cpld_io_10,cpld_io_09,cpld_io_08};

    // regfile
    wire [23:0] rf_rd_data;

    wire [23:0] ctrl0_q;
    wire [23:0] ctrl1_q;
    wire [23:0] att2_code_q;
    wire [23:0] status1_q;
    wire [23:0] att2_ctrl_q;
    wire [23:0] gpio_in0_q;
    wire [23:0] gpio_in1_q;

    wire        rf_commit_we;
    wire [2:0]  rf_w_bank;
    wire [3:0]  rf_w_reg;
    wire [23:0] rf_w_data;
    wire [23:0] rf_w_mask;

    wire        clr_status1_on_cs_rise;
    wire [23:0] status1_set_mask_pulse;

    wire att2_last_ok_set_pulse;
    wire att2_last_ok_clr_pulse;

    wire soft_reset_evt;

    regfile_compact #(
        .ID0_CHAR  (ID0_CHAR),
        .ID1_CHAR  (ID1_CHAR),
        .ID2_CHAR  (ID2_CHAR),
        .VER_MAJOR (VER_MAJOR),
        .VER_MINOR (VER_MINOR),
        .VER_PATCH (VER_PATCH)
    ) u_rf (
        .sclk (sclk),
        .cs_n (cs_n),
		  
		  .bitcnt(bitcnt),

        .gpio0_now (gpio0_now),
        .gpio1_now (gpio1_now),

        .soft_reset_evt (soft_reset_evt),

        .commit_we (rf_commit_we),
        .w_bank    (rf_w_bank),
        .w_reg     (rf_w_reg),
        .w_data    (rf_w_data),
        .w_mask    (rf_w_mask),

        .status1_set_mask_pulse (status1_set_mask_pulse),
        .clr_status1_on_cs_rise (clr_status1_on_cs_rise),

        .att2_last_ok_set_pulse (att2_last_ok_set_pulse),
        .att2_last_ok_clr_pulse (att2_last_ok_clr_pulse),

        .rd_bank (cmd_bank),
        .rd_reg  (cmd_reg),
        .rd_data (rf_rd_data),

        .ctrl0_q     (ctrl0_q),
        .ctrl1_q     (ctrl1_q),
        .att2_code_q (att2_code_q),
        .status1_q   (status1_q),
        .att2_ctrl_q (att2_ctrl_q),
        .gpio_in0_q  (gpio_in0_q),
        .gpio_in1_q  (gpio_in1_q)
    );

    // STATUS0 + arm clear-on-read STATUS1
    wire [23:0] status0_word;
    wire        att2_busy;

    status_block u_status (
        .sclk      (sclk),
        .cs_n      (cs_n),

        .evt_cmd_end   (evt_cmd_end),
        .cmd_byte_full (cmd_byte_full),
        .sel_is_cpld   (sel_is_cpld),

        .STAT_LTC6948    (STAT_LTC6948),
        .cs_active       (cs_active),
        .sel_latched     (sel_latched),
        .err_invalid_sel (err_invalid_sel),
        .att2_busy       (att2_busy),

        .status0_word           (status0_word),
        .clr_status1_on_cs_rise (clr_status1_on_cs_rise)
    );

    // ATT2 engine (TRUE AUTOLATCH uses raw MOSI)
    wire apply_done_set_pulse;

    att2_engine u_att2 (
        .sclk      (sclk),
        .cs_n      (cs_n),

        .mosi_raw  (mosi),          // <<< NEW connection

        .cs_active (cs_active),
        .bitcnt    (bitcnt),

        .sel_is_cpld   (sel_is_cpld),
        .evt_cmd_end   (evt_cmd_end),
        .cmd_byte_full (cmd_byte_full),

        .evt_frame_end (evt_frame_end),
        .is_write_lat  (is_write_lat),
        .wr_data_full  (wr_data_full),

        .ctrl0_autolatch (ctrl0_q[CTRL0_AUTOLATCH]),
        .ctrl0_le_pol    (ctrl0_q[CTRL0_LE_POL]),
        .att2_code_q     (att2_code_q),

        .ATT2_RX_LE    (ATT2_RX_LE),
        .ATT2_SCLK_RX  (ATT2_SCLK_RX),
        .ATT2_MOSI_RX  (ATT2_MOSI_RX),

        .att2_busy     (att2_busy),

        .apply_done_set_pulse (apply_done_set_pulse),
        .last_ok_set_pulse    (att2_last_ok_set_pulse),
        .last_ok_clr_pulse    (att2_last_ok_clr_pulse)
    );

    // SOFT_RST at frame end: write CTRL0 with bit6=1
    assign soft_reset_evt =
        evt_frame_end && sel_is_cpld && is_write_lat &&
        (bank_lat == BANK1) && (reg_lat == R_CTRL0) &&
        (wr_data_full[CTRL0_SOFT_RST_W1] == 1'b1);

    assign clr_err_invalid_sel_evt = soft_reset_evt;

    // ERR_INVALID_ADDR sticky (set at CMD end if impl_fn()==0)
    wire inv_addr_set_pulse = evt_cmd_end && sel_is_cpld && !cmd_addr_valid;

    // ---------------------------
    // NEW: ERR_WRITE_TO_RO sticky (set at frame end for "illegal write")
    // ---------------------------
    localparam [23:0] MASK_CTRL0_RW     = 24'h00003F; // bits0..5
    localparam [23:0] MASK_CTRL1_RW     = 24'hFFFFFF; // all
    localparam [23:0] MASK_ATT2_CODE_RW = 24'h00007F; // bits0..6
    localparam [23:0] MASK_ATT2_CTRL_W1 = 24'h000001; // bit0 only (APPLY)

    wire wr_to_ctrl0     = (bank_lat == BANK1) && (reg_lat == R_CTRL0);
    wire wr_to_ctrl1     = (bank_lat == BANK1) && (reg_lat == R_CTRL1);
    wire wr_to_att2_code = (bank_lat == BANK2) && (reg_lat == R_ATT2_CODE);
    wire wr_to_att2_ctrl = (bank_lat == BANK2) && (reg_lat == R_ATT2_CTRL);

    // "reserved bits write attempt" checks:
    // CTRL0: bits7..23 must be 0 (bit6 is allowed as W1 action)
    wire ctrl0_reserved_written = (wr_data_full & 24'hFFFF80) != 24'd0;

    // ATT2_CODE: bits7..23 must be 0
    wire att2_code_reserved_written = (wr_data_full & 24'hFFFF80) != 24'd0;

    // ATT2_CTRL: only bit0 allowed (W1 APPLY); other bits => error
    wire att2_ctrl_reserved_written = (wr_data_full & ~MASK_ATT2_CTRL_W1) != 24'd0;

    // Full condition: only for implemented addresses and CPLD-selected writes at frame end
    wire err_write_to_ro_pulse =
        evt_frame_end && sel_is_cpld && is_write_lat && impl_fn(bank_lat, reg_lat) &&
        (
            // All BANK0 regs are RO (ID/VER/STATUS0/STATUS1)
            (bank_lat == BANK0) ||

            // All BANK3 regs are RO (GPIO snapshots)
            (bank_lat == BANK3) ||

            // ATT2_CTRL accepts only bit0 (W1)
            (wr_to_att2_ctrl && att2_ctrl_reserved_written) ||

            // CTRL0 reserved bits write attempt
            (wr_to_ctrl0 && ctrl0_reserved_written) ||

            // ATT2_CODE reserved bits write attempt
            (wr_to_att2_code && att2_code_reserved_written)
            // CTRL1 is fully RW => no error
        );

    // STATUS1 set mask pulses
    assign status1_set_mask_pulse =
          (apply_done_set_pulse     ? (24'd1 << STAT1_APPLY_DONE)   : 24'd0)
        | (soft_reset_evt           ? (24'd1 << STAT1_SOFT_RST_OCC) : 24'd0)
        | (inv_addr_set_pulse       ? (24'd1 << STAT1_ERR_INV_ADDR) : 24'd0)
        | (err_write_to_ro_pulse    ? (24'd1 << STAT1_ERR_WR_TO_RO) : 24'd0);

    // Commit writes to writable regs (soft reset overrides)
    assign rf_commit_we =
        evt_frame_end && sel_is_cpld && is_write_lat && !soft_reset_evt &&
        (wr_to_ctrl0 || wr_to_ctrl1 || wr_to_att2_code);

    assign rf_w_bank = bank_lat;
    assign rf_w_reg  = reg_lat;
    assign rf_w_data = wr_data_full;

    assign rf_w_mask =
        wr_to_ctrl0     ? MASK_CTRL0_RW :
        wr_to_ctrl1     ? MASK_CTRL1_RW :
        wr_to_att2_code ? MASK_ATT2_CODE_RW :
                          24'd0;

    // Read mux for spi_engine READ response (CMD-decoded address)
    reg [23:0] rd_mux;
    always @* begin
        rd_mux = 24'd0;

        if (!cmd_addr_valid) begin
            rd_mux = 24'd0;
        end else if (cmd_bank == BANK0 && cmd_reg == R_STAT0) begin
            rd_mux = status0_word;
        end else if (cmd_bank == BANK2 && cmd_reg == R_ATT2_CTRL) begin
            rd_mux = 24'd0;
            rd_mux[ATT2CTL_BUSY_RO]    = att2_busy;
            rd_mux[ATT2CTL_LAST_OK_RO] = att2_ctrl_q[ATT2CTL_LAST_OK_RO];
        end else begin
            rd_mux = rf_rd_data;
        end
    end
    assign rd_word_for_cmd = rd_mux;

    // Outputs from CTRL0
    assign TPS_EN     = ctrl0_q[0];
    assign LED_RX     = ctrl0_q[1];
    assign ATT1_RX_C1 = ctrl0_q[2];
    assign ATT1_RX_C2 = ctrl0_q[3];

endmodule

`default_nettype wire

