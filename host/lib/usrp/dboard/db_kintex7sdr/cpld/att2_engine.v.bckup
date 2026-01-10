`timescale 1ns / 1ps

`default_nettype none
// ============================================================================
// att2_engine.v - smart PE43711 (ATT2) with TRUE AUTOLATCH
//
// Shift window: last 7 payload bits -> bitcnt 25..31 (exactly 7 SCLK pulses).
//  - APPLY mode    : shift stored ATT2_CODE[6:0]
//  - AUTOLATCH mode: shift raw MOSI bits as they arrive (payload D[6:0])
//
// LE pulse: asserted while SCLK high and le_armed=1, right after frame end.
// ============================================================================

module att2_engine (
    input  wire sclk,
    input  wire cs_n,

    input  wire mosi_raw,     // <<< NEW: raw MOSI for true AUTOLATCH

    input  wire       cs_active,
    input  wire [5:0] bitcnt,

    input  wire       sel_is_cpld,
    input  wire       evt_cmd_end,
    input  wire [7:0] cmd_byte_full,

    input  wire       evt_frame_end,
    input  wire       is_write_lat,
    input  wire [23:0] wr_data_full,

    input  wire       ctrl0_autolatch,
    input  wire       ctrl0_le_pol,
    input  wire [23:0] att2_code_q,

    output wire ATT2_RX_LE,
    output wire ATT2_SCLK_RX,
    output wire ATT2_MOSI_RX,

    output wire att2_busy,

    output wire apply_done_set_pulse,
    output wire last_ok_set_pulse,
    output wire last_ok_clr_pulse
);

    localparam [2:0] BANK2 = 3'd2;
    localparam [3:0] R_ATT2_CODE = 4'h0;
    localparam [3:0] R_ATT2_CTRL = 4'h1;

    localparam integer APPLY_W1_BIT = 0;

    // Decode CMD at cmd end
    wire cmd_is_write  = (cmd_byte_full[7] == 1'b1);
    wire [2:0] cmd_bank = cmd_byte_full[6:4];
    wire [3:0] cmd_reg  = cmd_byte_full[3:0];

    // mode: 0 idle, 1 APPLY, 2 AUTOLATCH
    reg [1:0] mode;
    reg       le_armed;

    // Last 7 payload clocks: bitcnt 25..31
    wire shift_window =
        sel_is_cpld && cs_active &&
        (bitcnt >= 6'd25) && (bitcnt <= 6'd31) &&
        (mode != 2'd0);

    assign ATT2_SCLK_RX = shift_window ? sclk : 1'b0;

    // APPLY mode MOSI bit prepared on negedge for setup
    reg att2_mosi_reg;

    // True AUTOLATCH = raw MOSI in window, APPLY = stored bit in window
    assign ATT2_MOSI_RX =
        (shift_window && mode == 2'd2) ? mosi_raw :
        (shift_window && mode == 2'd1) ? att2_mosi_reg :
        1'b0;

    // Prepare stored code bits for APPLY: bitcnt 25..31 -> code[6..0]
    // bitcnt 25 -> code[6], ..., bitcnt 31 -> code[0]
    always @(negedge sclk or posedge cs_n) begin
        if (cs_n) begin
            att2_mosi_reg <= 1'b0;
        end else begin
            if (shift_window && mode == 2'd1) begin
                att2_mosi_reg <= att2_code_q[31 - bitcnt]; // 25..31 -> 6..0
            end else begin
                att2_mosi_reg <= 1'b0;
            end
        end
    end

    // Trigger conditions at frame end
    wire apply_trigger      = (mode == 2'd1) && wr_data_full[APPLY_W1_BIT];
    wire autolatch_trigger  = (mode == 2'd2);

    // Apply done pulse (used to set STATUS1.APPLY_DONE)
    assign apply_done_set_pulse =
        evt_frame_end && sel_is_cpld && is_write_lat &&
        (apply_trigger || autolatch_trigger);

    assign last_ok_set_pulse = apply_done_set_pulse;

    // Clear LAST_OK at the start of an operation (cmd end of relevant writes)
    assign last_ok_clr_pulse =
        evt_cmd_end && sel_is_cpld && cmd_is_write &&
        (
          (cmd_bank == BANK2 && cmd_reg == R_ATT2_CTRL) ||
          (cmd_bank == BANK2 && cmd_reg == R_ATT2_CODE && ctrl0_autolatch)
        );

    // Busy = mode active or LE armed
    assign att2_busy = (mode != 2'd0) || le_armed;

    // LE pulse: active while SCLK high and le_armed=1
    wire le_pulse_level = le_armed && sclk;
    assign ATT2_RX_LE = ctrl0_le_pol ? ~le_pulse_level : le_pulse_level;

    // State
    always @(posedge sclk or posedge cs_n) begin
        if (cs_n) begin
            mode     <= 2'd0;
            le_armed <= 1'b0;
        end else begin
            // Select mode at CMD end
            if (evt_cmd_end && sel_is_cpld && cmd_is_write) begin
                if (cmd_bank == BANK2 && cmd_reg == R_ATT2_CTRL) begin
                    mode     <= 2'd1; // APPLY
                    le_armed <= 1'b0;
                end else if (cmd_bank == BANK2 && cmd_reg == R_ATT2_CODE && ctrl0_autolatch) begin
                    mode     <= 2'd2; // AUTOLATCH
                    le_armed <= 1'b0;
                end else begin
                    mode <= 2'd0;
                end
            end

            // Frame end -> optional LE + stop shifting mode
            if (evt_frame_end && sel_is_cpld && is_write_lat) begin
                if (apply_trigger || autolatch_trigger) begin
                    le_armed <= 1'b1;
                end
                mode <= 2'd0;
            end
        end
    end

endmodule

`default_nettype wire
