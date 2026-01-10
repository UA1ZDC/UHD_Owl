`timescale 1ns / 1ps

`default_nettype none
// ============================================================================
// status_block.v
//  - Builds STATUS0 dynamic word
//  - Arms "clear STATUS1 on CS rising" when CMD is READ of BANK0:0x07
// ============================================================================

module status_block (
    input  wire sclk,
    input  wire cs_n,

    input  wire       evt_cmd_end,
    input  wire [7:0] cmd_byte_full,
    input  wire       sel_is_cpld,

    input  wire       STAT_LTC6948,
    input  wire       cs_active,
    input  wire [2:0] sel_latched,
    input  wire       err_invalid_sel,
    input  wire       att2_busy,

    output wire [23:0] status0_word,
    output reg         clr_status1_on_cs_rise
);

    // STATUS0 layout (byte in [7:0])
    // bit0 STAT_LTC6948
    // bit1 SPI_XFER_ACTIVE
    // bit2 SEL_IS_CPLD
    // bit3 ERR_INVALID_SEL
    // bits6..4 SEL_LATCHED[2:0]
    // bit7 ATT2_BUSY
    wire [7:0] status0_byte = {
        att2_busy,
        sel_latched,
        err_invalid_sel,
        sel_is_cpld,
        cs_active,
        STAT_LTC6948
    };

    assign status0_word = {16'd0, status0_byte};

    wire cmd_is_read = (cmd_byte_full[7] == 1'b0);
    wire [2:0] cmd_bank = cmd_byte_full[6:4];
    wire [3:0] cmd_reg  = cmd_byte_full[3:0];

    localparam [2:0] BANK0 = 3'd0;
    localparam [3:0] R_STAT1 = 4'h7;

    // Arm clear-on-read: if we detect READ of STATUS1 at CMD end, clear on CS^.
    always @(posedge sclk or posedge cs_n) begin
        if (cs_n) begin
            // transaction ended -> disarm for the next one
            clr_status1_on_cs_rise <= 1'b0;
        end else begin
            if (evt_cmd_end && sel_is_cpld && cmd_is_read &&
                (cmd_bank == BANK0) && (cmd_reg == R_STAT1)) begin
                clr_status1_on_cs_rise <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire

