`timescale 1ns / 1ps

`default_nettype none
// ============================================================================
// top_module_full_clean.v - Top-level wrapper (FULL UCF pin list)
// + SPI routing by GPIO SPI_ADDR
//
// Cleanups vs your current top_module_full.v:
//  - Removed unused "cpld_rst_n_from_gpio/_unused_rst" wires.
//    (cpld_io_03 is already visible inside regmap_core via gpio0_now[3] and
//     can be used for async reset in regfile_compact_patched if you enabled it.)
// ============================================================================

module top_module_full #(
    parameter [7:0] ID0_CHAR   = "K",
    parameter [7:0] ID1_CHAR   = "7",
    parameter [7:0] ID2_CHAR   = "S",
    parameter [7:0] VER_MAJOR  = 8'd1,
    parameter [7:0] VER_MINOR  = 8'd0,
    parameter [7:0] VER_PATCH  = 8'd0,

    // Destination codes on SPI_ADDR[2:0]
    parameter [2:0] CPLD_DEST      = 3'b000,
    parameter [2:0] DEST_LTC6948   = 3'b001,
    parameter [2:0] DEST_LTC5594   = 3'b010,
    parameter [2:0] DEST_AD7922    = 3'b011,
    parameter [2:0] DEST_AD7922_2  = 3'b100
)(
    // RX SPI (master -> CPLD)
    input  wire       SCLK_RX,
    input  wire       SEN_RX,       // active-low CS
    input  wire       MOSI_RX,
    output wire       MISO_RX,

    // LTC6948 SPI
    output wire       SCLK_LTC6948,
    output wire       MOSI_LTC6948,
    output wire       CS_LTC6948,      // active-low
    input  wire       MISO_LTC6948,
    input  wire       STAT_LTC6948,

    // LTC5594
    output wire       SEN_LTC5594,     // active-low
    output wire       SCLK_LTC5594,
    output wire       MOSI_LTC5594,
    input  wire       MISO_LTC5594,

    // AD7922 (ADC1)
    input  wire       SDO_AD7922,
    output wire       SEN_AD7922,      // active-low
    output wire       SCLK_AD7922,
    output wire       SDI_AD7922,

    // AD7922_2 (ADC2)
    input  wire       SDO_AD7922_2,
    output wire       SEN_AD7922_2,    // active-low
    output wire       SCLK_AD7922_2,
    output wire       SDI_AD7922_2,

    // ATT control
    output wire ATT1_RX_C1,
    output wire ATT1_RX_C2,
    output wire ATT2_RX_LE,
    output wire ATT2_SCLK_RX,
    output wire ATT2_MOSI_RX,

    // GPIO cpld_io_00..15
    input  wire cpld_io_00, input  wire cpld_io_01, input  wire cpld_io_02, input  wire cpld_io_03,
    input  wire cpld_io_04, input  wire cpld_io_05, input  wire cpld_io_06, input  wire cpld_io_07,
    input  wire cpld_io_08, input  wire cpld_io_09, input  wire cpld_io_10, input  wire cpld_io_11,
    input  wire cpld_io_12, input  wire cpld_io_13, input  wire cpld_io_14, input  wire cpld_io_15,

    // LED / Power enable
    output wire LED_RX,
    output wire TPS_EN
);

    // --------------------------------------------------------------------
    // SPI_ADDR derived from GPIO pins (per your UHD driver map):
    // SPI_ADDR[2:0] = GPIO[2:0] = {io_02, io_01, io_00}
    // --------------------------------------------------------------------
    wire [2:0] spi_addr_from_gpio = {cpld_io_02, cpld_io_01, cpld_io_00};

    // --------------------------------------------------------------------
    // Internal interconnect
    // --------------------------------------------------------------------
    wire        cs_active;
    wire [5:0]  bitcnt;
    wire        evt_cmd_end;
    wire        evt_frame_end;
    wire [7:0]  cmd_byte_full;

    wire        is_write_lat;
    wire [2:0]  bank_lat;
    wire [3:0]  reg_lat;
    wire [23:0] wr_data_full;

    wire [2:0]  sel_latched;
    wire        sel_is_cpld;
    wire        err_invalid_sel;

    wire        clr_err_invalid_sel_evt;
    wire [23:0] rd_word_for_cmd;

    wire        miso_o;
    wire        miso_oe;

    // --------------------------------------------------------------------
    // SPI engine
    // NOTE: use spi_engine_clean.v (same module name "spi_engine")
    // --------------------------------------------------------------------
    spi_engine #(
        .CPLD_DEST(CPLD_DEST)
    ) u_spi (
        .sclk            (SCLK_RX),
        .cs_n            (SEN_RX),
        .mosi            (MOSI_RX),

        .spi_addr        (spi_addr_from_gpio),
        .rd_word_in      (rd_word_for_cmd),

        .cs_active       (cs_active),
        .bitcnt          (bitcnt),

        .evt_cmd_end     (evt_cmd_end),
        .evt_frame_end   (evt_frame_end),
        .cmd_byte_full   (cmd_byte_full),

        .is_write_lat    (is_write_lat),
        .bank_lat        (bank_lat),
        .reg_lat         (reg_lat),

        .wr_data_full    (wr_data_full),

        .sel_latched     (sel_latched),
        .sel_is_cpld     (sel_is_cpld),
        .err_invalid_sel (err_invalid_sel),

        .clr_err_invalid_sel_evt (clr_err_invalid_sel_evt),

        .miso_o          (miso_o),
        .miso_oe         (miso_oe)
    );

    // --------------------------------------------------------------------
    // Internal CPLD regmap + ATT2
    // --------------------------------------------------------------------
    regmap_core #(
        .ID0_CHAR   (ID0_CHAR),
        .ID1_CHAR   (ID1_CHAR),
        .ID2_CHAR   (ID2_CHAR),
        .VER_MAJOR  (VER_MAJOR),
        .VER_MINOR  (VER_MINOR),
        .VER_PATCH  (VER_PATCH)
    ) u_map (
        .sclk            (SCLK_RX),
        .cs_n            (SEN_RX),
        .mosi            (MOSI_RX),

        .cs_active       (cs_active),
        .bitcnt          (bitcnt),

        .evt_cmd_end     (evt_cmd_end),
        .evt_frame_end   (evt_frame_end),
        .cmd_byte_full   (cmd_byte_full),

        .is_write_lat    (is_write_lat),
        .bank_lat        (bank_lat),
        .reg_lat         (reg_lat),

        .wr_data_full    (wr_data_full),

        .sel_latched     (sel_latched),
        .sel_is_cpld     (sel_is_cpld),
        .err_invalid_sel (err_invalid_sel),

        .STAT_LTC6948    (STAT_LTC6948),

        .cpld_io_00(cpld_io_00), .cpld_io_01(cpld_io_01), .cpld_io_02(cpld_io_02), .cpld_io_03(cpld_io_03),
        .cpld_io_04(cpld_io_04), .cpld_io_05(cpld_io_05), .cpld_io_06(cpld_io_06), .cpld_io_07(cpld_io_07),
        .cpld_io_08(cpld_io_08), .cpld_io_09(cpld_io_09), .cpld_io_10(cpld_io_10), .cpld_io_11(cpld_io_11),
        .cpld_io_12(cpld_io_12), .cpld_io_13(cpld_io_13), .cpld_io_14(cpld_io_14), .cpld_io_15(cpld_io_15),

        //.TPS_EN         (TPS_EN),
        //.LED_RX         (LED_RX),

        .ATT1_RX_C1     (ATT1_RX_C1),
        .ATT1_RX_C2     (ATT1_RX_C2),

        .ATT2_RX_LE     (ATT2_RX_LE),
        .ATT2_SCLK_RX   (ATT2_SCLK_RX),
        .ATT2_MOSI_RX   (ATT2_MOSI_RX),

        .rd_word_for_cmd (rd_word_for_cmd),
        .clr_err_invalid_sel_evt (clr_err_invalid_sel_evt)
    );

    // --------------------------------------------------------------------
    // Transparent SPI routing to external devices (by sel_latched)
    // --------------------------------------------------------------------
    wire sel_ltc6948  = (sel_latched == DEST_LTC6948);
    wire sel_ltc5594  = (sel_latched == DEST_LTC5594);
    wire sel_ad7922   = (sel_latched == DEST_AD7922);
    wire sel_ad7922_2 = (sel_latched == DEST_AD7922_2);

    wire en_ltc6948   = cs_active && sel_ltc6948;
    wire en_ltc5594   = cs_active && sel_ltc5594;
    wire en_ad7922    = cs_active && sel_ad7922;
    wire en_ad7922_2  = cs_active && sel_ad7922_2;

    // CS/SEN active-low
    assign CS_LTC6948   = en_ltc6948  ? 1'b0 : 1'b1;
    assign SEN_LTC5594  = en_ltc5594  ? 1'b0 : 1'b1;
    assign SEN_AD7922   = en_ad7922   ? 1'b0 : 1'b1;
    assign SEN_AD7922_2 = en_ad7922_2 ? 1'b0 : 1'b1;

    // Forward SCLK/MOSI when enabled
    assign SCLK_LTC6948  = en_ltc6948  ? SCLK_RX : 1'b0;
    assign MOSI_LTC6948  = en_ltc6948  ? MOSI_RX : 1'b0;

    assign SCLK_LTC5594  = en_ltc5594  ? SCLK_RX : 1'b0;
    assign MOSI_LTC5594  = en_ltc5594  ? MOSI_RX : 1'b0;

    assign SCLK_AD7922   = en_ad7922   ? SCLK_RX : 1'b0;
    assign SDI_AD7922    = en_ad7922   ? MOSI_RX : 1'b0;

    assign SCLK_AD7922_2 = en_ad7922_2 ? SCLK_RX : 1'b0;
    assign SDI_AD7922_2  = en_ad7922_2 ? MOSI_RX : 1'b0;

    // --------------------------------------------------------------------
    // External MISO mux (used when CPLD is NOT selected)
    // --------------------------------------------------------------------
    reg ext_miso_val;
    reg ext_miso_valid;

    always @* begin
        ext_miso_val   = 1'b0;
        ext_miso_valid = 1'b0;

        if (sel_ltc6948) begin
            ext_miso_val   = MISO_LTC6948;
            ext_miso_valid = 1'b1;
        end else if (sel_ltc5594) begin
            ext_miso_val   = MISO_LTC5594;
            ext_miso_valid = 1'b1;
        end else if (sel_ad7922) begin
            ext_miso_val   = SDO_AD7922;
            ext_miso_valid = 1'b1;
        end else if (sel_ad7922_2) begin
            ext_miso_val   = SDO_AD7922_2;
            ext_miso_valid = 1'b1;
        end
    end

    wire ext_miso_oe = cs_active && ext_miso_valid && !sel_is_cpld;

    // Tri-state MISO on the physical pin
    assign MISO_RX = miso_oe     ? miso_o      :
                     ext_miso_oe ? ext_miso_val :
                     1'bZ;
							
							
							
	 assign TPS_EN = sel_ltc5594;
    assign LED_RX = en_ltc5594;

endmodule

`default_nettype wire
