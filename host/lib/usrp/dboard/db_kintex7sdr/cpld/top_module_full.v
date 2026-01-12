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
    parameter [2:0] DEST_AD7922_2  = 3'b100,
	 parameter [2:0] DEST_NONE			= 3'b111
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
	 input wire [15:0] CPLD_i,

    // LED / Power enable
    output wire LED_RX,
    output wire TPS_EN
);

    // --------------------------------------------------------------------
    // SPI_ADDR derived from GPIO pins (per your UHD driver map):
    // SPI_ADDR[2:0] = GPIO[2:0] = {io_02, io_01, io_00}
    // --------------------------------------------------------------------
    wire [2:0] spi_addr_from_gpio = CPLD_i[2:0];
	 
	 wire arst = CPLD_i[3];
	 
	reg [2:0] sel_latched = DEST_NONE;
	 
	     // Latch selector on CS falling edge (start of transaction)
    always @(negedge SEN_RX) begin
	  sel_latched <= arst ? DEST_NONE : spi_addr_from_gpio;
        // err_invalid_sel is sticky (cleared only by soft reset event)
    end
	
	wire SCLK_CPLD, SEN_CPLD, MOSI_CPLD, MISO_CPLD;
	
	// decode выбора
	wire sel_cpld     = (sel_latched == CPLD_DEST);
	wire sel_6948     = (sel_latched == DEST_LTC6948);
	wire sel_5594     = (sel_latched == DEST_LTC5594);
	wire sel_ad7922   = (sel_latched == DEST_AD7922);
	wire sel_ad7922_2 = (sel_latched == DEST_AD7922_2);

	// что считается "неактивным" для SEN/CS (обычно 1, если активный низ)
	localparam CS_INACTIVE = 1'b1;

	// Раздаём мастера на конкретный SPI-слейв (остальным держим безопасные уровни)
	assign SCLK_CPLD     = sel_cpld     ? SCLK_RX : 1'b0;
	assign SEN_CPLD      = sel_cpld     ? SEN_RX  : CS_INACTIVE;
	assign MOSI_CPLD     = sel_cpld     ? MOSI_RX : 1'b0;

	assign SCLK_LTC6948  = sel_6948     ? SCLK_RX : 1'b0;
	assign CS_LTC6948    = sel_6948     ? SEN_RX  : CS_INACTIVE;
	assign MOSI_LTC6948  = sel_6948     ? MOSI_RX : 1'b0;

	assign SCLK_LTC5594  = sel_5594     ? SCLK_RX : 1'b0;
	assign SEN_LTC5594   = sel_5594     ? SEN_RX  : CS_INACTIVE;
	assign MOSI_LTC5594  = sel_5594     ? MOSI_RX : 1'b0;

	assign SCLK_AD7922   = sel_ad7922   ? SCLK_RX : 1'b0;
	assign SEN_AD7922    = sel_ad7922   ? SEN_RX  : CS_INACTIVE;
	assign SDI_AD7922    = sel_ad7922   ? MOSI_RX : 1'b0;

	assign SCLK_AD7922_2 = sel_ad7922_2 ? SCLK_RX : 1'b0;
	assign SEN_AD7922_2  = sel_ad7922_2 ? SEN_RX  : CS_INACTIVE;
	assign SDI_AD7922_2  = sel_ad7922_2 ? MOSI_RX : 1'b0;

	// MISO mux (внутри не Z - просто выбираем 0 по умолчанию)
	wire miso_mux =
		 sel_cpld     ? MISO_CPLD     :
		 sel_6948     ? MISO_LTC6948  :
		 sel_5594     ? MISO_LTC5594  :
		 sel_ad7922   ? SDO_AD7922    :
		 sel_ad7922_2 ? SDO_AD7922_2  :
		 1'b0;

	// Tri-state на ФИЗИЧЕСКОМ пине MISO_RX.
	// Если SEN/CS активный НИЗ (типично), то драйвим MISO только когда SEN_RX==0:
	assign MISO_RX = (SEN_RX == 1'b0) ? miso_mux : 1'bZ;
							
	 
	  // --------------------------------------------------------------------
    // Internal interconnect
    // --------------------------------------------------------------------
    wire        cs_active = ~SEN_RX;
    wire [5:0]  bitcnt;
    wire        evt_cmd_end;
    wire        evt_frame_end;
    wire [7:0]  cmd_byte_full;

    wire        is_write_lat;
    wire [2:0]  bank_lat;
    wire [3:0]  reg_lat;
    wire [23:0] wr_data_full;

    wire        sel_is_cpld;
    wire        err_invalid_sel;

    wire        clr_err_invalid_sel_evt;
    wire [23:0] rd_word_for_cmd;

    //wire        miso_o;
    //wire        miso_oe;

    // --------------------------------------------------------------------
    // SPI engine
    // NOTE: use spi_engine_clean.v (same module name "spi_engine")
    // --------------------------------------------------------------------
    spi_engine #(
        .CPLD_DEST(CPLD_DEST)
    ) u_spi (
        .sclk            (SCLK_CPLD),
        .cs_n            (SEN_CPLD),
        .mosi            (MOSI_CPLD),
		  
		  .miso_o          (MISO_CPLD),
        .miso_oe         ( ),

        .spi_addr        (spi_addr_from_gpio),
        .sel_latched     (sel_latched),
        .sel_is_cpld     (sel_cpld),
		  
        .rd_word_in      (rd_word_for_cmd),
        .bitcnt          (bitcnt),

        .evt_cmd_end     (evt_cmd_end),
        .evt_frame_end   (evt_frame_end),
        .cmd_byte_full   (cmd_byte_full),

        .is_write_lat    (is_write_lat),
        .bank_lat        (bank_lat),
        .reg_lat         (reg_lat),

        .wr_data_full    (wr_data_full),

        .err_invalid_sel (err_invalid_sel),

        .clr_err_invalid_sel_evt (clr_err_invalid_sel_evt)
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

        .cpld_i(CPLD_i),

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
	 
	 
	 assign TPS_EN = (sel_latched == DEST_LTC5594);
    assign LED_RX = (sel_latched == DEST_NONE);


endmodule

`default_nettype wire
