//
// Copyright 2011 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

`timescale 1ns / 1ps
//`define LVDS 1
//`define DCM_FOR_RAMCLK
//////////////////////////////////////////////////////////////////////////////////

module u2plus
  (
   input CLK_FPGA_P, input CLK_FPGA_N,  // Diff
`ifdef ML605
   input CLK_SMA_P, input CLK_SMA_N,
   input CLK_IN_SEL,
   input DIP_2,
   input DIP_3,
   input DIP_4,
   input DIP_5,
   input DIP_6,
   input DIP_7,
   input DIP_8,
   input BUTTON_E,
   input BUTTON_S,
   input BUTTON_N,
   output LED_E,
   output LED_S,
   output LED_N,
   
   input  PLL_CLK_RESET,
   output PLL_CLK_LOCKED,
   input  PLL_DSP_RESET,
   output PLL_DSP_LOCKED,
   
   // ADC
   input ADC_DCO_p,input ADC_DCO_n,
   input ADC_OR_p,input ADC_OR_n,
   input ADC_11_p,input ADC_11_n,
   input ADC_10_p,input ADC_10_n,
   input ADC_9_p, input ADC_9_n,
   input ADC_8_p, input ADC_8_n,
   input ADC_7_p, input ADC_7_n,
   input ADC_6_p, input ADC_6_n,
   input ADC_5_p, input ADC_5_n,
   input ADC_4_p, input ADC_4_n,
   input ADC_3_p, input ADC_3_n,
   input ADC_2_p, input ADC_2_n,
   input ADC_1_p, input ADC_1_n,
   input ADC_0_p, input ADC_0_n,
   
   // DAC
   output DAC_DCI_p,output DAC_DCI_n,
   output DAC_FRAME_p,output DAC_FRAME_n,
   output DAC_15_p,output DAC_15_n,
   output DAC_14_p,output DAC_14_n,
   output DAC_13_p,output DAC_13_n,
   output DAC_12_p,output DAC_12_n,
   output DAC_11_p,output DAC_11_n,
   output DAC_10_p,output DAC_10_n,
   output DAC_9_p, output DAC_9_n,
   output DAC_8_p, output DAC_8_n,
   output DAC_7_p, output DAC_7_n,
   output DAC_6_p, output DAC_6_n,
   output DAC_5_p, output DAC_5_n,
   output DAC_4_p, output DAC_4_n,
   output DAC_3_p, output DAC_3_n,
   output DAC_2_p, output DAC_2_n,
   output DAC_1_p, output DAC_1_n,
   output DAC_0_p, output DAC_0_n,
`else
   
   // ADC
   input ADC_clkout_p, input ADC_clkout_n,
   input ADCA_12_p, input ADCA_12_n,
   input ADCA_10_p, input ADCA_10_n,
   input ADCA_8_p, input ADCA_8_n,
   input ADCA_6_p, input ADCA_6_n,
   input ADCA_4_p, input ADCA_4_n,
   input ADCA_2_p, input ADCA_2_n,
   input ADCA_0_p, input ADCA_0_n,
   input ADCB_12_p, input ADCB_12_n,
   input ADCB_10_p, input ADCB_10_n,
   input ADCB_8_p, input ADCB_8_n,
   input ADCB_6_p, input ADCB_6_n,
   input ADCB_4_p, input ADCB_4_n,
   input ADCB_2_p, input ADCB_2_n,
   input ADCB_0_p, input ADCB_0_n,
   
   // DAC
   output reg [15:0] DACA,
   output reg [15:0] DACB,
   input DAC_LOCK,     // unused for now
   
`endif // !`ifdef ML605

   // DB IO Pins
`ifdef ML605
   output [5:1] leds,  // LED4 is shared w/INIT_B
   input FPGA_RESET,
   output [2:0] debug,
   output TXD, input RXD, // UARTs
`else
   inout [15:0] io_tx,
   inout [15:0] io_rx,

   // Misc, debug
   output [5:1] leds,  // LED4 is shared w/INIT_B
   input FPGA_RESET,
   output [1:0] debug_clk,
   output [31:0] debug,
   output [3:1] TXD, input [3:1] RXD, // UARTs
   //input [3:0] dipsw,  // Forgot DIP Switches...
   
   // Clock Gen Control
   output [1:0] clk_en,
   output [1:0] clk_sel,
   input CLK_FUNC,        // FIXME is an input to control the 9510
   input clk_status,
`endif // !`ifdef ML605

   inout SCL, inout SDA,   // I2C

   // PPS
   input PPS_IN, input PPS2_IN,

   // SPI
`ifdef ML605
`else
   output SEN_CLK, output SCLK_CLK, output MOSI_CLK, input MISO_CLK,
   output SEN_DAC, output SCLK_DAC, output MOSI_DAC, input MISO_DAC,
   output SEN_ADC, output SCLK_ADC, output MOSI_ADC,
   output SEN_TX_DB, output SCLK_TX_DB, output MOSI_TX_DB, input MISO_TX_DB,
   output SEN_TX_DAC, output SCLK_TX_DAC, output MOSI_TX_DAC,
   output SEN_TX_ADC, output SCLK_TX_ADC, output MOSI_TX_ADC, input MISO_TX_ADC,
   output SEN_RX_DB, output SCLK_RX_DB, output MOSI_RX_DB, input MISO_RX_DB,
   output SEN_RX_DAC, output SCLK_RX_DAC, output MOSI_RX_DAC,
   output SEN_RX_ADC, output SCLK_RX_ADC, output MOSI_RX_ADC, input MISO_RX_ADC,
`endif // !`ifdef ML605

   // GigE PHY
`ifdef ML605
`else
   input CLK_TO_MAC,
`endif

   output reg [7:0] GMII_TXD,
   output reg GMII_TX_EN,
   output reg GMII_TX_ER,
   output GMII_GTX_CLK,
   input GMII_TX_CLK,  // 100mbps clk

   input GMII_RX_CLK,
   input [7:0] GMII_RXD,
   input GMII_RX_DV,
   input GMII_RX_ER,
   input GMII_COL,
   input GMII_CRS,

   input PHY_INTn,   // open drain
   inout MDIO,
   output MDC,
`ifdef ML605
   output PHY_RESETn
   );
`else
   output PHY_RESETn,
   output ETH_LED,
   
//   input POR,
   
   // Expansion
   input exp_time_in_p, input exp_time_in_n, // Diff
   output exp_time_out_p, output exp_time_out_n, // Diff 
   input exp_user_in_p, input exp_user_in_n, // Diff
   output exp_user_out_p, output exp_user_out_n, // Diff 
   
   // SERDES
   output ser_enable,
   output ser_prbsen,
   output ser_loopen,
   output ser_rx_en,
   
   output ser_tx_clk,
   output reg [15:0] ser_t,
   output reg ser_tklsb,
   output reg ser_tkmsb,

   input ser_rx_clk,
   input [15:0] ser_r,
   input ser_rklsb,
   input ser_rkmsb,

   // SRAM
   inout [35:0] RAM_D,
   output [20:0] RAM_A,
   output [3:0] RAM_BWn,
   output RAM_ZZ,
   output RAM_LDn,
   output RAM_OEn,
   output RAM_WEn,
   output RAM_CENn,
   output RAM_CLK,
   
   // SPI Flash
   output flash_cs,
   output flash_clk,
   output flash_mosi,
   input flash_miso
   );
`endif // !`ifdef ML605

`ifndef ML605
   wire  CLK_TO_MAC_int, CLK_TO_MAC_int2;
   IBUFG phyclk (.O(CLK_TO_MAC_int), .I(CLK_TO_MAC));
   BUFG phyclk2 (.O(CLK_TO_MAC_int2), .I(CLK_TO_MAC_int));
`else
   wire clk_to_mac;
`endif
      
   // FPGA-specific pins connections
   wire 	clk_fpga, dsp_clk, clk_div, dcm_out, wb_clk, clock_ready;

`ifndef ML605
   IBUFGDS clk_fpga_pin (.O(clk_fpga),.I(CLK_FPGA_P),.IB(CLK_FPGA_N));
   defparam 	clk_fpga_pin.IOSTANDARD = "LVPECL_25";
   
   wire 	exp_time_in;
   IBUFDS exp_time_in_pin (.O(exp_time_in),.I(exp_time_in_p),.IB(exp_time_in_n));
   defparam 	exp_time_in_pin.IOSTANDARD = "LVDS_25";
   
   wire 	exp_time_out;
   OBUFDS exp_time_out_pin (.O(exp_time_out_p),.OB(exp_time_out_n),.I(exp_time_out));
   defparam 	exp_time_out_pin.IOSTANDARD  = "LVDS_25";

   wire 	exp_user_in;
   IBUFDS exp_user_in_pin (.O(exp_user_in),.I(exp_user_in_p),.IB(exp_user_in_n));
   defparam 	exp_user_in_pin.IOSTANDARD = "LVDS_25";
   
   wire 	exp_user_out;
   OBUFDS exp_user_out_pin (.O(exp_user_out_p),.OB(exp_user_out_n),.I(exp_user_out));
   defparam 	exp_user_out_pin.IOSTANDARD  = "LVDS_25";
`endif

   reg [5:0] 	clock_ready_d;
   always @(posedge clk_fpga)
     clock_ready_d[5:0] <= {clock_ready_d[4:0],clock_ready};
   wire 	dcm_rst = ~&clock_ready_d & |clock_ready_d;

   // ADC A is inverted on the schematic to facilitate a clean layout
   //  We account for that here by inverting it
`ifndef ML605
`ifdef LVDS
   wire [13:0] 	adc_a, adc_a_inv, adc_b;
   capture_ddrlvds #(.WIDTH(14)) capture_ddrlvds
     (.clk(dsp_clk), .ssclk_p(ADC_clkout_p), .ssclk_n(ADC_clkout_n), 
      .in_p({{ADCA_12_p, ADCA_10_p, ADCA_8_p, ADCA_6_p, ADCA_4_p, ADCA_2_p, ADCA_0_p},
	     {ADCB_12_p, ADCB_10_p, ADCB_8_p, ADCB_6_p, ADCB_4_p, ADCB_2_p, ADCB_0_p}}), 
      .in_n({{ADCA_12_n, ADCA_10_n, ADCA_8_n, ADCA_6_n, ADCA_4_n, ADCA_2_n, ADCA_0_n},
	     {ADCB_12_n, ADCB_10_n, ADCB_8_n, ADCB_6_n, ADCB_4_n, ADCB_2_n, ADCB_0_n}}), 
      .out({adc_a_inv,adc_b}));
   assign adc_a = ~adc_a_inv;
`else
   reg [13:0] 	adc_a, adc_b, adc_a_pre, adc_b_pre;
   always @(posedge dsp_clk)
     begin
	adc_a_pre <= {ADCA_12_p,ADCA_12_n, ADCA_10_p,ADCA_10_n, ADCA_8_p,ADCA_8_n, ADCA_6_p,ADCA_6_n,
		   ADCA_4_p,ADCA_4_n, ADCA_2_p,ADCA_2_n, ADCA_0_p,ADCA_0_n };
	adc_b_pre <= {ADCB_12_p,ADCB_12_n, ADCB_10_p,ADCB_10_n, ADCB_8_p,ADCB_8_n, ADCB_6_p,ADCB_6_n,
		   ADCB_4_p,ADCB_4_n, ADCB_2_p,ADCB_2_n, ADCB_0_p,ADCB_0_n };
	adc_a <= ~adc_a_pre; //Note: A must be inverted, but not B
	adc_b <= adc_b_pre;
     end
`endif // !`ifdef LVDS
`else
   wire [11:0]     adc_a;
   capture_ad9434 #(.WIDTH(12)) capture_ad9434
     (.clk(dsp_clk), .ssclk_p(ADC_DCO_p), .ssclk_n(ADC_DCO_n), 
      .in_p({ ADC_11_p, ADC_10_p, ADC_9_p, ADC_8_p, ADC_7_p, ADC_6_p, ADC_5_p, ADC_4_p, ADC_3_p, ADC_2_p, ADC_1_p, ADC_0_p}),
      .in_n({ ADC_11_n, ADC_10_n, ADC_9_n, ADC_8_n, ADC_7_n, ADC_6_n, ADC_5_n, ADC_4_n, ADC_3_n, ADC_2_n, ADC_1_n, ADC_0_n}),
      .out(adc_a));
`endif
   
`ifndef ML605
   // Handle Clocks
   DCM DCM_INST (.CLKFB(dsp_clk), 
                 .CLKIN(clk_fpga), 
                 .DSSEN(0), 
                 .PSCLK(0), 
                 .PSEN(0), 
                 .PSINCDEC(0), 
                 .RST(dcm_rst), 
                 .CLKDV(clk_div), 
                 .CLKFX(), 
                 .CLKFX180(), 
                 .CLK0(dcm_out), 
                 .CLK2X(), 
                 .CLK2X180(), 
                 .CLK90(), 
                 .CLK180(), 
                 .CLK270(clk270_100), 
                 .LOCKED(LOCKED_OUT), 
                 .PSDONE(), 
                 .STATUS());
   defparam DCM_INST.CLK_FEEDBACK = "1X";
   defparam DCM_INST.CLKDV_DIVIDE = 2.0;
   defparam DCM_INST.CLKFX_DIVIDE = 1;
   defparam DCM_INST.CLKFX_MULTIPLY = 4;
   defparam DCM_INST.CLKIN_DIVIDE_BY_2 = "FALSE";
   defparam DCM_INST.CLKIN_PERIOD = 10.000;
   defparam DCM_INST.CLKOUT_PHASE_SHIFT = "NONE";
   defparam DCM_INST.DESKEW_ADJUST = "SYSTEM_SYNCHRONOUS";
   defparam DCM_INST.DFS_FREQUENCY_MODE = "LOW";
   defparam DCM_INST.DLL_FREQUENCY_MODE = "LOW";
   defparam DCM_INST.DUTY_CYCLE_CORRECTION = "TRUE";
   defparam DCM_INST.FACTORY_JF = 16'h8080;
   defparam DCM_INST.PHASE_SHIFT = 0;
   defparam DCM_INST.STARTUP_WAIT = "FALSE";

   BUFG dspclk_BUFG (.I(dcm_out), .O(dsp_clk));
   BUFG wbclk_BUFG (.I(clk_div), .O(wb_clk));
`else
wire dsp_clk_100M;

   pll_clk pll_clk_instance
   (// Clock in ports
    .CLK_IN1_P(CLK_FPGA_P),      // IN
    .CLK_IN1_N(CLK_FPGA_N),
    .RESET(PLL_CLK_RESET),
    // Clock out ports
    .LOCKED(PLL_CLK_LOCKED),
    .CLK_OUT1(),     // OUT 50 MHz
    .CLK_OUT2(clk_to_mac),     // OUT 125 MHz
    .CLK_OUT3(dsp_clk_100M),     // OUT 100 MHz
    .CLK_OUT4(),     // OUT 100 MHz, clk270_100
    .CLK_OUT5(clk_fpga)     // OUT 100 MHz
    );     

wire clk_rx, clk_rx_180;
   pll_rx pll_rx_instance
   (// Clock in ports
    .CLK_IN1(GMII_RX_CLK),      // IN
    // Clock out ports
    .CLK_OUT1(clk_rx),     // OUT
    .CLK_OUT2(clk_rx_180));    // OUT

   pll_dsp pll_dsp_instance
   (// Clock in ports
    .CLK_IN1_P(CLK_SMA_P),  // IN 100MHz
    .CLK_IN1_N(CLK_SMA_N),  // IN 100MHz
    .RESET(PLL_DSP_RESET),
    // Clock out ports
    .LOCKED(PLL_DSP_LOCKED),
    .CLK_OUT1(dsp_clk),    // OUT 122.88MHz
    .CLK_OUT2(wb_clk));    // OUT 61.44MHz

`endif

`ifndef ML605
   // Create clock for external SRAM thats -90degree phase to DSPCLK (i.e) 2nS earlier at 100MHz.
   BUFG  clk270_100_buf_i1 (.I(clk270_100), 
			    .O(clk270_100_buf));
   OFDDRRSE RAM_CLK_i1 (.Q(RAM_CLK),
			.C0(clk270_100_buf),
			.C1(~clk270_100_buf),
			.CE(1'b1),
			.D0(1'b1),
			.D1(1'b0),
			.R(1'b0),
			.S(1'b0));

`endif
  
   // I2C -- Don't use external transistors for open drain, the FPGA implements this
   IOBUF scl_pin(.O(scl_pad_i), .IO(SCL), .I(scl_pad_o), .T(scl_pad_oen_o));
   IOBUF sda_pin(.O(sda_pad_i), .IO(SDA), .I(sda_pad_o), .T(sda_pad_oen_o));

   // LEDs are active low outputs
   wire [5:0] leds_int;
`ifndef ML605
   assign     {ETH_LED,leds} = {6'b011111 ^ leds_int};  // drive low to turn on leds
`else
   assign     {ETH_LED,leds} = {6'b000000 ^ leds_int};  // drive low to turn on leds
`endif
   
   // SPI
`ifndef ML605
   wire       miso, mosi, sclk;

   assign 	{SCLK_CLK,MOSI_CLK} 	   = ~SEN_CLK ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_DAC,MOSI_DAC} 	   = ~SEN_DAC ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_ADC,MOSI_ADC} 	   = ~SEN_ADC ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_TX_DB,MOSI_TX_DB}    = ~SEN_TX_DB ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_TX_DAC,MOSI_TX_DAC}  = ~SEN_TX_DAC ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_TX_ADC,MOSI_TX_ADC}  = ~SEN_TX_ADC ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_RX_DB,MOSI_RX_DB}    = ~SEN_RX_DB ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_RX_DAC,MOSI_RX_DAC}  = ~SEN_RX_DAC ? {sclk,mosi} : 2'B0;
   assign 	{SCLK_RX_ADC,MOSI_RX_ADC}  = ~SEN_RX_ADC ? {sclk,mosi} : 2'B0;
   
   assign 	miso 			   = (~SEN_CLK & MISO_CLK) | (~SEN_DAC & MISO_DAC) |
					     (~SEN_TX_DB & MISO_TX_DB) | (~SEN_TX_ADC & MISO_TX_ADC) |
					     (~SEN_RX_DB & MISO_RX_DB) | (~SEN_RX_ADC & MISO_RX_ADC);
   
`endif
   wire 	GMII_TX_EN_unreg, GMII_TX_ER_unreg;
   wire [7:0] 	GMII_TXD_unreg;
   wire 	GMII_GTX_CLK_int;
   
   always @(posedge GMII_GTX_CLK_int)
     begin
	GMII_TX_EN <= GMII_TX_EN_unreg;
	GMII_TX_ER <= GMII_TX_ER_unreg;
	GMII_TXD <= GMII_TXD_unreg;
     end

`ifndef ML605
   OFDDRRSE OFDDRRSE_gmii_inst 
     (.Q(GMII_GTX_CLK),      // Data output (connect directly to top-level port)
      .C0(GMII_GTX_CLK_int),    // 0 degree clock input
      .C1(~GMII_GTX_CLK_int),    // 180 degree clock input
      .CE(1),    // Clock enable input
      .D0(0),    // Posedge data input
      .D1(1),    // Negedge data input
      .R(0),      // Synchronous reset input
      .S(0)       // Synchronous preset input
      );
`else
    ODDR #(
        .DDR_CLK_EDGE("OPPOSITE_EDGE"), // "OPPOSITE_EDGE" or "SAME_EDGE"
        .INIT(1'b0), // Initial value of Q: 1'b0 or 1'b1
        .SRTYPE("SYNC") // Set/Reset type: "SYNC" or "ASYNC"
    ) ODDR_gmii_inst (
    .Q(GMII_GTX_CLK), // 1-bit DDR output
    .C(GMII_GTX_CLK_int), // 1-bit clock input
    .CE(1'b1), // 1-bit clock enable input
    .D1(1'b0), // 1-bit data input (positive edge)
    .D2(1'b1), // 1-bit data input (negative edge)
    .R(1'b0), // 1-bit reset
    .S(1'b0) // 1-bit set
    );

   wire [7:0] GMII_RXD_buf_n ,GMII_RXD_buf;
   wire GMII_RX_DV_buf, GMII_RX_DV_buf_n, GMII_RX_ER_buf, GMII_RX_ER_buf_n;

    IDDR erx_dv_ifd
    (
      .Q1( GMII_RX_DV_buf), // 1-bit output captured with C0 clock
      .Q2 ( GMII_RX_DV_buf_n), // 1-bit output captured with C1 clock
      .C ( clk_rx), // 1-bit clock input
      .CE ( 1),  // 1-bit clock enable input
      .D ( GMII_RX_DV),   // 1-bit data input
      .R ( 0),      // Synchronous reset input
      .S ( 0)      // Synchronous preset input
      );
   IDDR erx_er_ifd
    (
      .Q1 ( GMII_RX_ER_buf), // 1-bit output captured with C0 clock
      .Q2 ( GMII_RX_ER_buf_n), // 1-bit output captured with C1 clock
      .C ( clk_rx), // 1-bit clock input
      .CE ( 1),  // 1-bit clock enable input
      .D ( GMII_RX_ER),   // 1-bit data input
      .R ( 0),      // Synchronous reset input
      .S ( 0)      // Synchronous preset input
    );

   IDDR erxd_0_ifd
    (
      .Q1 ( GMII_RXD_buf[0]), // 1-bit output captured with C0 clock
      .Q2 ( GMII_RXD_buf_n[0]), // 1-bit output captured with C1 clock
      .C ( clk_rx), // 1-bit clock input
      .CE ( 1),  // 1-bit clock enable input
      .D ( GMII_RXD[0]),   // 1-bit data input
      .R ( 0),      // Synchronous reset input
      .S ( 0)      // Synchronous preset input
      );
   IDDR erxd_1_ifd
    (
      .Q1 ( GMII_RXD_buf[1]), // 1-bit output captured with C0 clock
      .Q2 ( GMII_RXD_buf_n[1]), // 1-bit output captured with C1 clock
      .C ( clk_rx), // 1-bit clock input
      .CE ( 1),  // 1-bit clock enable input
      .D ( GMII_RXD[1]),   // 1-bit data input
      .R ( 0),      // Synchronous reset input
      .S ( 0)      // Synchronous preset input
      );

   IDDR erxd_2_ifd
    (
      .Q1 ( GMII_RXD_buf[2]), // 1-bit output captured with C0 clock
      .Q2 ( GMII_RXD_buf_n[2]), // 1-bit output captured with C1 clock
      .C ( clk_rx), // 1-bit clock input
      .CE ( 1),  // 1-bit clock enable input
      .D ( GMII_RXD[2]),   // 1-bit data input
      .R ( 0),      // Synchronous reset input
      .S ( 0)      // Synchronous preset input
      );
   IDDR erxd_3_ifd
   (
      .Q1 ( GMII_RXD_buf[3]), // 1-bit output captured with C0 clock
      .Q2 ( GMII_RXD_buf_n[3]), // 1-bit output captured with C1 clock
      .C ( clk_rx), // 1-bit clock input
      .CE ( 1),  // 1-bit clock enable input
      .D ( GMII_RXD[3]),   // 1-bit data input
      .R ( 0),      // Synchronous reset input
      .S ( 0)      // Synchronous preset input
      );
   IDDR erxd_4_ifd
    (
      .Q1 ( GMII_RXD_buf[4]), // 1-bit output captured with C0 clock
      .Q2 ( GMII_RXD_buf_n[4]), // 1-bit output captured with C1 clock
      .C ( clk_rx), // 1-bit clock input
      .CE ( 1),  // 1-bit clock enable input
      .D ( GMII_RXD[4]),   // 1-bit data input
      .R ( 0),      // Synchronous reset input
      .S ( 0)      // Synchronous preset input
      );
   IDDR erxd_5_ifd
   (
      .Q1 ( GMII_RXD_buf[5]), // 1-bit output captured with C0 clock
      .Q2 ( GMII_RXD_buf_n[5]), // 1-bit output captured with C1 clock
      .C ( clk_rx), // 1-bit clock input
      .CE ( 1),  // 1-bit clock enable input
      .D ( GMII_RXD[5]),   // 1-bit data input
      .R ( 0),      // Synchronous reset input
      .S ( 0)      // Synchronous preset input
      );
   IDDR erxd_6_ifd
    (
      .Q1 ( GMII_RXD_buf[6]), // 1-bit output captured with C0 clock
      .Q2 ( GMII_RXD_buf_n[6]), // 1-bit output captured with C1 clock
      .C ( clk_rx), // 1-bit clock input
      .CE ( 1),  // 1-bit clock enable input
      .D ( GMII_RXD[6]),   // 1-bit data input
      .R ( 0),      // Synchronous reset input
      .S ( 0)      // Synchronous preset input
      );
   IDDR erxd_7_ifd
    (
      .Q1 ( GMII_RXD_buf[7]), // 1-bit output captured with C0 clock
      .Q2 ( GMII_RXD_buf_n[7]), // 1-bit output captured with C1 clock
      .C ( clk_rx), // 1-bit clock input
      .CE ( 1),  // 1-bit clock enable input
      .D ( GMII_RXD[7]),   // 1-bit data input
      .R ( 0),      // Synchronous reset input
      .S ( 0)      // Synchronous preset input
      );

`endif
   
`ifndef ML605
   wire ser_tklsb_unreg, ser_tkmsb_unreg;
   wire [15:0] ser_t_unreg;
   wire        ser_tx_clk_int;
   
   always @(posedge ser_tx_clk_int)
     begin
	ser_tklsb <= ser_tklsb_unreg;
	ser_tkmsb <= ser_tkmsb_unreg;
	ser_t <= ser_t_unreg;
     end

   assign ser_tx_clk = clk_fpga;

   reg [15:0] ser_r_int;
   reg 	      ser_rklsb_int, ser_rkmsb_int;

   always @(posedge ser_rx_clk)
     begin
	ser_r_int <= ser_r;
	ser_rklsb_int <= ser_rklsb;
	ser_rkmsb_int <= ser_rkmsb;
     end
`endif
   
   /*
   OFDDRRSE OFDDRRSE_serdes_inst 
     (.Q(ser_tx_clk),      // Data output (connect directly to top-level port)
      .C0(ser_tx_clk_int),    // 0 degree clock input
      .C1(~ser_tx_clk_int),    // 180 degree clock input
      .CE(1),    // Clock enable input
      .D0(0),    // Posedge data input
      .D1(1),    // Negedge data input
      .R(0),      // Synchronous reset input
      .S(0)       // Synchronous preset input
      );
   */


`ifndef ML605
   //
   // Instantiate IO for Bidirectional bus to SRAM
   //
   wire [35:0] RAM_D_pi;
   wire [35:0] RAM_D_po;
   wire        RAM_D_poe;
   
   genvar      i;
   
   generate  
      for (i=0;i<36;i=i+1)
        begin : gen_RAM_D_IO

	   IOBUF #(
		   .DRIVE(12),
		   .IOSTANDARD("LVCMOS25"),
		   .SLEW("FAST")
		   )
	     RAM_D_i (
		      .O(RAM_D_pi[i]),
		      .I(RAM_D_po[i]),
		      .IO(RAM_D[i]),
		      .T(RAM_D_poe)
		      );
	end // block: gen_RAM_D_IO
   endgenerate
`endif

   
   
   wire [15:0] dac_a_int, dac_b_int;
   // DAC A and B are swapped in schematic to facilitate clean layout
   // DAC A is also inverted in schematic to facilitate clean layout
`ifndef ML605
   always @(negedge dsp_clk) DACA <= ~dac_b_int;
   always @(negedge dsp_clk) DACB <= dac_a_int;
`endif

   wire 	pps;
   assign pps = PPS_IN ^ PPS2_IN;

`ifdef ML605
    transmit_ad9122 #(.WIDTH(16)) transmit_inst (
            .clk(dsp_clk),
            .in_a(dac_a_int),
            .in_b(dac_b_int),
            .dac_dci_out_p(DAC_DCI_p),
            .dac_dci_out_n(DAC_DCI_n),
            .dac_frame_out_p(DAC_FRAME_p),
            .dac_frame_out_n(DAC_FRAME_n),
            .delay_value({DIP_4,DIP_5,DIP_6,DIP_7,DIP_8}),
            .delay_reset(BUTTON_E),
            .out_p({DAC_15_p,DAC_14_p,DAC_13_p,DAC_12_p,DAC_11_p,DAC_10_p,DAC_9_p,DAC_8_p,DAC_7_p,DAC_6_p,DAC_5_p,DAC_4_p,DAC_3_p,DAC_2_p,DAC_1_p,DAC_0_p}),
            .out_n({DAC_15_n,DAC_14_n,DAC_13_n,DAC_12_n,DAC_11_n,DAC_10_n,DAC_9_n,DAC_8_n,DAC_7_n,DAC_6_n,DAC_5_n,DAC_4_n,DAC_3_n,DAC_2_n,DAC_1_n,DAC_0_n}));

   wire   PHY_RESETn_internal;
`endif
   
   u2plus_core u2p_c(.dsp_clk           (dsp_clk),
		     .wb_clk            (wb_clk),
		     .clock_ready       (clock_ready),
`ifndef ML605
		     .clk_to_mac	(CLK_TO_MAC_int2),
`else
                     .clk_to_mac        (clk_to_mac),
`endif
		     .pps_in		(pps),
		     .leds		(leds_int),
`ifndef ML605
		     .debug		(debug[31:0]),
		     .debug_clk		(debug_clk[1:0]),
		     .exp_time_in	(exp_time_in),
		     .exp_time_out	(exp_time_out),
`endif
		     .GMII_COL		(GMII_COL),
		     .GMII_CRS		(GMII_CRS),
		     .GMII_TXD		(GMII_TXD_unreg[7:0]),
		     .GMII_TX_EN	(GMII_TX_EN_unreg),
		     .GMII_TX_ER	(GMII_TX_ER_unreg),
		     .GMII_GTX_CLK	(GMII_GTX_CLK_int),
		     .GMII_TX_CLK	(GMII_TX_CLK),
`ifndef ML605
		     .GMII_RXD		(GMII_RXD[7:0]),
		     .GMII_RX_CLK	(GMII_RX_CLK),
		     .GMII_RX_DV	(GMII_RX_DV),
		     .GMII_RX_ER	(GMII_RX_ER),
`else
                     .GMII_RXD          (GMII_RXD_buf[7:0]),
                     .GMII_RX_CLK       (clk_rx),
                     .GMII_RX_DV        (GMII_RX_DV_buf),
                     .GMII_RX_ER        (GMII_RX_ER_buf),
`endif
		     .MDIO		(MDIO),
		     .MDC		(MDC),
		     .PHY_INTn		(PHY_INTn),
`ifndef ML605
		     .PHY_RESETn	(PHY_RESETn),
		     .ser_enable	(ser_enable),
		     .ser_prbsen	(ser_prbsen),
		     .ser_loopen	(ser_loopen),
		     .ser_rx_en		(ser_rx_en),
		     .ser_tx_clk	(ser_tx_clk_int),
		     .ser_t		(ser_t_unreg[15:0]),
		     .ser_tklsb		(ser_tklsb_unreg),
		     .ser_tkmsb		(ser_tkmsb_unreg),
		     .ser_rx_clk	(ser_rx_clk),
		     .ser_r		(ser_r_int[15:0]),
		     .ser_rklsb		(ser_rklsb_int),
		     .ser_rkmsb		(ser_rkmsb_int),
		     .adc_a		(adc_a[13:0]),
		     .adc_ovf_a		(1'b0),
		     .adc_on_a		(),
		     .adc_oe_a		(),
		     .adc_b		(adc_b[13:0]),
		     .adc_ovf_b		(1'b0),
`else
                     .PHY_RESETn        (PHY_RESETn_internal),
                     .adc_a        ({adc_a[11:0],2'b0}),
                     .adc_ovf_a        (1'b0),
                     .adc_on_a        (),
                     .adc_oe_a        (),
                     .adc_b        ({adc_a[11:0],2'b0}),
                     .adc_ovf_b        (1'b0),
`endif
		     .adc_on_b		(),
		     .adc_oe_b		(),
		     .dac_a		(dac_a_int[15:0]),
		     .dac_b		(dac_b_int[15:0]),
		     .scl_pad_i		(scl_pad_i),
		     .scl_pad_o		(scl_pad_o),
		     .scl_pad_oen_o	(scl_pad_oen_o),
		     .sda_pad_i		(sda_pad_i),
		     .sda_pad_o		(sda_pad_o),
		     .sda_pad_oen_o	(sda_pad_oen_o),
`ifndef ML605
		     .clk_en		(clk_en[1:0]),
		     .clk_sel		(clk_sel[1:0]),
		     .clk_func		(clk_func),
		     .clk_status	(clk_status),
		     .sclk		(sclk),
		     .mosi		(mosi),
		     .miso		(miso),
		     .sen_clk		(SEN_CLK),
		     .sen_dac		(SEN_DAC),
		     .sen_adc           (SEN_ADC),
		     .sen_tx_db		(SEN_TX_DB),
		     .sen_tx_adc	(SEN_TX_ADC),
		     .sen_tx_dac	(SEN_TX_DAC),
		     .sen_rx_db		(SEN_RX_DB),
		     .sen_rx_adc	(SEN_RX_ADC),
		     .sen_rx_dac	(SEN_RX_DAC),
		     .io_tx		(io_tx[15:0]),
		     .io_rx		(io_rx[15:0]),
		     .RAM_D_po          (RAM_D_po),
		     .RAM_D_pi          (RAM_D_pi),
		     .RAM_D_poe         (RAM_D_poe),
		     .RAM_A             (RAM_A),
		     .RAM_CE1n          (RAM_CE1n),
		     .RAM_CENn          (RAM_CENn),
		     .RAM_WEn           (RAM_WEn),
		     .RAM_OEn           (RAM_OEn),
		     .RAM_LDn           (RAM_LDn), 
		     .uart_tx_o         (TXD[3:1]),
		     .uart_rx_i         ({1'b1,RXD[3:1]}),
		     .uart_baud_o       (),
		     .sim_mode          (1'b0),
		     .clock_divider     (2),
		     .button            (FPGA_RESET),
		     .spiflash_cs       (flash_cs),
		     .spiflash_clk      (flash_clk),
		     .spiflash_miso     (flash_miso),
		     .spiflash_mosi     (flash_mosi)
		     );
`else
                     .uart_tx_o         (TXD),
                     .uart_rx_i         ({3'b111,RXD}),
                     .uart_baud_o       (),
                     .sim_mode          (1'b0),
                     .clock_divider     (2),
                     .button            (1'b0)
                     );
`endif

`ifndef ML605
   // Drive low so that RAM does not sleep.
   assign RAM_ZZ = 0;
   // Byte Writes are qualified by the global write enable
   // Always do 36bit operations to extram.
   assign RAM_BWn = 4'b0000;
`else
   assign debug[0] = PHY_RESETn;
   assign debug[1] = PHY_INTn;
   assign debug[2] = FPGA_RESET;
   assign PHY_RESETn = (~FPGA_RESET)&(PHY_RESETn_internal);
   assign LED_E = 1'b0;
   assign LED_S = 1'b0;
   assign LED_N = 1'b0;
`endif   
endmodule // u2plus
