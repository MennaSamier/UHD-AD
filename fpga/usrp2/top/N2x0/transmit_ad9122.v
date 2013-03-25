//
// Copyright 2013 Analog Devices Inc
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

module transmit_ad9122
  #(parameter WIDTH=16)
   (input clk,
    input delay_value,
    input delay_reset,
    output dac_dci_out_p,
    output dac_dci_out_n,
    output dac_frame_out_p,
    output dac_frame_out_n,
    output [WIDTH-1:0] out_p,
    output [WIDTH-1:0] out_n,
    input [WIDTH-1:0] in_a,
    input [WIDTH-1:0] in_b);

   wire [WIDTH-1:0]  ddr_dat;
   wire [WIDTH-1:0]  ddr_dat_delay;
   wire dci_tmp;
   wire dci_delay;
   wire frame_tmp;
   wire [4:0] delay_value;


//DATA
   genvar i;
   generate
      for(i = 0; i < WIDTH; i = i + 1)
      begin : gen_lvds_pins
       ODDR #(
       .DDR_CLK_EDGE("OPPOSITE_EDGE"), // "OPPOSITE_EDGE" or "SAME_EDGE"
       .INIT(1'b0), // Initial value of Q: 1’b0 or 1’b1
       .SRTYPE("SYNC") // Set/Reset type: "SYNC" or "ASYNC"
       ) ODDR_inst (
       .Q(ddr_dat[i]), // 1-bit DDR output
       .C(clk), // 1-bit clock input
       .CE(1'b1), // 1-bit clock enable input
       .D1(in_a[i]), // 1-bit data input (positive edge)
       .D2(in_b[i]), // 1-bit data input (negative edge)
       .R(1'b0), // 1-bit reset
       .S(1'b0) // 1-bit set
       );

(* IODELAY_GROUP = "dci_group" *) // Specifies group name for associated IODELAYs and IDELAYCTRL
IODELAYE1 #(
           .CINVCTRL_SEL("FALSE"), // Enable dynamic clock inversion ("TRUE"/"FALSE")
           .DELAY_SRC("O"), // Delay input ("I", "CLKIN", "DATAIN", "IO", "O")
           .HIGH_PERFORMANCE_MODE("TRUE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
           .IDELAY_TYPE("FIXED"), // "DEFAULT", "FIXED", "VARIABLE", or "VAR_LOADABLE"
           .IDELAY_VALUE(0), // Input delay tap setting (0-32)
           .ODELAY_TYPE("VARIABLE"), // "FIXED", "VARIABLE", or "VAR_LOADABLE"
           .ODELAY_VALUE(0), // Output delay tap setting (0-32)
           .REFCLK_FREQUENCY(200.0), // IDELAYCTRL clock input frequency in MHz
           .SIGNAL_PATTERN("CLOCK") // "DATA" or "CLOCK" input signal
           )
IODELAYE1_inst (
           .CNTVALUEOUT(), // 5-bit output - Counter value for monitoring purpose
           .DATAOUT(ddr_dat_delay[i]), // 1-bit output - Delayed data output
           .C(clk), // 1-bit input - Clock input
           .CE(1'b0), // 1-bit input - Active high enable increment/decrement function
           .CINVCTRL(1'b0), // 1-bit input - Dynamically inverts the Clock (C) polarity
           .CLKIN(clk), // 1-bit input - Clock Access into the IODELAY
           .CNTVALUEIN(5'b00000), // 5-bit input - Counter value for loadable counter application
           .DATAIN(1'b0), // 1-bit input - Internal delay data
           .IDATAIN(1'b0), // 1-bit input - Delay data input
           .INC(1'b0), // 1-bit input - Increment / Decrement tap delay
           .ODATAIN(ddr_dat[i]), // 1-bit input - Data input for the output datapath from the device
           .RST(1'b0), // 1-bit input - Active high, synchronous reset, resets delay chain to IDELAY_VALUE / ODELAY_VALUE tap. If no value is specified, the default is 0.
           .T(1'b0) // 1-bit input - 3-state input control. Tie high for input-only or internal delay or tie low for output only.
           );

       OBUFDS #(.IOSTANDARD("LVDS_25")) obufds 
        (.I(ddr_dat_delay[i]), .O(out_p[i]), .OB(out_n[i]) );
      end
   endgenerate

//DCI
       ODDR #(
       .DDR_CLK_EDGE("OPPOSITE_EDGE"), // "OPPOSITE_EDGE" or "SAME_EDGE"
       .INIT(1'b0), // Initial value of Q: 1’b0 or 1’b1
       .SRTYPE("SYNC") // Set/Reset type: "SYNC" or "ASYNC"
       ) ODDR_inst_dci (
       .Q(dci_tmp), // 1-bit DDR output
       .C(clk), // 1-bit clock input
       .CE(1'b1), // 1-bit clock enable input
       .D1(1'b1), // 1-bit data input (positive edge)
       .D2(1'b0), // 1-bit data input (negative edge)
       .R(1'b0), // 1-bit reset
       .S(1'b0) // 1-bit set
       );

(* IODELAY_GROUP = "dci_group" *) // Specifies group name for associated IODELAYs and IDELAYCTRL
IODELAYE1 #(
           .CINVCTRL_SEL("FALSE"), // Enable dynamic clock inversion ("TRUE"/"FALSE")
           .DELAY_SRC("O"), // Delay input ("I", "CLKIN", "DATAIN", "IO", "O")
           .HIGH_PERFORMANCE_MODE("TRUE"), // Reduced jitter ("TRUE"), Reduced power ("FALSE")
           .IDELAY_TYPE("FIXED"), // "DEFAULT", "FIXED", "VARIABLE", or "VAR_LOADABLE"
           .IDELAY_VALUE(0), // Input delay tap setting (0-32)
           .ODELAY_TYPE("VAR_LOADABLE"), // "FIXED", "VARIABLE", or "VAR_LOADABLE"
           .ODELAY_VALUE(0), // Output delay tap setting (0-32)
           .REFCLK_FREQUENCY(200.0), // IDELAYCTRL clock input frequency in MHz
           .SIGNAL_PATTERN("CLOCK") // "DATA" or "CLOCK" input signal
           )
IODELAYE1_dci_inst (
           .CNTVALUEOUT(), // 5-bit output - Counter value for monitoring purpose
           .DATAOUT(dci_delay), // 1-bit output - Delayed data output
           .C(clk), // 1-bit input - Clock input
           .CE(1'b0), // 1-bit input - Active high enable increment/decrement function
           .CINVCTRL(1'b0), // 1-bit input - Dynamically inverts the Clock (C) polarity
           .CLKIN(clk), // 1-bit input - Clock Access into the IODELAY
           .CNTVALUEIN(delay_value), // 5-bit input - Counter value for loadable counter application
           .DATAIN(1'b0), // 1-bit input - Internal delay data
           .IDATAIN(1'b0), // 1-bit input - Delay data input
           .INC(1'b0), // 1-bit input - Increment / Decrement tap delay
           .ODATAIN(dci_tmp), // 1-bit input - Data input for the output datapath from the device
           .RST(delay_reset), // 1-bit input - Active high, synchronous reset, resets delay chain to IDELAY_VALUE / ODELAY_VALUE tap. If no value is specified, the default is 0.
           .T(1'b0) // 1-bit input - 3-state input control. Tie high for input-only or internal delay or tie low for output only.
           );

(* IODELAY_GROUP = "dci_group" *) // Specifies group name for associated IODELAYs and IDELAYCTRL
IDELAYCTRL IDELAYCTRL_inst (
          .RDY(), // 1-bit Indicates the validity of the reference clock input, REFCLK. When REFCLK
          // disappears (i.e., REFCLK is held High or Low for one clock period or more), the RDY
          // signal is deasserted.
          .REFCLK(clk), // 1-bit Provides a voltage bias, independent of process, voltage, and temperature
          // variations, to the tap-delay lines in the IOBs. The frequency of REFCLK must be 200
          // MHz to guarantee the tap-delay value specified in the applicable data sheet.
          .RST(1'b0) // 1-bit Resets the IDELAYCTRL circuitry. The RST signal is an active-high asynchronous
          // reset. To reset the IDELAYCTRL, assert it High for at least 50 ns.
          ); // End of IDELAYCTRL_inst instantiation

       OBUFDS #(.IOSTANDARD("LVDS_25")) obufds_dci 
        (.I(dci_delay), .O(dac_dci_out_p), .OB(dac_dci_out_n) );


//FRAME
       ODDR #(
       .DDR_CLK_EDGE("OPPOSITE_EDGE"), // "OPPOSITE_EDGE" or "SAME_EDGE"
       .INIT(1'b0), // Initial value of Q: 1’b0 or 1’b1
       .SRTYPE("SYNC") // Set/Reset type: "SYNC" or "ASYNC"
       ) ODDR_inst_frame (
       .Q(frame_tmp), // 1-bit DDR output
       .C(clk), // 1-bit clock input
       .CE(1'b1), // 1-bit clock enable input
       .D1(1'b0), // 1-bit data input (positive edge)
       .D2(1'b0), // 1-bit data input (negative edge)
       .R(1'b0), // 1-bit reset
       .S(1'b0) // 1-bit set
       );

       OBUFDS #(.IOSTANDARD("LVDS_25")) obufds_frame 
        (.I(frame_tmp), .O(dac_frame_out_p), .OB(dac_frame_out_n) );

endmodule // transmit_ad9122
