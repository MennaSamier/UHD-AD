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
    output dac_dci_out_p,
    output dac_dci_out_n,
    output dac_frame_out_p,
    output dac_frame_out_n,
    output [WIDTH-1:0] out_p,
    output [WIDTH-1:0] out_n,
    input [WIDTH-1:0] in_a,
    input [WIDTH-1:0] in_b);

   wire [WIDTH-1:0]  ddr_dat;
   wire dci_tmp;
   wire frame_tmp;
   
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

       OBUFDS #(.IOSTANDARD("LVDS_25")) obufds 
        (.I(ddr_dat[i]), .O(out_p[i]), .OB(out_n[i]) );
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

       OBUFDS #(.IOSTANDARD("LVDS_25")) obufds_dci 
        (.I(dci_tmp), .O(dac_dci_out_p), .OB(dac_dci_out_n) );


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
