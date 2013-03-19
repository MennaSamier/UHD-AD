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



module capture_ad9434
  #(parameter WIDTH=7)
   (input clk,
    input ssclk_p,
    input ssclk_n,
    input [WIDTH-1:0] in_p,
    input [WIDTH-1:0] in_n,
    output reg [WIDTH-1:0] out);

   wire [WIDTH-1:0] 	   ddr_dat;
   wire 		   ssclk;
   wire [WIDTH-1:0]    out_pre1;
   reg [WIDTH-1:0] 	   out_pre2;
   
   IBUFGDS #(.IOSTANDARD("LVDS_25"), .DIFF_TERM("TRUE")) 
   clkbuf (.O(ssclk), .I(ssclk_p), .IB(ssclk_n));
   
   genvar 	       i;
   generate
      for(i = 0; i < WIDTH; i = i + 1)
	begin : gen_lvds_pins
	   IBUFDS #(.IOSTANDARD("LVDS_25"),.DIFF_TERM("FALSE")) ibufds 
	      (.O(ddr_dat[i]), .I(in_p[i]), .IB(in_n[i]) );
	end
   endgenerate

   always @(posedge ssclk)
     out_pre2 <= ddr_dat;

   always @(posedge clk)
     out      <= out_pre2;
   
endmodule // capture_ad9434
