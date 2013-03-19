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


// Boot RAM for S3A, 8KB, dual port

// RAMB16BWE_S36_S36: 512 x 32 + 4 Parity bits byte-wide write Dual-Port RAM
//      Spartan-3A Xilinx HDL Libraries Guide, version 10.1.1

module bootram
  (input clk, input reset,
   input [13:0] if_adr,
   output [31:0] if_data,

   input [13:0] dwb_adr_i,
   input [31:0] dwb_dat_i,
   output [31:0] dwb_dat_o,
   input dwb_we_i,
   output reg dwb_ack_o,
   input dwb_stb_i,
   input [3:0] dwb_sel_i);
`ifdef ML605
   wire [31:0] DOA0, DOA1, DOA2, DOA3;
   wire [31:0] DOB0, DOB1, DOB2, DOB3;
   wire        ENB0, ENB1, ENB2, ENB3;
`else
   wire [31:0] DOA0, DOA1, DOA2, DOA3, DOA4, DOA5, DOA6, DOA7;
   wire [31:0] DOB0, DOB1, DOB2, DOB3, DOB4, DOB5, DOB6, DOB7;
   wire        ENB0, ENB1, ENB2, ENB3, ENB4, ENB5, ENB6, ENB7;
`endif
   wire [3:0]  WEB;

`ifdef ML605
   reg [1:0]   delayed_if_bank;
   always @(posedge clk)
     delayed_if_bank <= if_adr[13:12];
   assign if_data =  (delayed_if_bank[1] ? (delayed_if_bank[0] ? DOA3 : DOA2) : (delayed_if_bank[0] ? DOA1 : DOA0));
   assign dwb_dat_o = (dwb_adr_i[13] ? (dwb_adr_i[12] ? DOB3 : DOB2) : (dwb_adr_i[12] ? DOB1 : DOB0));
`else
   reg [2:0]   delayed_if_bank;
   always @(posedge clk)
     delayed_if_bank <= if_adr[13:11];
   
   assign if_data = delayed_if_bank[2] ?
                   (delayed_if_bank[1] ? (delayed_if_bank[0] ? DOA7 : DOA6) : (delayed_if_bank[0] ? DOA5 : DOA4))
                 : (delayed_if_bank[1] ? (delayed_if_bank[0] ? DOA3 : DOA2) : (delayed_if_bank[0] ? DOA1 : DOA0));
   
   
   assign dwb_dat_o = dwb_adr_i[13] ?
                     (dwb_adr_i[12] ? (dwb_adr_i[11] ? DOB7 : DOB6) : (dwb_adr_i[11] ? DOB5 : DOB4))
                   : (dwb_adr_i[12] ? (dwb_adr_i[11] ? DOB3 : DOB2) : (dwb_adr_i[11] ? DOB1 : DOB0));
`endif

   always @(posedge clk)
     if(reset)
       dwb_ack_o <= 0;
     else
       dwb_ack_o <= dwb_stb_i & ~dwb_ack_o;
   
`ifdef ML605
   assign ENB0 = dwb_stb_i & (dwb_adr_i[13:12] == 2'b00);
   assign ENB1 = dwb_stb_i & (dwb_adr_i[13:12] == 2'b01);
   assign ENB2 = dwb_stb_i & (dwb_adr_i[13:12] == 2'b10);
   assign ENB3 = dwb_stb_i & (dwb_adr_i[13:12] == 2'b11);

   assign WEB = {4{dwb_we_i}} & dwb_sel_i;

//RAM0
        BRAM_TDP_MACRO 
          #(.BRAM_SIZE("36Kb"), // Target BRAM: "18Kb" or "36Kb"
            .DEVICE("VIRTEX6"), // Target device: "VIRTEX5", "VIRTEX6", "SPARTAN6"
            .DOA_REG(0), // Optional port A output register (0 or 1)
            .DOB_REG(0), // Optional port B output register (0 or 1)
            //.INIT_A(36’h000000000), // Initial values on port A output port
            //.INIT_B(36’h000000000), // Initial values on port B output port
            .INIT_FILE ("NONE"),
            .READ_WIDTH_A (36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .READ_WIDTH_B (36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .SIM_COLLISION_CHECK ("ALL"), // Collision check enable "ALL", "WARNING_ONLY", "GENERATE_X_ONLY" or "NONE"
            //.SRVAL_A(36’h000000000), // Set/Reset value for port A output
            //.SRVAL_B(36’h000000000), // Set/Reset value for port B output
            .WRITE_MODE_A("WRITE_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
            .WRITE_MODE_B("WRITE_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
            .WRITE_WIDTH_A(36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .WRITE_WIDTH_B(36)) 
        RAM0 (.DOA(DOA0), // Output port-A data, width defined by READ_WIDTH_A parameter
            .DOB(DOB0), // Output port-B data, width defined by READ_WIDTH_B parameter
            .ADDRA(if_adr[11:2]), // Input port-A address, width defined by Port A depth
            .ADDRB(dwb_adr_i[11:2]), // Input port-B address, width defined by Port B depth
            .CLKA(clk), // 1-bit input port-A clock
            .CLKB(clk), // 1-bit input port-B clock
            .DIA(36'hfffffffff), // Input port-A data, width defined by WRITE_WIDTH_A parameter
            .DIB({4'hf,dwb_dat_i}), // Input port-B data, width defined by WRITE_WIDTH_B parameter
            .ENA(1'b1), // 1-bit input port-A enable
            .ENB(ENB0), // 1-bit input port-B enable
            .REGCEA(1'b0), // 1-bit input port-A output register enable
            .REGCEB(1'b0),//1-bit input port-B output register enable
            .RSTA(1'b0),    //1-bit input port-A reset
            .RSTB(1'b0),    //1-bit input port-B reset
            .WEA(1'b0),      //Input port-A write enable, width defined by Port A depth
            .WEB(WEB)); // End of BRAM_TDP_MACRO_inst instantiation

//RAM1
        BRAM_TDP_MACRO 
          #(.BRAM_SIZE("36Kb"), // Target BRAM: "18Kb" or "36Kb"
            .DEVICE("VIRTEX6"), // Target device: "VIRTEX5", "VIRTEX6", "SPARTAN6"
            .DOA_REG(0), // Optional port A output register (0 or 1)
            .DOB_REG(0), // Optional port B output register (0 or 1)
            //.INIT_A(36’h000000000), // Initial values on port A output port
            //.INIT_B(36’h000000000), // Initial values on port B output port
            .INIT_FILE ("NONE"),
            .READ_WIDTH_A (36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .READ_WIDTH_B (36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .SIM_COLLISION_CHECK ("ALL"), // Collision check enable "ALL", "WARNING_ONLY", "GENERATE_X_ONLY" or "NONE"
            //.SRVAL_A(36’h000000000), // Set/Reset value for port A output
            //.SRVAL_B(36’h000000000), // Set/Reset value for port B output
            .WRITE_MODE_A("WRITE_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
            .WRITE_MODE_B("WRITE_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
            .WRITE_WIDTH_A(36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .WRITE_WIDTH_B(36)) 
        RAM1 (.DOA(DOA1), // Output port-A data, width defined by READ_WIDTH_A parameter
            .DOB(DOB1), // Output port-B data, width defined by READ_WIDTH_B parameter
            .ADDRA(if_adr[11:2]), // Input port-A address, width defined by Port A depth
            .ADDRB(dwb_adr_i[11:2]), // Input port-B address, width defined by Port B depth
            .CLKA(clk), // 1-bit input port-A clock
            .CLKB(clk), // 1-bit input port-B clock
            .DIA(36'hfffffffff), // Input port-A data, width defined by WRITE_WIDTH_A parameter
            .DIB({4'hf,dwb_dat_i}), // Input port-B data, width defined by WRITE_WIDTH_B parameter
            .ENA(1'b1), // 1-bit input port-A enable
            .ENB(ENB1), // 1-bit input port-B enable
            .REGCEA(1'b0), // 1-bit input port-A output register enable
            .REGCEB(1'b0),//1-bit input port-B output register enable
            .RSTA(1'b0),    //1-bit input port-A reset
            .RSTB(1'b0),    //1-bit input port-B reset
            .WEA(1'b0),      //Input port-A write enable, width defined by Port A depth
            .WEB(WEB)); // End of BRAM_TDP_MACRO_inst instantiation

//RAM2
        BRAM_TDP_MACRO
          #(.BRAM_SIZE("36Kb"), // Target BRAM: "18Kb" or "36Kb"
            .DEVICE("VIRTEX6"), // Target device: "VIRTEX5", "VIRTEX6", "SPARTAN6"
            .DOA_REG(0), // Optional port A output register (0 or 1)
            .DOB_REG(0), // Optional port B output register (0 or 1)
            //.INIT_A(36’h000000000), // Initial values on port A output port
            //.INIT_B(36’h000000000), // Initial values on port B output port
            .INIT_FILE ("NONE"),
            .READ_WIDTH_A (36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .READ_WIDTH_B (36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .SIM_COLLISION_CHECK ("ALL"), // Collision check enable "ALL", "WARNING_ONLY", "GENERATE_X_ONLY" or "NONE"
            //.SRVAL_A(36’h000000000), // Set/Reset value for port A output
            //.SRVAL_B(36’h000000000), // Set/Reset value for port B output
            .WRITE_MODE_A("WRITE_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
            .WRITE_MODE_B("WRITE_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
            .WRITE_WIDTH_A(36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .WRITE_WIDTH_B(36)) 
        RAM2 (.DOA(DOA2), // Output port-A data, width defined by READ_WIDTH_A parameter
            .DOB(DOB2), // Output port-B data, width defined by READ_WIDTH_B parameter
            .ADDRA(if_adr[11:2]), // Input port-A address, width defined by Port A depth
            .ADDRB(dwb_adr_i[11:2]), // Input port-B address, width defined by Port B depth
            .CLKA(clk), // 1-bit input port-A clock
            .CLKB(clk), // 1-bit input port-B clock
            .DIA(36'hfffffffff), // Input port-A data, width defined by WRITE_WIDTH_A parameter
            .DIB({4'hf,dwb_dat_i}), // Input port-B data, width defined by WRITE_WIDTH_B parameter
            .ENA(1'b1), // 1-bit input port-A enable
            .ENB(ENB2), // 1-bit input port-B enable
            .REGCEA(1'b0), // 1-bit input port-A output register enable
            .REGCEB(1'b0),//1-bit input port-B output register enable
            .RSTA(1'b0),    //1-bit input port-A reset
            .RSTB(1'b0),    //1-bit input port-B reset
            .WEA(1'b0),      //Input port-A write enable, width defined by Port A depth
            .WEB(WEB)); // End of BRAM_TDP_MACRO_inst instantiation

//RAM3
        BRAM_TDP_MACRO
          #(.BRAM_SIZE("36Kb"), // Target BRAM: "18Kb" or "36Kb"
            .DEVICE("VIRTEX6"), // Target device: "VIRTEX5", "VIRTEX6", "SPARTAN6"
            .DOA_REG(0), // Optional port A output register (0 or 1)
            .DOB_REG(0), // Optional port B output register (0 or 1)
            //.INIT_A(36’h000000000), // Initial values on port A output port
            //.INIT_B(36’h000000000), // Initial values on port B output port
            .INIT_FILE ("NONE"),
            .READ_WIDTH_A (36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .READ_WIDTH_B (36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .SIM_COLLISION_CHECK ("ALL"), // Collision check enable "ALL", "WARNING_ONLY", "GENERATE_X_ONLY" or "NONE"
            //.SRVAL_A(36’h000000000), // Set/Reset value for port A output
            //.SRVAL_B(36’h000000000), // Set/Reset value for port B output
            .WRITE_MODE_A("WRITE_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
            .WRITE_MODE_B("WRITE_FIRST"), // "WRITE_FIRST", "READ_FIRST", or "NO_CHANGE"
            .WRITE_WIDTH_A(36), // Valid values are 1-36 (19-36 only valid when BRAM_SIZE="36Kb")
            .WRITE_WIDTH_B(36)) 
         RAM3 (.DOA(DOA3), // Output port-A data, width defined by READ_WIDTH_A parameter
            .DOB(DOB3), // Output port-B data, width defined by READ_WIDTH_B parameter
            .ADDRA(if_adr[11:2]), // Input port-A address, width defined by Port A depth
            .ADDRB(dwb_adr_i[11:2]), // Input port-B address, width defined by Port B depth
            .CLKA(clk), // 1-bit input port-A clock
            .CLKB(clk), // 1-bit input port-B clock
            .DIA(36'hfffffffff), // Input port-A data, width defined by WRITE_WIDTH_A parameter
            .DIB({4'hf,dwb_dat_i}), // Input port-B data, width defined by WRITE_WIDTH_B parameter
            .ENA(1'b1), // 1-bit input port-A enable
            .ENB(ENB3), // 1-bit input port-B enable
            .REGCEA(1'b0), // 1-bit input port-A output register enable
            .REGCEB(1'b0),//1-bit input port-B output register enable
            .RSTA(1'b0),    //1-bit input port-A reset
            .RSTB(1'b0),    //1-bit input port-B reset
            .WEA(1'b0),      //Input port-A write enable, width defined by Port A depth
            .WEB(WEB)); // End of BRAM_TDP_MACRO_inst instantiation
`else
   assign ENB0 = dwb_stb_i & (dwb_adr_i[13:11] == 3'b000);
   assign ENB1 = dwb_stb_i & (dwb_adr_i[13:11] == 3'b001);
   assign ENB2 = dwb_stb_i & (dwb_adr_i[13:11] == 3'b010);
   assign ENB3 = dwb_stb_i & (dwb_adr_i[13:11] == 3'b011);
   assign ENB4 = dwb_stb_i & (dwb_adr_i[13:11] == 3'b100);
   assign ENB5 = dwb_stb_i & (dwb_adr_i[13:11] == 3'b101);
   assign ENB6 = dwb_stb_i & (dwb_adr_i[13:11] == 3'b110);
   assign ENB7 = dwb_stb_i & (dwb_adr_i[13:11] == 3'b111);

   assign WEB = {4{dwb_we_i}} & dwb_sel_i;
   
   RAMB16BWE_S36_S36 
     #(.INIT_A(36'h000000000), // Value of output RAM registers on Port A at startup
       .INIT_B(36'h000000000), // Value of output RAM registers on Port B at startup
       .SIM_COLLISION_CHECK("ALL"), // "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
       .SRVAL_A(36'h000000000), // Port       A output value upon SSR    assertion
       .SRVAL_B(36'h000000000), // Port       B output value upon SSR    assertion
       .WRITE_MODE_A("WRITE_FIRST"), //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
       .WRITE_MODE_B("WRITE_FIRST")) //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
   RAM0
     (.DOA(DOA0),           // Port A 32-bit Data Output
      .DOPA(),              // Port A 4-bit Parity Output
      .ADDRA(if_adr[10:2]), // Port A 9-bit Address Input
      .CLKA(clk),           // Port A 1-bit Clock
      .DIA(32'hffffffff),   // Port A 32-bit Data Input
      .DIPA(4'hf),          // Port A 4-bit parity Input
      .ENA(1'b1),           // Port A 1-bit RAM Enable Input
      .SSRA(1'b0),          // Port A 1-bit Synchronous Set/Reset Input
      .WEA(1'b0),           // Port A 4-bit Write Enable Input

      .DOB(DOB0),              // Port B 32-bit Data Output
      .DOPB(),                 // Port B 4-bit Parity Output
      .ADDRB(dwb_adr_i[10:2]), // Port B 9-bit Address Input
      .CLKB(clk),              // Port B 1-bit Clock
      .DIB(dwb_dat_i),         // Port B 32-bit Data Input
      .DIPB(4'hf),             // Port-B 4-bit parity Input
      .ENB(ENB0),              // Port B 1-bit RAM Enable Input
      .SSRB(1'b0),             // Port B 1-bit Synchronous Set/Reset Input
      .WEB(WEB)                // Port B 4-bit Write Enable Input
      );   // End of RAMB16BWE_S36_S36_inst instantiation

   RAMB16BWE_S36_S36 
     #(.INIT_A(36'h000000000), // Value of output RAM registers on Port A at startup
       .INIT_B(36'h000000000), // Value of output RAM registers on Port B at startup
       .SIM_COLLISION_CHECK("ALL"), // "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
       .SRVAL_A(36'h000000000), // Port       A output value upon SSR    assertion
       .SRVAL_B(36'h000000000), // Port       B output value upon SSR    assertion
       .WRITE_MODE_A("WRITE_FIRST"), //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
       .WRITE_MODE_B("WRITE_FIRST")) //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
   RAM1
     (.DOA(DOA1),           // Port A 32-bit Data Output
      .DOPA(),              // Port A 4-bit Parity Output
      .ADDRA(if_adr[10:2]), // Port A 9-bit Address Input
      .CLKA(clk),           // Port A 1-bit Clock
      .DIA(32'hffffffff),          // Port A 32-bit Data Input
      .DIPA(4'hf),          // Port A 4-bit parity Input
      .ENA(1'b1),           // Port A 1-bit RAM Enable Input
      .SSRA(1'b0),          // Port A 1-bit Synchronous Set/Reset Input
      .WEA(1'b0),           // Port A 4-bit Write Enable Input

      .DOB(DOB1),              // Port B 32-bit Data Output
      .DOPB(),                 // Port B 4-bit Parity Output
      .ADDRB(dwb_adr_i[10:2]), // Port B 9-bit Address Input
      .CLKB(clk),              // Port B 1-bit Clock
      .DIB(dwb_dat_i),         // Port B 32-bit Data Input
      .DIPB(4'hf),             // Port-B 4-bit parity Input
      .ENB(ENB1),              // Port B 1-bit RAM Enable Input
      .SSRB(1'b0),             // Port B 1-bit Synchronous Set/Reset Input
      .WEB(WEB)                // Port B 4-bit Write Enable Input
      );   // End of RAMB16BWE_S36_S36_inst instantiation

   RAMB16BWE_S36_S36 
     #(.INIT_A(36'h000000000), // Value of output RAM registers on Port A at startup
       .INIT_B(36'h000000000), // Value of output RAM registers on Port B at startup
       .SIM_COLLISION_CHECK("ALL"), // "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
       .SRVAL_A(36'h000000000), // Port       A output value upon SSR    assertion
       .SRVAL_B(36'h000000000), // Port       B output value upon SSR    assertion
       .WRITE_MODE_A("WRITE_FIRST"), //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
       .WRITE_MODE_B("WRITE_FIRST")) //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
   RAM2
     (.DOA(DOA2),           // Port A 32-bit Data Output
      .DOPA(),              // Port A 4-bit Parity Output
      .ADDRA(if_adr[10:2]), // Port A 9-bit Address Input
      .CLKA(clk),           // Port A 1-bit Clock
      .DIA(32'hffffffff),          // Port A 32-bit Data Input
      .DIPA(4'hf),          // Port A 4-bit parity Input
      .ENA(1'b1),           // Port A 1-bit RAM Enable Input
      .SSRA(1'b0),          // Port A 1-bit Synchronous Set/Reset Input
      .WEA(1'b0),           // Port A 4-bit Write Enable Input

      .DOB(DOB2),              // Port B 32-bit Data Output
      .DOPB(),                 // Port B 4-bit Parity Output
      .ADDRB(dwb_adr_i[10:2]), // Port B 9-bit Address Input
      .CLKB(clk),              // Port B 1-bit Clock
      .DIB(dwb_dat_i),         // Port B 32-bit Data Input
      .DIPB(4'hf),             // Port-B 4-bit parity Input
      .ENB(ENB2),              // Port B 1-bit RAM Enable Input
      .SSRB(1'b0),             // Port B 1-bit Synchronous Set/Reset Input
      .WEB(WEB)                // Port B 4-bit Write Enable Input
      );   // End of RAMB16BWE_S36_S36_inst instantiation

   RAMB16BWE_S36_S36 
     #(.INIT_A(36'h000000000), // Value of output RAM registers on Port A at startup
       .INIT_B(36'h000000000), // Value of output RAM registers on Port B at startup
       .SIM_COLLISION_CHECK("ALL"), // "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
       .SRVAL_A(36'h000000000), // Port       A output value upon SSR    assertion
       .SRVAL_B(36'h000000000), // Port       B output value upon SSR    assertion
       .WRITE_MODE_A("WRITE_FIRST"), //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
       .WRITE_MODE_B("WRITE_FIRST")) //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
   RAM3
     (.DOA(DOA3),           // Port A 32-bit Data Output
      .DOPA(),              // Port A 4-bit Parity Output
      .ADDRA(if_adr[10:2]), // Port A 9-bit Address Input
      .CLKA(clk),           // Port A 1-bit Clock
      .DIA(32'hffffffff),          // Port A 32-bit Data Input
      .DIPA(4'hf),          // Port A 4-bit parity Input
      .ENA(1'b1),           // Port A 1-bit RAM Enable Input
      .SSRA(1'b0),          // Port A 1-bit Synchronous Set/Reset Input
      .WEA(1'b0),           // Port A 4-bit Write Enable Input

      .DOB(DOB3),              // Port B 32-bit Data Output
      .DOPB(),                 // Port B 4-bit Parity Output
      .ADDRB(dwb_adr_i[10:2]), // Port B 9-bit Address Input
      .CLKB(clk),              // Port B 1-bit Clock
      .DIB(dwb_dat_i),         // Port B 32-bit Data Input
      .DIPB(4'hf),             // Port-B 4-bit parity Input
      .ENB(ENB3),              // Port B 1-bit RAM Enable Input
      .SSRB(1'b0),             // Port B 1-bit Synchronous Set/Reset Input
      .WEB(WEB)                // Port B 4-bit Write Enable Input
      );   // End of RAMB16BWE_S36_S36_inst instantiation
      
   RAMB16BWE_S36_S36 
     #(.INIT_A(36'h000000000), // Value of output RAM registers on Port A at startup
       .INIT_B(36'h000000000), // Value of output RAM registers on Port B at startup
       .SIM_COLLISION_CHECK("ALL"), // "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
       .SRVAL_A(36'h000000000), // Port       A output value upon SSR    assertion
       .SRVAL_B(36'h000000000), // Port       B output value upon SSR    assertion
       .WRITE_MODE_A("WRITE_FIRST"), //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
       .WRITE_MODE_B("WRITE_FIRST")) //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
   RAM4
     (.DOA(DOA4),           // Port A 32-bit Data Output
      .DOPA(),              // Port A 4-bit Parity Output
      .ADDRA(if_adr[10:2]), // Port A 9-bit Address Input
      .CLKA(clk),           // Port A 1-bit Clock
      .DIA(32'hffffffff),          // Port A 32-bit Data Input
      .DIPA(4'hf),          // Port A 4-bit parity Input
      .ENA(1'b1),           // Port A 1-bit RAM Enable Input
      .SSRA(1'b0),          // Port A 1-bit Synchronous Set/Reset Input
      .WEA(1'b0),           // Port A 4-bit Write Enable Input

      .DOB(DOB4),              // Port B 32-bit Data Output
      .DOPB(),                 // Port B 4-bit Parity Output
      .ADDRB(dwb_adr_i[10:2]), // Port B 9-bit Address Input
      .CLKB(clk),              // Port B 1-bit Clock
      .DIB(dwb_dat_i),         // Port B 32-bit Data Input
      .DIPB(4'hf),             // Port-B 4-bit parity Input
      .ENB(ENB4),              // Port B 1-bit RAM Enable Input
      .SSRB(1'b0),             // Port B 1-bit Synchronous Set/Reset Input
      .WEB(WEB)                // Port B 4-bit Write Enable Input
      );   // End of RAMB16BWE_S36_S36_inst instantiation
      
   RAMB16BWE_S36_S36 
     #(.INIT_A(36'h000000000), // Value of output RAM registers on Port A at startup
       .INIT_B(36'h000000000), // Value of output RAM registers on Port B at startup
       .SIM_COLLISION_CHECK("ALL"), // "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
       .SRVAL_A(36'h000000000), // Port       A output value upon SSR    assertion
       .SRVAL_B(36'h000000000), // Port       B output value upon SSR    assertion
       .WRITE_MODE_A("WRITE_FIRST"), //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
       .WRITE_MODE_B("WRITE_FIRST")) //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
   RAM5
     (.DOA(DOA5),           // Port A 32-bit Data Output
      .DOPA(),              // Port A 4-bit Parity Output
      .ADDRA(if_adr[10:2]), // Port A 9-bit Address Input
      .CLKA(clk),           // Port A 1-bit Clock
      .DIA(32'hffffffff),          // Port A 32-bit Data Input
      .DIPA(4'hf),          // Port A 4-bit parity Input
      .ENA(1'b1),           // Port A 1-bit RAM Enable Input
      .SSRA(1'b0),          // Port A 1-bit Synchronous Set/Reset Input
      .WEA(1'b0),           // Port A 4-bit Write Enable Input

      .DOB(DOB5),              // Port B 32-bit Data Output
      .DOPB(),                 // Port B 4-bit Parity Output
      .ADDRB(dwb_adr_i[10:2]), // Port B 9-bit Address Input
      .CLKB(clk),              // Port B 1-bit Clock
      .DIB(dwb_dat_i),         // Port B 32-bit Data Input
      .DIPB(4'hf),             // Port-B 4-bit parity Input
      .ENB(ENB5),              // Port B 1-bit RAM Enable Input
      .SSRB(1'b0),             // Port B 1-bit Synchronous Set/Reset Input
      .WEB(WEB)                // Port B 4-bit Write Enable Input
      );   // End of RAMB16BWE_S36_S36_inst instantiation
         
   RAMB16BWE_S36_S36 
     #(.INIT_A(36'h000000000), // Value of output RAM registers on Port A at startup
       .INIT_B(36'h000000000), // Value of output RAM registers on Port B at startup
       .SIM_COLLISION_CHECK("ALL"), // "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
       .SRVAL_A(36'h000000000), // Port       A output value upon SSR    assertion
       .SRVAL_B(36'h000000000), // Port       B output value upon SSR    assertion
       .WRITE_MODE_A("WRITE_FIRST"), //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
       .WRITE_MODE_B("WRITE_FIRST")) //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
   RAM6
     (.DOA(DOA6),           // Port A 32-bit Data Output
      .DOPA(),              // Port A 4-bit Parity Output
      .ADDRA(if_adr[10:2]), // Port A 9-bit Address Input
      .CLKA(clk),           // Port A 1-bit Clock
      .DIA(32'hffffffff),          // Port A 32-bit Data Input
      .DIPA(4'hf),          // Port A 4-bit parity Input
      .ENA(1'b1),           // Port A 1-bit RAM Enable Input
      .SSRA(1'b0),          // Port A 1-bit Synchronous Set/Reset Input
      .WEA(1'b0),           // Port A 4-bit Write Enable Input

      .DOB(DOB6),              // Port B 32-bit Data Output
      .DOPB(),                 // Port B 4-bit Parity Output
      .ADDRB(dwb_adr_i[10:2]), // Port B 9-bit Address Input
      .CLKB(clk),              // Port B 1-bit Clock
      .DIB(dwb_dat_i),         // Port B 32-bit Data Input
      .DIPB(4'hf),             // Port-B 4-bit parity Input
      .ENB(ENB6),              // Port B 1-bit RAM Enable Input
      .SSRB(1'b0),             // Port B 1-bit Synchronous Set/Reset Input
      .WEB(WEB)                // Port B 4-bit Write Enable Input
      );   // End of RAMB16BWE_S36_S36_inst instantiation
      
   RAMB16BWE_S36_S36 
     #(.INIT_A(36'h000000000), // Value of output RAM registers on Port A at startup
       .INIT_B(36'h000000000), // Value of output RAM registers on Port B at startup
       .SIM_COLLISION_CHECK("ALL"), // "NONE", "WARNING_ONLY", "GENERATE_X_ONLY", "ALL"
       .SRVAL_A(36'h000000000), // Port       A output value upon SSR    assertion
       .SRVAL_B(36'h000000000), // Port       B output value upon SSR    assertion
       .WRITE_MODE_A("WRITE_FIRST"), //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
       .WRITE_MODE_B("WRITE_FIRST")) //       WRITE_FIRST, READ_FIRST    or NO_CHANGE
   RAM7
     (.DOA(DOA7),           // Port A 32-bit Data Output
      .DOPA(),              // Port A 4-bit Parity Output
      .ADDRA(if_adr[10:2]), // Port A 9-bit Address Input
      .CLKA(clk),           // Port A 1-bit Clock
      .DIA(32'hffffffff),          // Port A 32-bit Data Input
      .DIPA(4'hf),          // Port A 4-bit parity Input
      .ENA(1'b1),           // Port A 1-bit RAM Enable Input
      .SSRA(1'b0),          // Port A 1-bit Synchronous Set/Reset Input
      .WEA(1'b0),           // Port A 4-bit Write Enable Input

      .DOB(DOB7),              // Port B 32-bit Data Output
      .DOPB(),                 // Port B 4-bit Parity Output
      .ADDRB(dwb_adr_i[10:2]), // Port B 9-bit Address Input
      .CLKB(clk),              // Port B 1-bit Clock
      .DIB(dwb_dat_i),         // Port B 32-bit Data Input
      .DIPB(4'hf),             // Port-B 4-bit parity Input
      .ENB(ENB7),              // Port B 1-bit RAM Enable Input
      .SSRB(1'b0),             // Port B 1-bit Synchronous Set/Reset Input
      .WEB(WEB)                // Port B 4-bit Write Enable Input
      );   // End of RAMB16BWE_S36_S36_inst instantiation

`endif
endmodule // bootram
