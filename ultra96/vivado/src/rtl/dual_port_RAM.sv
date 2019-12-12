/*
 MIT License

 Copyright (c) 2019 Yuya Kudo

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

//-----------------------------------------------------------------------------
// module      : dual_port_RAM
// description :
module dual_port_RAM
  #(parameter
    /*
     You can specify the following two parameters.
     1. DATA_WIDTH : data width
     2. RAM_DEPTH  : data capacity
     */
    DATA_WIDTH   = 8,
    RAM_DEPTH    = 256,
    localparam
    LB_RAM_DEPTH = $clog2(RAM_DEPTH))
   (input  logic [DATA_WIDTH-1:0]   din0,
    input  logic [DATA_WIDTH-1:0]   din1,
    input  logic [LB_RAM_DEPTH-1:0] addr0,
    input  logic [LB_RAM_DEPTH-1:0] addr1,
    output logic [DATA_WIDTH-1:0]   dout0,
    output logic [DATA_WIDTH-1:0]   dout1,
    input  logic                    wr_en0,
    input  logic                    wr_en1,
    input  logic                    clk);

   logic [DATA_WIDTH-1:0]           ram[RAM_DEPTH-1:0] = '{default:0};
   logic [DATA_WIDTH-1:0]           din_r0, din_r1;
   logic [LB_RAM_DEPTH-1:0]         addr_r0, addr_r1;
   logic                            wr_en_r0, wr_en_r1;

   always_ff @(posedge clk) begin
      din_r0   <= din0;
      addr_r0  <= addr0;
      wr_en_r0 <= wr_en0;

      if(wr_en_r0) begin
         ram[addr_r0] <= din_r0;
      end
   end

   always_ff @(posedge clk) begin
      din_r1   <= din1;
      addr_r1  <= addr1;
      wr_en_r1 <= wr_en1;

      if(wr_en_r1) begin
         ram[addr_r1] <= din_r1;
      end
   end

   assign dout0 = ram[addr_r0];
   assign dout1 = ram[addr_r1];

endmodule
