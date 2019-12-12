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
// module      : reg_fifo
// description :
module reg_fifo
  #(parameter
    /*
     You can specify the following parameters.
     1. DATA_WIDTH : input and output data width
     2. FIFO_DEPTH : data capacity
     */
    DATA_WIDTH    = 8,
    FIFO_DEPTH    = 4,
    localparam
    LB_FIFO_DEPTH = $clog2(FIFO_DEPTH))
   (input  logic [DATA_WIDTH-1:0]  in_data,
    input  logic                   in_valid,
    output logic                   in_ready,
    output logic [DATA_WIDTH-1:0]  out_data,
    output logic                   out_valid,
    input  logic                   out_ready,
    input  logic                   clear,
    output logic [LB_FIFO_DEPTH:0] count,
    input  logic                   clk,
    input  logic                   rstn);

   logic [FIFO_DEPTH-1:0][DATA_WIDTH-1:0] dist_ram = '{default:0};
   logic [LB_FIFO_DEPTH-1:0]              waddr_r, raddr_r;
   logic [LB_FIFO_DEPTH:0]                count_r;
   logic                                  in_exec, out_exec;

   always_comb begin
      in_ready  = (count_r < FIFO_DEPTH) ? 1 : 0;
      out_valid = (0 < count_r) ? 1 : 0;
      out_data  = dist_ram[raddr_r];
      count     = count_r;
      in_exec   = in_valid  & in_ready;
      out_exec  = out_valid & out_ready;
   end

   always_ff @(posedge clk) begin
     if(!rstn | clear) begin
        waddr_r <= 0;
        raddr_r <= 0;
        count_r <= 0;
     end
     else begin
        case({in_exec, out_exec})
          2'b00 : begin
          end
          2'b01 : begin
             raddr_r <= raddr_r + 1;
             count_r <= count_r - 1;
          end
          2'b10 : begin
             dist_ram[waddr_r] <= in_data;
             waddr_r           <= waddr_r + 1;
             count_r           <= count_r + 1;
          end
          2'b11 : begin
             dist_ram[waddr_r] <= in_data;
             waddr_r           <= waddr_r + 1;
             raddr_r           <= raddr_r + 1;
          end
        endcase
     end
   end

endmodule
