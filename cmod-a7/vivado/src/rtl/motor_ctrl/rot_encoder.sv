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

module rot_encoder
  #(parameter
    BAND_WIDTH = 32)
   (input logic                   sa,
    input logic                   sb,
    output logic [BAND_WIDTH-1:0] count,
    input logic                   clk,
    input logic                   rstn);

   logic [BAND_WIDTH-1:0]         count_r;
   logic [3:0]                    patern;
   logic [1:0]                    sa_q, sb_q;

   always_comb begin
      count  = count_r;
      patern = {sa_q[1], sb_q[1], sa_q[0], sb_q[0]};
   end

   always_ff @(posedge clk) begin
      if(!rstn) begin
         count_r <= 0;
         sa_q    <= 0;
         sb_q    <= 0;
      end
      else begin
         sa_q[1] <= sa_q[0];
         sb_q[1] <= sb_q[0];
         sa_q[0] <= sa;
         sb_q[0] <= sb;
         case(patern)
           4'b0010: count_r <= count_r + 1;
           4'b1011: count_r <= count_r + 1;
           4'b1101: count_r <= count_r + 1;
           4'b0100: count_r <= count_r + 1;
           4'b0001: count_r <= count_r - 1;
           4'b0111: count_r <= count_r - 1;
           4'b1110: count_r <= count_r - 1;
           4'b1000: count_r <= count_r - 1;
           default: count_r <= count_r;
         endcase
      end
   end

endmodule
