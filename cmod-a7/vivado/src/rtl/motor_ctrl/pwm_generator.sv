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

module pwm_generator
  #(parameter
    BAND_WIDTH       = 32,
    PWM_PERIOD_CYCLE = 1000)
   (input logic [BAND_WIDTH-1:0] width,
    output logic                 pwm,
    input logic                  clk,
    input logic                  rstn);

   logic [BAND_WIDTH-1:0]        count_r;

   always_comb begin
      pwm = (count_r < width) ? 1 : 0;
   end

   always_ff @(posedge clk) begin
      if(!rstn) begin
         count_r <= 0;
      end
      else if(count_r < PWM_PERIOD_CYCLE) begin
         count_r <= count_r + 1;
      end
      else begin
         count_r <= 0;
      end
   end

endmodule
