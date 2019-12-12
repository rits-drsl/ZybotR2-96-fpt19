/*
 The MIT License (MIT)
 Copyright (c) 2019 Yuya Kudo.
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
*/

timeunit 1ns;
timeprecision 1ns;

module pid_controller_tb
    #(localparam
      BAND_WIDTH         = 48,
      CLK_FREQ           = 100_000_000,
      ONE_ROTATION_PULSE = 630,
      SAMPLING_RATE      = 100)
   ();

   logic [BAND_WIDTH-1:0] target_rot_v;
   logic [BAND_WIDTH-1:0] p_gain;
   logic [BAND_WIDTH-1:0] i_gain;
   logic [BAND_WIDTH-1:0] d_gain;
   logic [BAND_WIDTH-1:0] rot_cnt;
   logic [BAND_WIDTH-1:0] pulse_width;
   logic                  rstn;
   logic                  clk;

   //-----------------------------------------------------------------------------
   // clock generater
   localparam CLK_PERIOD = 1_000_000_000 / CLK_FREQ;

   initial begin
      clk = 1'b0;
   end

   always #(CLK_PERIOD / 2) begin
      clk = ~clk;
   end

   //-----------------------------------------------------------------------------
   // DUT
   pid_controller #(.BAND_WIDTH(BAND_WIDTH),
                    .CLK_FREQ(CLK_FREQ),
                    .ONE_ROTATION_PULSE(ONE_ROTATION_PULSE),
                    .SAMPLING_RATE(SAMPLING_RATE))
   dut(.target_rot_v(target_rot_v),
       .p_gain(p_gain),
       .i_gain(i_gain),
       .d_gain(d_gain),
       .rot_cnt(rot_cnt),
       .pulse_width(pulse_width),
       .clk(clk),
       .rstn(rstn));

   //-----------------------------------------------------------------------------
   // test scenario
   initial begin
      target_rot_v <= 1_000;
      p_gain       <= 100_0000;
      i_gain       <= 0;
      d_gain       <= 0;
      rot_cnt      <= 0;
      rstn         <= 0;

      repeat(100) @(posedge clk);
      rstn         <= 1;

      while(1) begin
         $display("%d\n", pulse_width);
         rot_cnt <= rot_cnt + 1;
         repeat(200) @(posedge clk);
      end

      $finish;
   end

endmodule
