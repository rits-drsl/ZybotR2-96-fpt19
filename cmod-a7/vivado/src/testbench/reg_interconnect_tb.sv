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

`include "../rtl/if/r_busif.sv";
`include "../rtl/if/w_busif.sv";

module reg_interconnect_tb
  #(localparam
    DATA_WIDTH   = 32,
    RAM_DEPTH    = 256,
    LB_RAM_DEPTH = $clog2(RAM_DEPTH),
    CLK_FREQ     = 100_000_000)
   ();

   w_busif #(DATA_WIDTH, RAM_DEPTH) w_s0();
   r_busif #(DATA_WIDTH, RAM_DEPTH) r_s0();
   w_busif #(DATA_WIDTH, RAM_DEPTH) w_s1();
   r_busif #(DATA_WIDTH, RAM_DEPTH) r_s1();
   w_busif #(DATA_WIDTH, RAM_DEPTH) w_s2();
   r_busif #(DATA_WIDTH, RAM_DEPTH) r_s2();
   r_busif #(DATA_WIDTH, RAM_DEPTH) r_mem();
   w_busif #(DATA_WIDTH, RAM_DEPTH) w_mem();

   logic                    rstn;
   logic                    clk;

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
   reg_interconnect dut(.w_s0(w_s0),
                        .r_s0(r_s0),
                        .w_s1(w_s1),
                        .r_s1(r_s1),
                        .w_s2(w_s2),
                        .r_s2(r_s2),
                        .r_mem(r_mem),
                        .w_mem(w_mem),
                        .rstn(rstn),
                        .clk(clk));

   //-----------------------------------------------------------------------------
   // test scenario
   initial begin
      rstn       <= 0;
      w_s0.data  <= 0;
      w_s1.data  <= 0;
      w_s2.data  <= 0;
      w_s0.addr  <= 0;
      w_s1.addr  <= 0;
      w_s2.addr  <= 0;
      w_s0.valid <= 0;
      w_s1.valid <= 0;
      w_s2.valid <= 0;
      r_s0.addr  <= 0;
      r_s1.addr  <= 0;
      r_s2.addr  <= 0;
      r_s0.valid <= 0;
      r_s1.valid <= 0;
      r_s2.valid <= 0;

      r_mem.data  <= 0;
      r_mem.ready <= 0;
      w_mem.ready <= 0;

      repeat(100) @(posedge clk);
      rstn       <= 1;

      // m0 read
      repeat(1) @(posedge clk);
      r_s0.addr  <= 32'h20;
      r_s0.valid <= 1;

      while(!r_mem.valid) @(posedge clk);
      r_mem.data  <= 32'h10;
      r_mem.ready <= 1;
      repeat(1) @(posedge clk);
      r_s0.valid  <= 0;
      repeat(1) @(posedge clk);
      r_mem.ready <= 0;

      // m1 write & m1 read
      repeat(1) @(posedge clk);
      w_s1.data  <= 32'h77;
      w_s1.addr  <= 32'h0F;
      w_s1.valid <= 1;

      r_s1.addr  <= 32'h0F;
      r_s1.valid <= 1;

      while(!w_mem.valid) @(posedge clk);
      w_mem.ready <= 1;
      repeat(1) @(posedge clk);
      w_s1.valid <= 0;

      while(!r_mem.valid) @(posedge clk);
      r_mem.data  <= 32'h20;
      r_mem.ready <= 1;
      repeat(1) @(posedge clk);
      r_s1.valid  <= 0;
      repeat(1) @(posedge clk);
      r_mem.ready <= 0;

      // m2 write
      repeat(1) @(posedge clk);
      w_s2.data  <= 32'h79;
      w_s2.addr  <= 32'h08;
      w_s2.valid <= 1;

      while(!w_mem.valid) @(posedge clk);
      w_mem.ready <= 1;

      repeat(1) @(posedge clk);
      w_s2.valid <= 0;

      // m0 write & m2 read
      repeat(1) @(posedge clk);
      w_s0.data  <= 32'h80;
      w_s0.addr  <= 32'h04;
      w_s0.valid <= 1;

      r_s2.addr  <= 32'h0F;
      r_s2.valid <= 1;

      while(!w_mem.valid) @(posedge clk);
      w_mem.ready <= 1;

      repeat(1) @(posedge clk);
      w_s0.valid  <= 0;

      while(!r_mem.valid) @(posedge clk);
      r_mem.data  <= 32'h20;
      r_mem.ready <= 1;

      repeat(1) @(posedge clk);
      r_s2.valid  <= 0;
      repeat(1) @(posedge clk);
      r_mem.ready <= 0;

      repeat(100) @(posedge clk);
      $finish;
   end

endmodule
