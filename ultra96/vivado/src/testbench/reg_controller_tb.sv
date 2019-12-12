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

module reg_controller_tb
  #(localparam
    DATA_WIDTH   = 32,
    RAM_DEPTH    = 256,
    LB_RAM_DEPTH = $clog2(RAM_DEPTH),
    CLK_FREQ     = 100_000_000)
   ();

   w_busif #(DATA_WIDTH, RAM_DEPTH) bulk_tx();
   w_busif #(DATA_WIDTH, RAM_DEPTH) bulk_rx();
   r_busif #(DATA_WIDTH, RAM_DEPTH) r_mem();
   w_busif #(DATA_WIDTH, RAM_DEPTH) w_mem();
   logic                            rstn;
   logic                            clk;

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
   reg_controller dut(.bulk_tx(bulk_tx),
                      .bulk_rx(bulk_rx),
                      .r_mem(r_mem),
                      .w_mem(w_mem),
                      .rstn(rstn),
                      .clk(clk));

   //-----------------------------------------------------------------------------
   // test scenario
   initial begin
      bulk_tx.ready <= 0;
      bulk_rx.data  <= 0;
      bulk_rx.addr  <= 0;
      bulk_rx.valid <= 0;
      r_mem.addr    <= 0;
      r_mem.valid   <= 0;
      w_mem.data    <= 0;
      w_mem.addr    <= 0;
      w_mem.valid   <= 0;
      rstn          <= 0;

      repeat(100) @(posedge clk);
      bulk_tx.ready <= 1;
      rstn          <= 1;

      repeat(1) @(posedge clk);
      bulk_rx.data  <= 32'h12345678;
      bulk_rx.addr  <= 8'hFF;
      bulk_rx.valid <= 1;

      w_mem.data    <= 32'h12121212;
      w_mem.addr    <= 8'h7F;
      w_mem.valid   <= 1;

      repeat(1) @(posedge clk);
      while(bulk_rx.valid | w_mem.valid) begin
         if(bulk_rx.ready) begin
            bulk_rx.valid <= 0;
         end
         if(w_mem.ready) begin
            w_mem.valid <= 0;
         end

         repeat(1) @(posedge clk);
      end

      r_mem.addr    <= 8'h7F;
      r_mem.valid   <= 1;

      repeat(100) @(posedge clk);
      $finish;
   end

endmodule
