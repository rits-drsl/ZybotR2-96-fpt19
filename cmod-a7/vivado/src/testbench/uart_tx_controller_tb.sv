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

module uart_tx_controller_tb
    #(localparam
      DATA_WIDTH      = 32,
      RAM_DEPTH       = 256,
      UART_FIFO_DEPTH = 64,
      UART_BAUD_RATE  = 115200,
      CLK_FREQ        = 100_000_000)
   ();

   logic                            out, clk, rstn;
   w_busif #(DATA_WIDTH, RAM_DEPTH) bulk_tx();

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
   uart_tx_controller #(.UART_FIFO_DEPTH(UART_FIFO_DEPTH),
                        .UART_BAUD_RATE(UART_BAUD_RATE),
                        .CLK_FREQ(CLK_FREQ))
   dut(.uart_txd(out),
       .bulk_tx(bulk_tx),
       .clk(clk),
       .rstn(rstn));

   //-----------------------------------------------------------------------------
   // test scenario
   localparam UART_DATA_WIDTH    = 9;
   localparam LB_UART_DATA_WIDTH = $clog2(UART_DATA_WIDTH);
   localparam PULSE_WIDTH        = CLK_FREQ / UART_BAUD_RATE;

   logic [7:0]  addr[2] = {8'hFF, 8'h12};
   logic [31:0] data[2] = {32'h0000FFFF, 32'h12345678};

   initial begin
      bulk_tx.data  <= 0;
      bulk_tx.addr  <= 0;
      bulk_tx.valid <= 0;
      rstn          <= 0;

      repeat(100) @(posedge clk);
      rstn           <= 1;

      repeat(1) @(posedge clk);
      while(!bulk_tx.ready) @(posedge clk);
      bulk_tx.data  <= data[0];
      bulk_tx.addr  <= addr[0];
      bulk_tx.valid <= 1;

      repeat(1) @(posedge clk);
      while(!bulk_tx.ready) @(posedge clk);
      bulk_tx.data  <= data[1];
      bulk_tx.addr  <= addr[1];
      bulk_tx.valid <= 1;

      repeat(1) @(posedge clk);
      while(!bulk_tx.ready) @(posedge clk);
      bulk_tx.valid <= 0;

      repeat(100000) @(posedge clk);
      $finish;
   end

endmodule
