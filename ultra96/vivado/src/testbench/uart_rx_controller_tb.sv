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

module uart_rx_controller_tb
  #(localparam
    DATA_WIDTH      = 32,
    RAM_DEPTH       = 256,
    UART_FIFO_DEPTH = 64,
    UART_BAUD_RATE  = 115200,
    CLK_FREQ        = 100_000_000)
  ();

   logic in, clk, rstn;
   w_busif #(DATA_WIDTH, RAM_DEPTH) bulk_rx();

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
   uart_rx_controller #(.UART_FIFO_DEPTH(UART_FIFO_DEPTH),
                        .UART_BAUD_RATE(UART_BAUD_RATE),
                        .CLK_FREQ(CLK_FREQ))
   dut(.uart_rxd(in),
       .bulk_rx(bulk_rx),
       .clk(clk),
       .rstn(rstn));

   //-----------------------------------------------------------------------------
   // test scenario
   localparam UART_DATA_WIDTH    = 9;
   localparam LB_UART_DATA_WIDTH = $clog2(UART_DATA_WIDTH);
   localparam PULSE_WIDTH        = CLK_FREQ / UART_BAUD_RATE;

   int tx_cnt  = 0;
   logic [UART_DATA_WIDTH-1:0] data[6] = {9'h022, 9'h132, 9'h0ff, 9'h0ff, 9'h0ff, 9'h0ff};

   initial begin
      in            <= 1;
      bulk_rx.ready <= 0;
      rstn          <= 0;

      repeat(100) @(posedge clk);
      rstn          <= 1;

      while(tx_cnt < 6) begin
         for(int index = -1; index <= UART_DATA_WIDTH; index++) begin
            case(index)
              -1:              in = 0;
              UART_DATA_WIDTH: in = 1;
              default:         in = data[tx_cnt][index];
            endcase

            repeat(PULSE_WIDTH) @(posedge clk);
         end
         repeat(PULSE_WIDTH / 2) @(posedge clk);

         tx_cnt++;
      end

      bulk_rx.ready <= 1;
      repeat(1) @(posedge clk);
      assert(bulk_rx.addr == data[0] && bulk_rx.data == {data[1], data[2], data[3], data[4]})
        else $error("simulation is fault. bulk_rx.data : %x", bulk_rx.data);

      repeat(100) @(posedge clk);

      $finish;
   end

endmodule
