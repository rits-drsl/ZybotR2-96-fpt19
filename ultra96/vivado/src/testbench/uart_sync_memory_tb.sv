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

module uart_sync_memory_tb
  #(localparam
    DATA_WIDTH         = 32,
    RAM_DEPTH          = 256,
    UART_TX_FIFO_DEPTH = 4,
    UART_RX_FIFO_DEPTH = 64,
    UART_BAUD_RATE     = 115200,
    CLK_FREQ           = 100_000_000)
  ();

   w_busif #(DATA_WIDTH, RAM_DEPTH) w_s0();
   r_busif #(DATA_WIDTH, RAM_DEPTH) r_s0();
   w_busif #(DATA_WIDTH, RAM_DEPTH) w_s1();
   r_busif #(DATA_WIDTH, RAM_DEPTH) r_s1();
   w_busif #(DATA_WIDTH, RAM_DEPTH) w_s2();
   r_busif #(DATA_WIDTH, RAM_DEPTH) r_s2();

   logic  uart_rxd, uart_txd;
   logic  clk, rstn;

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
   uart_sync_memory #(.UART_TX_FIFO_DEPTH(UART_TX_FIFO_DEPTH),
                      .UART_RX_FIFO_DEPTH(UART_RX_FIFO_DEPTH),
                      .UART_BAUD_RATE(UART_BAUD_RATE),
                      .CLK_FREQ(CLK_FREQ))
   dut(.w_s0(w_s0),
       .r_s0(r_s0),
       .w_s1(w_s1),
       .r_s1(r_s1),
       .w_s2(w_s2),
       .r_s2(r_s2),
       .uart_rxd(uart_rxd),
       .uart_txd(uart_txd),
       .clk(clk),
       .rstn(rstn));

   //-----------------------------------------------------------------------------
   // test scenario
   localparam UART_DATA_WIDTH    = 8;
   localparam LB_UART_DATA_WIDTH = $clog2(UART_DATA_WIDTH);
   localparam PULSE_WIDTH        = CLK_FREQ / UART_BAUD_RATE;

   int         cnt     = 0;
   logic [7:0] data[5] = {8'hF0, 8'hF1, 8'hF2, 8'hF3, 8'hF4};

   initial begin
      uart_rxd   <= 1;
      w_s0.data  <= 0;
      w_s0.addr  <= 0;
      w_s0.valid <= 0;
      r_s0.addr  <= 0;
      r_s0.valid <= 0;
      w_s1.data  <= 0;
      w_s1.addr  <= 0;
      w_s1.valid <= 0;
      r_s1.addr  <= 0;
      r_s1.valid <= 0;
      w_s2.data  <= 0;
      w_s2.addr  <= 0;
      w_s2.valid <= 0;
      r_s2.addr  <= 0;
      r_s2.valid <= 0;
      rstn       <= 0;

      repeat(100) @(posedge clk);
      rstn       <= 1;

      //--- rx test
      // generate and input uart_rxd
      while(cnt < 5) begin
         for(int index = -1; index <= UART_DATA_WIDTH; index++) begin
            case(index)
              -1:              uart_rxd = 0;
              UART_DATA_WIDTH: uart_rxd = 1;
              default:         uart_rxd = data[cnt][index];
            endcase

            repeat(PULSE_WIDTH) @(posedge clk);
         end

         cnt++;
      end

      // read memory
      repeat(100) @(posedge clk);
      r_s2.addr  <= 8'h0F;
      r_s2.valid <= 1;

      while(!r_s2.ready) @(posedge clk);
      r_s2.valid <= 0;

      assert(r_s2.data == {data[1], data[2], data[3], data[4]})
        else $error("simulation is fault. r_s2.data : %x", r_s2.data);

      //--- tx test
      for(int i = 0; i < 10; i++) begin
         w_s2.data  <= {data[1], data[2], data[3], data[4]};
         w_s2.addr  <= data[0] + i;
         w_s2.valid <= 1;

         while(!w_s2.ready) @(posedge clk);
         w_s2.valid <= 0;

         repeat(1) @(posedge clk);
      end

      // とりあえず波形見る
      repeat(100000) @(posedge clk);

      $finish;
   end

endmodule
