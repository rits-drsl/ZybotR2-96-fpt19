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
// include description of interface for memory bus
`include "if/w_busif.sv"
`include "if/r_busif.sv"

//-----------------------------------------------------------------------------
// module      : uart_sync_memory
// description :
module uart_sync_memory
  #(parameter
    UART_TX_FIFO_DEPTH = 4,
    UART_RX_FIFO_DEPTH = 64,
    UART_BAUD_RATE     = 115200,
    CLK_FREQ           = 100_000_000,
    localparam
    REG_DEPTH    = 256,
    DATA_WIDTH   = 32,
    NUM_M_PORT   = 3,
    LB_REG_DEPTH = $clog2(REG_DEPTH))
   (w_busif.slave w_s0,
    r_busif.slave r_s0,
    w_busif.slave w_s1,
    r_busif.slave r_s1,
    w_busif.slave w_s2,
    r_busif.slave r_s2,

    input logic  uart_rxd,
    output logic uart_txd,

    input logic  clk,
    input logic  rstn);

   r_busif r_mem();
   w_busif w_mem();
   w_busif bulk_rx();
   w_busif bulk_tx();

   reg_interconnect #()
   reg_interconn(.w_s0(w_s0),
                 .r_s0(r_s0),
                 .w_s1(w_s1),
                 .r_s1(r_s1),
                 .w_s2(w_s2),
                 .r_s2(r_s2),
                 .r_mem(r_mem),
                 .w_mem(w_mem),
                 .rstn(rstn),
                 .clk(clk));

   reg_controller #()
   reg_ctrl(.bulk_tx(bulk_tx),
            .bulk_rx(bulk_rx),
            .r_mem(r_mem),
            .w_mem(w_mem),
            .rstn(rstn),
            .clk(clk));

   uart_rx_controller #(.UART_FIFO_DEPTH(UART_RX_FIFO_DEPTH),
                        .UART_BAUD_RATE(UART_BAUD_RATE),
                        .CLK_FREQ(CLK_FREQ))
   uart_rx_ctrl(.bulk_rx(bulk_rx),
                .uart_rxd(uart_rxd),
                .clk(clk),
                .rstn(rstn));

   uart_tx_controller #(.UART_FIFO_DEPTH(UART_TX_FIFO_DEPTH),
                        .UART_BAUD_RATE(UART_BAUD_RATE),
                        .CLK_FREQ(CLK_FREQ))
   uart_tx_ctrl(.bulk_tx(bulk_tx),
                .uart_txd(uart_txd),
                .clk(clk),
                .rstn(rstn));

endmodule
