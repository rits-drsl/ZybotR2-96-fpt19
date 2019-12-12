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
// module      : uart_rx_controller
// description : UARTデータ受信＆データデコードモジュール
//               | addr | data0 | data1 | data2 | data3 |
//               0      8       16      24      32      40
module uart_rx_controller
  #(parameter
    UART_FIFO_DEPTH    = 64,
    UART_BAUD_RATE     = 115200,
    CLK_FREQ           = 100_000_000,
    localparam
    REG_DEPTH          = 256,
    DATA_WIDTH         = 32,
    UART_DATA_WIDTH    = 9,
    LB_REG_DEPTH       = $clog2(REG_DEPTH),
    LB_UART_FIFO_DEPTH = $clog2(UART_FIFO_DEPTH))
   (w_busif.master bulk_rx,
    input logic    uart_rxd,
    input logic    clk,
    input logic    rstn);

   logic [UART_DATA_WIDTH-1:0]  uart_data;
   logic                        uart_valid;

   logic                        fifo_in_ready;
   logic [UART_DATA_WIDTH-1:0]  fifo_out_data;
   logic                        fifo_out_valid;
   logic                        fifo_out_ready;
   logic [LB_UART_FIFO_DEPTH:0] fifo_count;
   logic                        fifo_clear;

   logic                        fifo_out_exec;

   typedef enum logic [1:0] {STT_ADDR,
                             STT_DATA,
                             STT_BULK,
                             STT_WAIT} bulk_rx_statetype;

   bulk_rx_statetype          bulk_rx_state;
   logic [DATA_WIDTH-1:0]     bulk_rx_data_r;
   logic [LB_REG_DEPTH-1:0]   bulk_rx_addr_r;
   logic                      bulk_rx_valid_r;

   logic [1:0]                bulk_rx_data_cnt_r;
   logic                      bulk_rx_exec;
   logic                      bulk_rx_fetch_exec;

   uart_rx #(.DATA_WIDTH(UART_DATA_WIDTH),
             .BAUD_RATE(UART_BAUD_RATE),
             .CLK_FREQ(CLK_FREQ))
   uart_rx_inst(.uart_in(uart_rxd),
                .data(uart_data),
                .valid(uart_valid),
                .ready(fifo_in_ready),
                .clk(clk),
                .rstn(rstn));

   sync_fifo #(.DATA_WIDTH(UART_DATA_WIDTH),
               .FIFO_DEPTH(UART_FIFO_DEPTH))
   sync_fifo_inst(.in_data(uart_data),
                  .in_valid(uart_valid),
                  .in_ready(fifo_in_ready),
                  .out_data(fifo_out_data),
                  .out_valid(fifo_out_valid),
                  .out_ready(fifo_out_ready),
                  .count(fifo_count),
                  .clear(fifo_clear),
                  .clk(clk),
                  .rstn(rstn));

   always_comb begin
      bulk_rx.data       = bulk_rx_data_r;
      bulk_rx.addr       = bulk_rx_addr_r;
      bulk_rx.valid      = bulk_rx_valid_r;
      bulk_rx_exec       = bulk_rx.valid & bulk_rx.ready;
      bulk_rx_fetch_exec = (5 <= fifo_count);

      fifo_out_ready     = (bulk_rx_state == STT_ADDR) | (bulk_rx_state == STT_DATA);
      fifo_out_exec      = fifo_out_valid & fifo_out_ready;
      fifo_clear         = 0;
   end

   always_ff @(posedge clk) begin
      if(!rstn) begin
         bulk_rx_state      <= STT_WAIT;
         bulk_rx_data_r     <= 0;
         bulk_rx_addr_r     <= 0;
         bulk_rx_valid_r    <= 0;
         bulk_rx_data_cnt_r <= 0;
      end
      else begin
         //-----------------------------------------------------------------------------
         // 4 state FSM about rx data
         case(bulk_rx_state)

           //-----------------------------------------------------------------------------
           // state      : STT_ADDR
           // behavior   : アドレス取得
           // next state : FIFOから1バイト取得 -> STT_DATA
           STT_ADDR: begin
              // 最上位ビット(アドレスを表すフラグ)がアサートされていれば値を取得する
              if((fifo_out_exec) && (fifo_out_data[UART_DATA_WIDTH-1] == 1'b1)) begin
                 bulk_rx_addr_r <= fifo_out_data[UART_DATA_WIDTH-2:0];
                 bulk_rx_state  <= STT_DATA;
              end
           end

           //-----------------------------------------------------------------------------
           // state      : STT_DATA
           // behavior   : データ取得
           // next state : FIFOから4バイト取得 -> STT_BULK
           STT_DATA: begin
              if(fifo_out_exec) begin
                 bulk_rx_data_r     <= {bulk_rx_data_r[DATA_WIDTH-UART_DATA_WIDTH:0], fifo_out_data[UART_DATA_WIDTH-2:0]};
                 bulk_rx_data_cnt_r <= bulk_rx_data_cnt_r + 1;

                 if(bulk_rx_data_cnt_r == 3) begin
                    bulk_rx_valid_r <= 1;
                    bulk_rx_state   <= STT_BULK;
                 end
              end
           end

           //-----------------------------------------------------------------------------
           // state      : STT_BULK
           // behavior   : bulk out待機状態
           // next state : valid-readyハンドシェイク成立 -> STT_WAIT
           STT_BULK: begin
              if(bulk_rx_exec) begin
                 bulk_rx_valid_r <= 0;
                 bulk_rx_state   <= STT_WAIT;
              end
           end

           //-----------------------------------------------------------------------------
           // state      : STT_WAIT
           // behavior   : FIFOにデータが貯まるのを待つ
           // next state : FIFOにデータが5バイト(アドレス1バイト + データ4バイト)以上貯まっている -> STT_ADDR
           STT_WAIT: begin
              if(bulk_rx_fetch_exec) begin
                 bulk_rx_data_cnt_r <= 0;
                 bulk_rx_state      <= STT_ADDR;
              end
           end
         endcase
      end
   end

endmodule
