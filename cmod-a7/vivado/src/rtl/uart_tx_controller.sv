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
// module      : uart_tx_controller
// description :
module uart_tx_controller
  #(parameter
    UART_FIFO_DEPTH = 64,
    UART_BAUD_RATE  = 115200,
    CLK_FREQ        = 100_000_000,
    localparam
    RAM_DEPTH       = 256,
    DATA_WIDTH      = 32,
    UART_DATA_WIDTH = 9,
    LB_RAM_DEPTH    = $clog2(RAM_DEPTH),
    FIFO_DATA_WIDTH = DATA_WIDTH + LB_RAM_DEPTH,
    LB_FIFO_DEPTH   = $clog2(UART_FIFO_DEPTH))
   (w_busif.slave bulk_tx,
    output logic  uart_txd,
    input logic   clk,
    input logic   rstn);

   logic [FIFO_DATA_WIDTH-1:0]  fifo_in_data;
   logic [FIFO_DATA_WIDTH-1:0]  fifo_out_data, fifo_out_data_r;
   logic                        fifo_out_valid;
   logic                        fifo_out_ready;
   logic [LB_FIFO_DEPTH:0]      fifo_count;
   logic                        fifo_clear;

   logic                        fifo_out_exec;

   logic [UART_DATA_WIDTH-1:0]  uart_tx_data, uart_tx_data_r;
   logic                        uart_tx_valid, uart_tx_valid_r;
   logic                        uart_tx_ready;

   typedef enum logic {STT_TX,
                       STT_WAIT} uart_tx_statetype;

   uart_tx_statetype uart_tx_state;
   logic             uart_tx_exec;
   logic [2:0]       uart_tx_cnt_r;

   sync_fifo #(.DATA_WIDTH(FIFO_DATA_WIDTH),
               .FIFO_DEPTH(UART_FIFO_DEPTH))
   sync_fifo_inst(.in_data(fifo_in_data),
                  .in_valid(bulk_tx.valid),
                  .in_ready(bulk_tx.ready),
                  .out_data(fifo_out_data),
                  .out_valid(fifo_out_valid),
                  .out_ready(fifo_out_ready),
                  .count(fifo_count),
                  .clear(fifo_clear),
                  .clk(clk),
                  .rstn(rstn));

   uart_tx #(.DATA_WIDTH(UART_DATA_WIDTH),
             .BAUD_RATE(UART_BAUD_RATE),
             .CLK_FREQ(CLK_FREQ))
   uart_tx_inst(.uart_out(uart_txd),
                .data(uart_tx_data),
                .valid(uart_tx_valid),
                .ready(uart_tx_ready),
                .clk(clk),
                .rstn(rstn));

   always_comb begin
      fifo_in_data   = {bulk_tx.addr, bulk_tx.data};
      fifo_out_ready = (uart_tx_state == STT_WAIT);
      fifo_out_exec  = fifo_out_valid & fifo_out_ready;
      fifo_clear     = 0;

      uart_tx_data   = uart_tx_data_r;
      uart_tx_valid  = uart_tx_valid_r;
      uart_tx_exec   = uart_tx_valid & uart_tx_ready;
   end

   always_ff @(posedge clk) begin
      if(!rstn) begin
         fifo_out_data_r <= 0;
         uart_tx_data_r  <= 0;
         uart_tx_valid_r <= 0;
         uart_tx_cnt_r   <= 0;
         uart_tx_state   <= STT_WAIT;
      end
      else begin
         //-----------------------------------------------------------------------------
         // 2 state FSM about bulk tx data
         case(uart_tx_state)

           //-----------------------------------------------------------------------------
           // state      : STT_TX
           // behavior   : データを送信する
           // next state : 5バイトの送信が終了 -> STT_WAIT
           STT_TX: begin
              if(uart_tx_exec || uart_tx_cnt_r == 0) begin
                 case(uart_tx_cnt_r)
                   // 最上位ビットが1 -> アドレス, 最上位ビットが0 -> データ
                   3'b000:  uart_tx_data_r <= {1'b1, fifo_out_data_r[39:32]};
                   3'b001:  uart_tx_data_r <= {1'b0, fifo_out_data_r[31:24]};
                   3'b010:  uart_tx_data_r <= {1'b0, fifo_out_data_r[23:16]};
                   3'b011:  uart_tx_data_r <= {1'b0, fifo_out_data_r[15:8]};
                   3'b100:  uart_tx_data_r <= {1'b0, fifo_out_data_r[7:0]};
                   default: uart_tx_state  <= STT_WAIT;
                 endcase

                 if(uart_tx_cnt_r < 5) begin
                    uart_tx_valid_r <= 1;
                    uart_tx_cnt_r   <= uart_tx_cnt_r + 1;
                 end
                 else begin
                    uart_tx_valid_r <= 0;
                    uart_tx_cnt_r   <= 0;
                 end
              end
           end

           //-----------------------------------------------------------------------------
           // state      : STT_WAIT
           // behavior   : FIFOにデータが貯まるのを待つ
           // next state : FIFOにデータが貯まっている -> STT_TX
           STT_WAIT: begin
              if(fifo_out_exec) begin
                 fifo_out_data_r <= fifo_out_data;
                 uart_tx_state   <= STT_TX;
              end
           end
         endcase
      end
   end

endmodule
