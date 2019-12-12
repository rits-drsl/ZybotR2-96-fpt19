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
// module      : reg_controller
// description : DPRAMを内包し、データの読み込み・書き込みを行うモジュール
//               データの更新があればUARTでデータを転送する
module reg_controller
  #(localparam
    REG_DEPTH    = 256,
    DATA_WIDTH   = 32,
    LB_REG_DEPTH = $clog2(REG_DEPTH))
   (w_busif.master bulk_tx,
    w_busif.slave  bulk_rx,
    r_busif.slave  r_mem,
    w_busif.slave  w_mem,

    input logic    rstn,
    input logic    clk);

   logic [DATA_WIDTH-1:0]   din0;
   logic [DATA_WIDTH-1:0]   din1;
   logic [LB_REG_DEPTH-1:0] addr0;
   logic [LB_REG_DEPTH-1:0] addr1;
   logic [DATA_WIDTH-1:0]   dout0;
   logic [DATA_WIDTH-1:0]   dout1;
   logic                    wr_en0;
   logic                    wr_en1;

   logic                    bulk_tx_valid_r;
   logic                    bulk_rx_valid_r;
   logic                    bulk_rx_ready_r;

   logic [1:0][DATA_WIDTH-1:0]   mem_w_data_q;
   logic [1:0][LB_REG_DEPTH-1:0] mem_w_addr_q;
   logic                         mem_w_valid_r;
   logic                         mem_w_ready_r;
   logic                         r_mem_ready_r;

   logic                         w_sel_r;

   logic                         mem_w_exec, bulk_tx_exec, bulk_rx_exec;

   // 0 : write port, 1 : read port
   dual_port_RAM #(.DATA_WIDTH(DATA_WIDTH),
                   .RAM_DEPTH(REG_DEPTH))
   dpram(.din0(din0),
         .din1(din1),
         .addr0(addr0),
         .addr1(addr1),
         .dout0(dout0),
         .dout1(dout1),
         .wr_en0(wr_en0),
         .wr_en1(wr_en1),
         .clk(clk));

   always_comb begin
      bulk_tx.data  = mem_w_data_q[1];
      bulk_tx.addr  = mem_w_addr_q[1];
      bulk_tx.valid = bulk_tx_valid_r;

      bulk_rx.ready = w_sel_r & mem_w_ready_r;

      w_mem.ready = ~w_sel_r & mem_w_ready_r;

      r_mem.data  = dout1;
      r_mem.ready = r_mem_ready_r;

      din0   = mem_w_data_q[0];
      din1   = 0;
      addr0  = mem_w_addr_q[0];
      addr1  = r_mem.addr;
      wr_en0 = mem_w_exec;
      wr_en1 = 0;

      bulk_tx_exec = bulk_tx.valid & bulk_tx.ready;
      mem_w_exec   = mem_w_valid_r & mem_w_ready_r;
   end

   always_ff @(posedge clk) begin
      if(!rstn) begin
         w_sel_r         <= 0;
         mem_w_data_q    <= 0;
         mem_w_addr_q    <= 0;
         mem_w_valid_r   <= 0;
         mem_w_ready_r   <= 0;
         r_mem_ready_r   <= 0;
         bulk_tx_valid_r <= 0;
      end
      else begin
         mem_w_data_q[1] <= mem_w_data_q[0];
         mem_w_addr_q[1] <= mem_w_addr_q[0];

         if(bulk_tx_valid_r == 0) begin
            // DPRAMへwriteを行うポートが２つあるため、
            // セレクタを用いて値を交互に参照する
            w_sel_r         <= ~w_sel_r;
            mem_w_data_q[0] <= (w_sel_r) ? bulk_rx.data  : w_mem.data;
            mem_w_addr_q[0] <= (w_sel_r) ? bulk_rx.addr  : w_mem.addr;
            mem_w_valid_r   <= (w_sel_r) ? bulk_rx.valid : w_mem.valid;
            mem_w_ready_r   <= 1;
         end
         else begin
            // uart_tx_controllerのFIFOにデータがfetchされるまでは
            // データの取得を止める
            w_sel_r         <= w_sel_r;
            mem_w_data_q[0] <= mem_w_data_q[1];
            mem_w_addr_q[0] <= mem_w_addr_q[1];
            mem_w_valid_r   <= mem_w_valid_r;
            mem_w_ready_r   <= 0;
         end

         // DPRAMからデータ読み込む際
         // 1cycleのdelayがある
         if(r_mem.valid) begin
            r_mem_ready_r <= 1;
         end
         else begin
            r_mem_ready_r <= 0;
         end

         // masterからDPRAMへの書き込みがあった場合のみ
         // UARTによるデータ転送を行う
         if(mem_w_exec & w_sel_r) begin
            bulk_tx_valid_r <= 1;
         end
         else if(bulk_tx_exec) begin
            bulk_tx_valid_r <= 0;
         end
         else begin
            bulk_tx_valid_r <= bulk_tx_valid_r;
         end
      end
   end

endmodule
