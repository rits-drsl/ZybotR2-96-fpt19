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

module ultra96_top
  #(localparam
    RAM_DEPTH          = 256,
    LB_RAM_DEPTH       = $clog2(RAM_DEPTH),
    UART_TX_FIFO_DEPTH = 4,
    UART_RX_FIFO_DEPTH = 64,
    UART_BAUD_RATE     = 115200,
    CLK_FREQ           = 50_000_000)
   (input logic       mipi_phy_csi_clk_n, mipi_phy_csi_clk_p,
    input logic [1:0] mipi_phy_csi_data_n, mipi_phy_csi_data_p,
    inout logic       HD_GPIO_0,
    inout logic       HD_GPIO_1,
    inout logic       HD_GPIO_2,
    inout logic       HD_GPIO_3,
    inout logic       HD_GPIO_4,
    inout logic       HD_GPIO_5,
    inout logic       HD_GPIO_6,
    inout logic       HD_GPIO_7,
    inout logic       HD_GPIO_8,
    inout logic       HD_GPIO_9,
    inout logic       HD_GPIO_10,
    inout logic       HD_GPIO_11,
    inout logic       HD_GPIO_12,
    inout logic       HD_GPIO_13,
    inout logic       HD_GPIO_14,
    inout logic       HD_GPIO_15);

   //-----------------------------------------------------------------------------
   // Block Design
   logic        clk, rstn;
   logic [31:0] w_data, r_data;
   logic [31:0] o_ctrl, i_ctrl;

   design_1_wrapper #()
   d1wrap(.mipi_phy_csi_clk_n(mipi_phy_csi_clk_n),
          .mipi_phy_csi_clk_p(mipi_phy_csi_clk_p),
          .mipi_phy_csi_data_n(mipi_phy_csi_data_n),
          .mipi_phy_csi_data_p(mipi_phy_csi_data_p),
          .w_data(w_data),
          .r_data(r_data),
          .o_ctrl(o_ctrl),
          .i_ctrl(i_ctrl),
          .clk_50M(clk),
          .rstn(rstn));

   //-----------------------------------------------------------------------------
   // register synchronization module
   w_busif w_0();
   r_busif r_0();

   logic        uart_rxd;
   logic        uart_rxd_q1, uart_rxd_q2, uart_rxd_q3;
   logic        uart_txd;

   uart_sync_memory #(.UART_TX_FIFO_DEPTH(UART_TX_FIFO_DEPTH),
                      .UART_RX_FIFO_DEPTH(UART_RX_FIFO_DEPTH),
                      .UART_BAUD_RATE(UART_BAUD_RATE),
                      .CLK_FREQ(CLK_FREQ))
   uart_sync_mem_inst(.w_s0(w_0),
                      .r_s0(r_0),
                      .uart_rxd(uart_rxd_q3),
                      .uart_txd(uart_txd),
                      .clk(clk),
                      .rstn(rstn));

   // メタステーブル対策
   always_ff @(posedge clk) begin
      if(!rstn) begin
         uart_rxd_q1 <= 1;
         uart_rxd_q2 <= 1;
         uart_rxd_q3 <= 1;
      end
      else begin
         uart_rxd_q3 <= uart_rxd_q2;
         uart_rxd_q2 <= uart_rxd_q1;
         uart_rxd_q1 <= uart_rxd;
      end
   end

   //-----------------------------------------------------------------------------
   // connection with register synchronization module
   //
   //--- o_ctrl (mode = 0 : read, mode = 1 : write)
   // | reserved | mode | start | addr |
   // 31         9      8       7      0
   //
   //--- i_ctrl
   // | reserved | done |
   // 31         1      0

   typedef enum logic [1:0] {STT_WRITE,
                             STT_READ,
                             STT_WAIT} statetype;
   statetype state;
   logic [7:0]  addr_r;
   logic [31:0] r_data_r, w_data_r;

   logic [1:0]  start_q;
   logic        done_r;
   logic        r_exec, w_exec;
   logic        r_valid_r, w_valid_r;

   always_comb begin
      i_ctrl    = done_r;
      r_data    = r_data_r;

      r_0.addr  = addr_r;
      r_0.valid = r_valid_r;
      w_0.data  = w_data_r;
      w_0.addr  = addr_r;
      w_0.valid = w_valid_r;

      r_exec    = r_0.valid & r_0.ready;
      w_exec    = w_0.valid & w_0.ready;
   end

   always_ff @(posedge clk) begin
      if(!rstn) begin
         state     <= STT_WAIT;
         addr_r    <= 0;
         r_data_r  <= 0;
         w_data_r  <= 0;
         start_q   <= 0;
         done_r    <= 0;
         r_valid_r <= 0;
         w_valid_r <= 0;
      end
      else begin
         start_q[1] <= start_q[0];
         start_q[0] <= o_ctrl[8];

         //-----------------------------------------------------------------------------
         // 3state FSM
         case(state)
           STT_WRITE: begin
              if(w_exec) begin
                 w_valid_r <= 0;
                 done_r    <= 1;
                 state     <= STT_WAIT;
              end
              else begin
                 w_valid_r <= 1;
              end
           end

           STT_READ: begin
              if(r_exec) begin
                 r_valid_r <= 0;
                 r_data_r  <= r_0.data;
                 done_r    <= 1;
                 state     <= STT_WAIT;
              end
              else begin
                 r_valid_r <= 1;
              end
           end

           STT_WAIT: begin
              if(start_q[0] == 0) begin
                 done_r <= 0;
              end

              if(start_q[0] == 1 && start_q[1] == 0) begin
                 addr_r <= o_ctrl[7:0];

                 // write mode
                 if(o_ctrl[9]) begin
                    w_data_r <= w_data;
                    state    <= STT_WRITE;
                 end
                 // read mode
                 else begin
                    state    <= STT_READ;
                 end
              end
           end
         endcase

      end
   end

   //-----------------------------------------------------------------------------
   // GPIO connection
   assign HD_GPIO_0  = 1'bz;
   assign HD_GPIO_1  = 1'bz;
   assign HD_GPIO_2  = 1'bz;
   assign HD_GPIO_3  = 1'bz;
   assign HD_GPIO_4  = 1'bz;
   assign HD_GPIO_5  = 1'bz;
   assign HD_GPIO_6  = uart_rxd;
   assign HD_GPIO_7  = 1'bz;
   assign HD_GPIO_8  = 1'bz;
   assign HD_GPIO_9  = 1'bz;
   assign HD_GPIO_10 = 1'bz;
   assign HD_GPIO_11 = 1'bz;
   assign HD_GPIO_12 = 1'bz;
   assign HD_GPIO_13 = uart_txd;
   assign HD_GPIO_14 = 1'bz;
   assign HD_GPIO_1  = 1'bz;

endmodule
