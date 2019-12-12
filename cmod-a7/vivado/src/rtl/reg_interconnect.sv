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
// module      : reg_interconnect
// description :
module reg_interconnect
  #(localparam
    REG_DEPTH           = 256,
    DATA_WIDTH          = 32,
    NUM_M_PORT          = 3,
    ARBIT_FIFO_DEPTH    = 4, // NUM_M_PORT < ARBIT_FIFO_DEPTH, ARBIT_FIFO_DEPTH == 2^N
    LB_REG_DEPTH        = $clog2(REG_DEPTH),
    LB_NUM_M_PORT       = $clog2(NUM_M_PORT),
    LB_ARBIT_FIFO_DEPTH = $clog2(ARBIT_FIFO_DEPTH))
   (w_busif.slave w_s0,
    r_busif.slave r_s0,
    w_busif.slave w_s1,
    r_busif.slave r_s1,
    w_busif.slave w_s2,
    r_busif.slave r_s2,

    r_busif.master r_mem,
    w_busif.master w_mem,

    input logic    rstn,
    input logic    clk);

   logic [NUM_M_PORT-1:0][LB_REG_DEPTH-1:0] m_r_addr;
   logic [NUM_M_PORT-1:0]                   m_r_valid;
   logic [NUM_M_PORT-1:0][DATA_WIDTH-1:0]   m_w_data;
   logic [NUM_M_PORT-1:0][LB_REG_DEPTH-1:0] m_w_addr;
   logic [NUM_M_PORT-1:0]                   m_w_valid;

   logic                                    m_r_exec, m_w_exec;

   logic [NUM_M_PORT-1:0]                   r_pd_in_data;
   logic [LB_NUM_M_PORT-1:0]                r_pd_index;
   logic                                    r_pd_valid;

   logic [NUM_M_PORT-1:0]                   w_pd_in_data;
   logic [LB_NUM_M_PORT-1:0]                w_pd_index;
   logic                                    w_pd_valid;

   logic                                    r_arbit_fifo_in_ready, w_arbit_fifo_in_ready;
   logic [LB_NUM_M_PORT-1:0]                r_arbit_fifo_out_data, w_arbit_fifo_out_data;
   logic                                    r_arbit_fifo_out_valid, w_arbit_fifo_out_valid;
   logic [LB_ARBIT_FIFO_DEPTH:0]            r_arbit_fifo_count, w_arbit_fifo_count;
   logic                                    r_arbit_fifo_clear, w_arbit_fifo_clear;

   logic                                    r_arbit_fifo_exec, w_arbit_fifo_exec;
   logic [LB_NUM_M_PORT-1:0]                r_arbit_index_r, w_arbit_index_r;
   logic                                    r_arbit_index_is_set_r, w_arbit_index_is_set_r;

   logic                                    r_interconn_ready, w_interconn_ready;

   rr_posedge_detector #(.DATA_WIDTH(NUM_M_PORT))
   r_pd(.in_data(r_pd_in_data),
        .index(r_pd_index),
        .valid(r_pd_valid),
        .ready(r_arbit_fifo_in_ready),
        .clk(clk),
        .rstn(rstn));

   rr_posedge_detector #(.DATA_WIDTH(NUM_M_PORT))
   w_pd(.in_data(w_pd_in_data),
        .index(w_pd_index),
        .valid(w_pd_valid),
        .ready(w_arbit_fifo_in_ready),
        .clk(clk),
        .rstn(rstn));

   sync_fifo #(.DATA_WIDTH(LB_NUM_M_PORT),
               .FIFO_DEPTH(ARBIT_FIFO_DEPTH))
   r_arbitration_fifo(.in_data(r_pd_index),
                      .in_valid(r_pd_valid),
                      .in_ready(r_arbit_fifo_in_ready),
                      .out_data(r_arbit_fifo_out_data),
                      .out_valid(r_arbit_fifo_out_valid),
                      .out_ready(r_interconn_ready),
                      .count(r_arbit_fifo_count),
                      .clear(r_arbit_fifo_clear),
                      .clk(clk),
                      .rstn(rstn));

   sync_fifo #(.DATA_WIDTH(LB_NUM_M_PORT),
               .FIFO_DEPTH(ARBIT_FIFO_DEPTH))
   w_arbitration_fifo(.in_data(w_pd_index),
                      .in_valid(w_pd_valid),
                      .in_ready(w_arbit_fifo_in_ready),
                      .out_data(w_arbit_fifo_out_data),
                      .out_valid(w_arbit_fifo_out_valid),
                      .out_ready(w_interconn_ready),
                      .count(w_arbit_fifo_count),
                      .clear(w_arbit_fifo_clear),
                      .clk(clk),
                      .rstn(rstn));

   always_comb begin
      //--- DPRAMとマスタの接続
      r_s0.data  = (r_arbit_index_is_set_r & (r_arbit_index_r == 0)) ? r_mem.data  : 0;
      r_s1.data  = (r_arbit_index_is_set_r & (r_arbit_index_r == 1)) ? r_mem.data  : 0;
      r_s2.data  = (r_arbit_index_is_set_r & (r_arbit_index_r == 2)) ? r_mem.data  : 0;
      r_s0.ready = (r_arbit_index_is_set_r & (r_arbit_index_r == 0)) ? r_mem.ready : 0;
      r_s1.ready = (r_arbit_index_is_set_r & (r_arbit_index_r == 1)) ? r_mem.ready : 0;
      r_s2.ready = (r_arbit_index_is_set_r & (r_arbit_index_r == 2)) ? r_mem.ready : 0;
      w_s0.ready = (w_arbit_index_is_set_r & (w_arbit_index_r == 0)) ? w_mem.ready : 0;
      w_s1.ready = (w_arbit_index_is_set_r & (w_arbit_index_r == 1)) ? w_mem.ready : 0;
      w_s2.ready = (w_arbit_index_is_set_r & (w_arbit_index_r == 2)) ? w_mem.ready : 0;

      m_r_addr  = {r_s2.addr,  r_s1.addr,  r_s0.addr};
      m_r_valid = {r_s2.valid, r_s1.valid, r_s0.valid};
      m_w_data  = {w_s2.data,  w_s1.data,  w_s0.data};
      m_w_addr  = {w_s2.addr,  w_s1.addr,  w_s0.addr};
      m_w_valid = {w_s2.valid, w_s1.valid, w_s0.valid};

      r_mem.addr  = (r_arbit_index_is_set_r) ? m_r_addr[r_arbit_index_r] : 0;
      r_mem.valid = (r_arbit_index_is_set_r) ? m_r_valid[r_arbit_index_r] : 0;
      w_mem.data  = (w_arbit_index_is_set_r) ? m_w_data[w_arbit_index_r] : 0;
      w_mem.addr  = (w_arbit_index_is_set_r) ? m_w_addr[w_arbit_index_r] : 0;
      w_mem.valid = (w_arbit_index_is_set_r) ? m_w_valid[w_arbit_index_r] : 0;
      //---

      r_pd_in_data = m_r_valid;
      w_pd_in_data = m_w_valid;

      m_r_exec = r_mem.valid & r_mem.ready;
      m_w_exec = w_mem.valid & w_mem.ready;

      r_interconn_ready = (0 < r_arbit_fifo_count) & !r_arbit_index_is_set_r;
      w_interconn_ready = (0 < w_arbit_fifo_count) & !w_arbit_index_is_set_r;

      r_arbit_fifo_exec = r_arbit_fifo_out_valid & r_interconn_ready;
      w_arbit_fifo_exec = w_arbit_fifo_out_valid & w_interconn_ready;

      r_arbit_fifo_clear = 0;
      w_arbit_fifo_clear = 0;
   end

   always_ff @(posedge clk) begin
      if(!rstn) begin
         r_arbit_index_r        <= 0;
         r_arbit_index_is_set_r <= 0;
         w_arbit_index_r        <= 0;
         w_arbit_index_is_set_r <= 0;
      end
      else begin
         if(r_arbit_fifo_exec) begin
            r_arbit_index_r        <= r_arbit_fifo_out_data;
            r_arbit_index_is_set_r <= 1;
         end
         else if(r_arbit_index_is_set_r & m_r_exec) begin
            r_arbit_index_is_set_r <= 0;
         end

         if(w_arbit_fifo_exec) begin
            w_arbit_index_r        <= w_arbit_fifo_out_data;
            w_arbit_index_is_set_r <= 1;
         end
         else if(w_arbit_index_is_set_r & m_w_exec) begin
            w_arbit_index_is_set_r <= 0;
         end
      end
   end

endmodule
