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
// module      : sync_fifo
// description : Synchronous FIFO consist of Dual Port RAM for FPGA implementation
module sync_fifo
  #(parameter
    /*
     You can specify the following parameters.
     1. DATA_WIDTH : input and output data width
     2. FIFO_DEPTH : data capacity
     */
    DATA_WIDTH    = 8,
    FIFO_DEPTH    = 256,
    localparam
    LB_FIFO_DEPTH = $clog2(FIFO_DEPTH))
   (input logic [DATA_WIDTH-1:0]   in_data,
    input logic                    in_valid,
    output logic                   in_ready,

    output logic [DATA_WIDTH-1:0]  out_data,
    output logic                   out_valid,
    input logic                    out_ready,

    output logic [LB_FIFO_DEPTH:0] count,
    input logic                    clear,

    input logic                    clk,
    input logic                    rstn);

   localparam PREFETCH_FIFO_DEPTH    = 4;
   localparam LB_PREFETCH_FIFO_DEPTH = $clog2(PREFETCH_FIFO_DEPTH);

   logic [LB_FIFO_DEPTH-1:0]       waddr_r, raddr_r;
   logic [LB_FIFO_DEPTH:0]         fifo_count_r, mem_count_r;
   logic [DATA_WIDTH-1:0]          mem_din0, mem_din1;
   logic [DATA_WIDTH-1:0]          mem_dout0, mem_dout1;
   logic                           mem_wr_en0, mem_wr_en1;

   logic                           in_exec, out_exec;

   logic                            prefetch_fifo_in_valid;
   logic                            prefetch_fifo_in_ready;
   logic [LB_PREFETCH_FIFO_DEPTH:0] prefetch_fifo_count;

   logic                            prefetch_exec;
   logic [LB_PREFETCH_FIFO_DEPTH:0] prefetch_count;


   dual_port_RAM #(.DATA_WIDTH(DATA_WIDTH),
                   .RAM_DEPTH(FIFO_DEPTH))
   dp_RAM(.din0(mem_din0),
          .din1(mem_din1),
          .addr0(waddr_r),
          .addr1(raddr_r),
          .dout0(mem_dout0),
          .dout1(mem_dout1),
          .wr_en0(mem_wr_en0),
          .wr_en1(mem_wr_en1),
          .clk(clk));

   reg_fifo #(.DATA_WIDTH(DATA_WIDTH),
              .FIFO_DEPTH(PREFETCH_FIFO_DEPTH))
   prefetch_fifo(.in_data(mem_dout1),
                 .in_valid(prefetch_fifo_in_valid),
                 .in_ready(prefetch_fifo_in_ready),
                 .out_data(out_data),
                 .out_valid(out_valid),
                 .out_ready(out_ready),
                 .clear(clear),
                 .count(prefetch_fifo_count),
                 .clk(clk),
                 .rstn(rstn));

   always_comb begin
      in_ready   = (fifo_count_r < FIFO_DEPTH) ? 1 : 0;
      count      = fifo_count_r;

      in_exec    = in_valid  & in_ready;
      out_exec   = out_valid & out_ready;

      mem_din0   = in_data;
      mem_din1   = 0;
      mem_wr_en0 = in_exec;
      mem_wr_en1 = 0;

      prefetch_count = prefetch_fifo_count + prefetch_fifo_in_valid;
      prefetch_exec  = (0 < mem_count_r) & (prefetch_count < PREFETCH_FIFO_DEPTH);
   end

   always_ff @(posedge clk) begin
      if(!rstn | clear) begin
         waddr_r                  <= 0;
         raddr_r                  <= 0;
         fifo_count_r             <= 0;
         mem_count_r              <= 0;
         prefetch_fifo_in_valid   <= 0;
      end
      else begin
         case({in_exec, out_exec})
           2'b10:   fifo_count_r <= fifo_count_r + 1;
           2'b01:   fifo_count_r <= fifo_count_r - 1;
           default: fifo_count_r <= fifo_count_r;
         endcase

         case({in_exec, prefetch_exec})
           2'b00: begin
              prefetch_fifo_in_valid <= 0;
           end
           2'b01: begin
              raddr_r                <= raddr_r + 1;
              mem_count_r            <= mem_count_r - 1;
              prefetch_fifo_in_valid <= 1;
           end
           2'b10: begin
              waddr_r                <= waddr_r + 1;
              mem_count_r            <= mem_count_r + 1;
              prefetch_fifo_in_valid <= 0;
           end
           2'b11: begin
              waddr_r                <= waddr_r + 1;
              raddr_r                <= raddr_r + 1;
              prefetch_fifo_in_valid <= 1;
           end
         endcase
      end
   end

endmodule
