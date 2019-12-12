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
// module      : rr_posedge_detector
// description : 入力の各ビットに対して立ち上がりエッジを観察し、
//               立ち上がりが観測されたビットのindexをラウンドロビン(rr)の要領で出力する
module rr_posedge_detector
  #(parameter
    DATA_WIDTH = 8,
    localparam
    LB_DATA_WIDTH = $clog2(DATA_WIDTH))
   (input  logic [DATA_WIDTH-1:0]    in_data,
    output logic [LB_DATA_WIDTH-1:0] index,
    output logic                     valid,
    input  logic                     ready,
    input  logic                     clk,
    input  logic                     rstn);

   logic [LB_DATA_WIDTH-1:0]         index_r;
   logic                             valid_r;
   logic                             out_exec;

   logic [DATA_WIDTH-1:0]            in_data_q;
   logic [DATA_WIDTH-1:0]            posedge_flag;
   logic [LB_DATA_WIDTH-1:0]         cnt_r;

   always_comb begin
      index    = index_r;
      valid    = valid_r;
      out_exec = valid & ready;
   end

   always_ff @(posedge clk) begin
      if(!rstn) begin
         index_r      <= 0;
         valid_r      <= 0;
         in_data_q    <= 0;
         posedge_flag <= 0;
         cnt_r        <= 0;
      end
      else begin
         // 立ち上がりエッジの検出を行う
         in_data_q    <= in_data;
         posedge_flag <= (~in_data_q & in_data) | posedge_flag;

         // カウンタの値が立ち上がりエッジを検出したビットを指したとき、
         // valid-readyハンドシェイクが成立するまでカウンタのインクリメントを止める
         if(posedge_flag[cnt_r]) begin
            index_r <= cnt_r;
            valid_r <= 1;
         end
         else begin
            if(cnt_r < DATA_WIDTH) begin
               cnt_r <= cnt_r + 1;
            end
            else begin
               cnt_r <= 0;
            end
         end

         if(out_exec) begin
            posedge_flag[cnt_r] <= 0;
            valid_r             <= 0;
         end
      end
   end

endmodule
