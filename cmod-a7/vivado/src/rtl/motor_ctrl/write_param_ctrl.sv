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

`include "../if/w_busif.sv"
`include "../if/r_busif.sv"

//-----------------------------------------------------------------------------
// module     : write_param_ctrl
// description: パラメータ書き込みを行うモジュール
//              読み込みの頻度(interval)の設定が可能
module write_param_ctrl
  #(parameter
    ADDR_ROT_CNT           = 8'h00,
    WRITE_ROT_CNT_INTERVAL = 1000000,
    WRITE_ROT_CNT_OFFSET   = 0)
    (w_busif.master     w_m,
     input logic [31:0] rot_cnt,
     input logic        clk,
     input logic        rstn);

   typedef enum logic [1:0] {STT_WRITE_EXEC,
                             STT_WRITE_WAIT,
                             STT_WRITE_WAIT_OFFSET} write_statetype;

   write_statetype  write_rot_cnt_state;

   logic        write_rot_cnt_flag;
   logic        write_rot_cnt_flag_r;
   logic        write_rot_cnt_rstn;

   logic [31:0] rot_cnt_r;

   logic [31:0] offset_cnt_r;
   logic [31:0] data_r;
   logic [7:0]  addr_r;
   logic        valid_r;
   logic        ready_r;

   logic        write_exec;

   general_cnt #(.INTERVAL(WRITE_ROT_CNT_INTERVAL))
   write_rot_cnt(.flag(write_rot_cnt_flag),
                 .clk(clk),
                 .rstn(write_rot_cnt_rstn));

   always_comb begin
      write_rot_cnt_rstn  = (write_rot_cnt_state  == STT_WRITE_WAIT) ? 1 : 0;

      w_m.data  = data_r;
      w_m.addr  = addr_r;
      w_m.valid = valid_r;

      write_exec  = w_m.valid & w_m.ready;
   end

   always_ff @(posedge clk) begin
      if(!rstn) begin
         write_rot_cnt_state  <= STT_WRITE_WAIT_OFFSET;
         offset_cnt_r         <= 0;
         data_r               <= 0;
         addr_r               <= 0;
         valid_r              <= 0;
         rot_cnt_r            <= 0;
         write_rot_cnt_flag_r <= 0;
      end
      else begin
         if(write_rot_cnt_flag) write_rot_cnt_flag_r <= 1;

         //-----------------------------------------------------------------------------
         // 2 state FSM
         // カウンタのwrite実行についての状態遷移
         case(write_rot_cnt_state)
           STT_WRITE_WAIT_OFFSET: begin
              offset_cnt_r <= offset_cnt_r + 1;
              if(offset_cnt_r == WRITE_ROT_CNT_OFFSET) begin
                 write_rot_cnt_state <= STT_WRITE_WAIT;
              end
           end
           STT_WRITE_EXEC: begin
              data_r  <= rot_cnt_r;
              addr_r  <= ADDR_ROT_CNT;
              valid_r <= 1;

              // ハンドシェイク成立
              if(write_exec) begin
                 valid_r             <= 0;
                 write_rot_cnt_state <= STT_WRITE_WAIT;
              end
           end

           STT_WRITE_WAIT: begin
              if(write_rot_cnt_flag_r) begin
                 rot_cnt_r            <= rot_cnt;
                 write_rot_cnt_state  <= STT_WRITE_EXEC;
                 write_rot_cnt_flag_r <= 0;
              end
           end
         endcase // case (write_rot_cnt_state)

      end
   end


endmodule
