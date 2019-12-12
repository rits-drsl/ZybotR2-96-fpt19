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
// module     : read_param_ctrl
// description: パラメータ読み込みを行うモジュール
//              読み込みの頻度(interval)の設定が可能
module read_param_ctrl
  #(parameter
    READ_ROT_VELOCITY_INTERVAL = 100,
    READ_GAIN_INTERVAL         = 1000000,
    ADDR_ROTATION_VELOCITY     = 8'h00,
    ADDR_PROPORTION_GAIN       = 8'h00,
    ADDR_INTEGRATION_GAIN      = 8'h00,
    ADDR_DERIVATIVE_GAIN       = 8'h00)
   (r_busif.master r_m,
    output logic [31:0] rot_v,
    output logic [31:0] p_gain,
    output logic [31:0] i_gain,
    output logic [31:0] d_gain,
    input logic         clk,
    input logic         rstn);

   typedef enum logic {STT_PRIOR_ROT_V,
                       STT_PRIOR_GAIN} prior_statetype;

   typedef enum logic {STT_READ_EXEC,
                       STT_READ_WAIT} read_statetype;

   typedef enum logic [1:0] {STT_GAIN_PROP,
                             STT_GAIN_INTG,
                             STT_GAIN_DEFF} gain_statetype;

   prior_statetype prior_state;
   read_statetype  read_rot_v_state, read_gain_state;
   gain_statetype  gain_state;

   logic        read_rot_v_flag, read_gain_flag;
   logic        read_rot_v_flag_r, read_gain_flag_r;
   logic        read_rot_v_cnt_rstn, read_gain_cnt_rstn;

   logic [31:0] rot_v_r;
   logic [31:0] p_gain_r;
   logic [31:0] i_gain_r;
   logic [31:0] d_gain_r;

   logic [31:0] data_r;
   logic [7:0]  addr_r;
   logic        valid_r;
   logic        ready_r;

   logic        read_exec;

   general_cnt #(.INTERVAL(READ_ROT_VELOCITY_INTERVAL))
   read_rot_velocity_cnt(.flag(read_rot_v_flag),
                         .clk(clk),
                         .rstn(read_rot_v_cnt_rstn));

   general_cnt #(.INTERVAL(READ_GAIN_INTERVAL))
   read_gain_cnt(.flag(read_gain_flag),
                 .clk(clk),
                 .rstn(read_gain_cnt_rstn));

   always_comb begin
      read_gain_cnt_rstn  = (read_gain_state  == STT_READ_WAIT) ? 1 : 0;
      read_rot_v_cnt_rstn = (read_rot_v_state == STT_READ_WAIT) ? 1 : 0;

      rot_v     = rot_v_r;
      p_gain    = p_gain_r;
      i_gain    = i_gain_r;
      d_gain    = d_gain_r;

      r_m.addr  = addr_r;
      r_m.valid = valid_r;

      read_exec  = r_m.valid & r_m.ready;
   end

   always_ff @(posedge clk) begin
      if(rstn == 0) begin
         prior_state       <= STT_PRIOR_ROT_V;
         read_rot_v_state  <= STT_READ_WAIT;
         read_gain_state   <= STT_READ_WAIT;
         gain_state        <= STT_GAIN_PROP;
         read_rot_v_flag_r <= 0;
         read_gain_flag_r  <= 0;
         rot_v_r           <= 0;
         p_gain_r          <= 0;
         i_gain_r          <= 0;
         d_gain_r          <= 0;
         data_r            <= 0;
         addr_r            <= 0;
         valid_r           <= 0;
         ready_r           <= 0;
      end
      else begin
         // intervalごとにアサートされるフラグの検出
         if(read_rot_v_flag) read_rot_v_flag_r <= 1;
         if(read_gain_flag)  read_gain_flag_r  <= 1;

         //-----------------------------------------------------------------------------
         // 2 state FSM
         // 排他制御の優先権についての状態遷移
         case(prior_state)
           STT_PRIOR_ROT_V: begin
              //-----------------------------------------------------------------------------
              // 2 state FSM
              // 角速度のread実行についての状態遷移
              case(read_rot_v_state)
                STT_READ_EXEC: begin
                   addr_r  <= ADDR_ROTATION_VELOCITY;
                   valid_r <= 1;

                   // ハンドシェイク成立
                   if(read_exec) begin
                      rot_v_r          <= r_m.data;
                      valid_r          <= 0;
                      read_rot_v_state <= STT_READ_WAIT;
                   end
                end

                STT_READ_WAIT: begin
                   if(read_rot_v_flag_r) begin
                      read_rot_v_state  <= STT_READ_EXEC;
                      read_rot_v_flag_r <= 0;
                   end
                end
              endcase // case (read_rot_v_state)

              // readが実行されていなければ優先権を遷移
              if(!read_rot_v_flag_r && read_rot_v_state == STT_READ_WAIT) begin
                 prior_state <= STT_PRIOR_GAIN;
              end
           end

           STT_PRIOR_GAIN: begin
              //-----------------------------------------------------------------------------
              // 2 state FSM
              // PIDゲインのread実行についての状態遷移
              case(read_gain_state)
                STT_READ_EXEC: begin
                   //-----------------------------------------------------------------------------
                   // 3 state FSM
                   // readするパラメータについての状態遷移
                   case(gain_state)
                     STT_GAIN_PROP: begin
                        addr_r  <= ADDR_PROPORTION_GAIN;
                        valid_r <= 1;

                        // ハンドシェイク成立
                        if(read_exec) begin
                           p_gain_r   <= r_m.data;
                           valid_r    <= 0;
                           gain_state <= STT_GAIN_INTG;
                        end
                     end

                     STT_GAIN_INTG: begin
                        addr_r  <= ADDR_INTEGRATION_GAIN;
                        valid_r <= 1;

                        // ハンドシェイク成立
                        if(read_exec) begin
                           i_gain_r   <= r_m.data;
                           valid_r    <= 0;
                           gain_state <= STT_GAIN_DEFF;
                        end
                     end

                     STT_GAIN_DEFF: begin
                        addr_r  <= ADDR_DERIVATIVE_GAIN;
                        valid_r <= 1;

                        // ハンドシェイク成立
                        if(read_exec) begin
                           d_gain_r        <= r_m.data;
                           valid_r         <= 0;
                           read_gain_state <= STT_READ_WAIT;
                        end
                     end

                     default: begin
                     end
                   endcase // case (gain_state)

                end

                STT_READ_WAIT: begin
                   if(read_gain_flag_r) begin
                      read_gain_state  <= STT_READ_EXEC;
                      gain_state       <= STT_GAIN_PROP;
                      read_gain_flag_r <= 0;
                   end
                end
              endcase // case (read_gain_state)

              // readが実行されていなければ優先権を遷移
              if(!read_gain_flag_r && read_gain_state == STT_READ_WAIT) begin
                 prior_state <= STT_PRIOR_ROT_V;
              end
           end
         endcase // case (prior_state)

      end
   end

endmodule
