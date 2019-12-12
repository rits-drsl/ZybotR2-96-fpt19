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

module Cmod_A7_top
  #(localparam
    RAM_DEPTH          = 256,
    LB_RAM_DEPTH       = $clog2(RAM_DEPTH),
    UART_TX_FIFO_DEPTH = 4,
    UART_RX_FIFO_DEPTH = 64,
    UART_BAUD_RATE     = 115200,
    CLK_FREQ           = 50_000_000,
    MOTOR_READ_ROT_VELOCITY_INTERVAL = 100,
    MOTOR_READ_GAIN_INTERVAL         = 100000,
    MOTOR_WRITE_ROT_CNT_INTERVAL     = 200000,  // 4ms
    R_MOTOR_WRITE_ROT_CNT_OFFSET     = 100000,
    L_MOTOR_WRITE_ROT_CNT_OFFSET     = 0,
    BTN_WRITE_INTERVAL               = 4999999) // 100ms
   (input logic        sysclk,
    input logic [1:0]  btn,

    output logic [1:0] led,
    output logic       led0_r, led0_g, led0_b,

    inout logic [7:0]  ja,
    inout logic        pio1, pio2, pio3, pio4, pio5,
                       pio6, pio7, pio8, pio9, pio10,
                       pio11, pio12, pio13, pio14, pio17,
                       pio18, pio19, pio20, pio21, pio22,
                       pio23, pio26, pio27, pio28, pio29,
                       pio30, pio31, pio32, pio33, pio34,
                       pio35, pio36, pio37, pio38, pio39,
                       pio40, pio41, pio42, pio43, pio44,
                       pio45, pio46, pio47, pio48);

   //-----------------------------------------------------------------------------
   // include memory map information
   `include "mem/mem_map.sv"

   //-----------------------------------------------------------------------------
   // Block Design
   logic        clk, rstn;
   design_1_wrapper #()
     d1wrap(.sysclk(sysclk),
            .clk50M(clk),
            .rstn(rstn));

   //-----------------------------------------------------------------------------
   // register syncronization module
   w_busif w_0();
   w_busif w_1();
   w_busif w_2();
   r_busif r_0();
   r_busif r_1();
   r_busif r_2();

   logic        uart_rxd;
   logic        uart_txd;
   logic        uart_rxd_q1, uart_rxd_q2, uart_rxd_q3;

   uart_sync_memory #(.UART_TX_FIFO_DEPTH(UART_TX_FIFO_DEPTH),
                      .UART_RX_FIFO_DEPTH(UART_RX_FIFO_DEPTH),
                      .UART_BAUD_RATE(UART_BAUD_RATE),
                      .CLK_FREQ(CLK_FREQ))
   uart_sync_mem_inst(.w_s0(w_0), // 使用箇所 : 右モータ制御モジュール
                      .r_s0(r_0), // 使用箇所 : 右モータ制御モジュール
                      .w_s1(w_1), // 使用箇所 : 左モータ制御モジュール
                      .r_s1(r_1), // 使用箇所 : 左モータ制御モジュール
                      .w_s2(w_2), // 使用箇所 : ボタン
                      .r_s2(r_2), // 使用箇所 : なし
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
   // motor ctrl

   logic        r_pwm, l_pwm;
   logic        r_dir, l_dir;
   logic [31:0] r_pulse_width, l_pulse_width;
   logic [31:0] r_rot_cnt, l_rot_cnt;
   logic [31:0] r_target_rot_v, l_target_rot_v;
   logic        r_sa, r_sb, l_sa, l_sb;
   logic        r_sa_q1, r_sa_q2, r_sa_q3;
   logic        r_sb_q1, r_sb_q2, r_sb_q3;
   logic        l_sa_q1, l_sa_q2, l_sa_q3;
   logic        l_sb_q1, l_sb_q2, l_sb_q3;

   motor_ctrl #(.CLK_FREQ(CLK_FREQ),
                .READ_ROT_VELOCITY_INTERVAL(MOTOR_READ_ROT_VELOCITY_INTERVAL),
                .READ_GAIN_INTERVAL(MOTOR_READ_GAIN_INTERVAL),
                .WRITE_ROT_CNT_INTERVAL(MOTOR_WRITE_ROT_CNT_INTERVAL),
                .WRITE_ROT_CNT_OFFSET(R_MOTOR_WRITE_ROT_CNT_OFFSET),
                .ADDR_ROTATION_VELOCITY(ADDR_R_ROTATION_VELOCITY),
                .ADDR_PROPORTION_GAIN(ADDR_R_PROPORTION_GAIN),
                .ADDR_INTEGRATION_GAIN(ADDR_R_INTEGRATION_GAIN),
                .ADDR_DERIVATIVE_GAIN(ADDR_R_DERIVATIVE_GAIN),
                .ADDR_ROT_CNT(ADDR_R_ROT_CNT))
   right_motor_ctrl_inst(.w_m(w_0),
                         .r_m(r_0),
                         .pwm(r_pwm),
                         .dir(r_dir),
                         .pulse_width(r_pulse_width),
                         .rot_cnt(r_rot_cnt),
                         .target_rot_v(r_target_rot_v),
                         .sa(r_sa_q3),
                         .sb(r_sb_q3),
                         .clk(clk),
                         .rstn(rstn));

   motor_ctrl #(.CLK_FREQ(CLK_FREQ),
                .READ_ROT_VELOCITY_INTERVAL(MOTOR_READ_ROT_VELOCITY_INTERVAL),
                .READ_GAIN_INTERVAL(MOTOR_READ_GAIN_INTERVAL),
                .WRITE_ROT_CNT_INTERVAL(MOTOR_WRITE_ROT_CNT_INTERVAL),
                .WRITE_ROT_CNT_OFFSET(L_MOTOR_WRITE_ROT_CNT_OFFSET),
                .ADDR_ROTATION_VELOCITY(ADDR_L_ROTATION_VELOCITY),
                .ADDR_PROPORTION_GAIN(ADDR_L_PROPORTION_GAIN),
                .ADDR_INTEGRATION_GAIN(ADDR_L_INTEGRATION_GAIN),
                .ADDR_DERIVATIVE_GAIN(ADDR_L_DERIVATIVE_GAIN),
                .ADDR_ROT_CNT(ADDR_L_ROT_CNT))
   left_motor_ctrl_inst(.w_m(w_1),
                        .r_m(r_1),
                        .pwm(l_pwm),
                        .dir(l_dir),
                        .pulse_width(l_pulse_width),
                        .rot_cnt(l_rot_cnt),
                        .target_rot_v(l_target_rot_v),
                        .sa(l_sa_q3),
                        .sb(l_sb_q3),
                        .clk(clk),
                        .rstn(rstn));

   // メタステーブル対策
   always_ff @(posedge clk) begin
      if(!rstn) begin
         r_sa_q1 <= 0;
         r_sa_q2 <= 0;
         r_sa_q3 <= 0;
         r_sb_q1 <= 0;
         r_sb_q2 <= 0;
         r_sb_q3 <= 0;
         l_sa_q1 <= 0;
         l_sa_q2 <= 0;
         l_sa_q3 <= 0;
         l_sb_q1 <= 0;
         l_sb_q2 <= 0;
         l_sb_q3 <= 0;
      end
      else begin
         r_sa_q3 <= r_sa_q2;
         r_sa_q2 <= r_sa_q1;
         r_sa_q1 <= r_sa;
         r_sb_q3 <= r_sb_q2;
         r_sb_q2 <= r_sb_q1;
         r_sb_q1 <= r_sb;
         l_sa_q3 <= l_sa_q2;
         l_sa_q2 <= l_sa_q1;
         l_sa_q1 <= l_sa;
         l_sb_q3 <= l_sb_q2;
         l_sb_q2 <= l_sb_q1;
         l_sb_q1 <= l_sb;
      end
   end

   //-----------------------------------------------------------------------------
   // OLED ctrl
   logic [31:0] oled_dbg1;
   logic [31:0] oled_dbg2;
   logic [31:0] oled_dbg3;
   logic [31:0] oled_dbg4;
   logic        oled_txd;

   HEX2OLED_module
     oled(.dbg1(oled_dbg1),
          .dbg2(oled_dbg2),
          .dbg3(oled_dbg3),
          .dbg4(oled_dbg4),
          .txd(oled_txd),
          .clk(clk),
          .rstn(rstn));

   always_comb begin
      oled_dbg1 = {l_target_rot_v[15:0], l_pulse_width[15:0]}; // [左モータ]設定角速度・出力パルス幅
      oled_dbg2 = {r_target_rot_v[15:0], r_pulse_width[15:0]}; // [右モータ]設定角速度・出力パルス幅
      oled_dbg3 = l_rot_cnt; // {16'h0, 16'h0};
      oled_dbg4 = r_rot_cnt; // {16'h0, 16'h0};
   end

   //-----------------------------------------------------------------------------
   // cnt
   logic [31:0] cnt_r;

   always_ff @(posedge clk) begin
      if(!rstn) begin
         cnt_r <= 0;
      end
      else begin
         cnt_r <= cnt_r + 1;
      end
   end

   //-----------------------------------------------------------------------------
   // btn
   logic [1:0] btn_r;

   always_ff @(posedge clk) begin
      if(!rstn) begin
         btn_r <= 0;
      end
      else begin
         if(cnt_r[8]) begin
            btn_r <= btn;
         end
      end
   end

   typedef enum logic {STT_WRITE_EXEC,
                       STT_WRITE_WAIT} btn_write_statetype;

   btn_write_statetype btn_write_state;

   logic [31:0] btn_write_data_r;
   logic [7:0]  btn_write_addr_r;
   logic        btn_write_valid_r;
   logic        btn_write_exec;

   logic btn_write_flag, btn_write_flag_r;
   logic btn_write_rstn;

   general_cnt #(.INTERVAL(BTN_WRITE_INTERVAL))
   btn_write_cnt(.flag(btn_write_flag),
                 .clk(clk),
                 .rstn(btn_write_rstn));

   always_comb begin
      btn_write_rstn  = (btn_write_state  == STT_WRITE_WAIT) ? 1 : 0;

      w_2.data  = btn_write_data_r;
      w_2.addr  = btn_write_addr_r;
      w_2.valid = btn_write_valid_r;

      btn_write_exec  = w_2.valid & w_2.ready;
   end

   always_ff @(posedge clk) begin
      if(!rstn) begin
         btn_write_state   <= STT_WRITE_WAIT;
         btn_write_flag_r  <= 0;
         btn_write_data_r  <= 0;
         btn_write_addr_r  <= 0;
         btn_write_valid_r <= 0;
      end
      else begin
         if(btn_write_flag) btn_write_flag_r <= 1;

         //-----------------------------------------------------------------------------
         // 2 state FSM
         // write実行についての状態遷移
         case(btn_write_state)
           STT_WRITE_EXEC: begin
              btn_write_valid_r <= 1;

              // ハンドシェイク成立
              if(btn_write_exec) begin
                 btn_write_valid_r <= 0;
                 btn_write_state   <= STT_WRITE_WAIT;
              end
           end

           STT_WRITE_WAIT: begin
              if(btn_write_flag_r) begin
                 btn_write_flag_r <= 0;
                 btn_write_data_r <= {30'b0, btn_r};
                 btn_write_addr_r <= ADDR_CMOD_BTN;
                 btn_write_state  <= STT_WRITE_EXEC;
              end
           end
         endcase
      end
   end


   //-----------------------------------------------------------------------------
   // led
   assign led0_r = (l_target_rot_v < r_target_rot_v)            ? 1'b1 : 1'b0;
   assign led0_g = (r_target_rot_v > l_target_rot_v)            ? 1'b1 : 1'b0;
   assign led0_b = (l_target_rot_v == 0 && r_target_rot_v == 0) ? 1'b1 : 1'b0;

   //-----------------------------------------------------------------------------
   // Pmod connection
   assign ja[0] = r_dir;
   assign ja[1] = r_pwm;
   assign ja[2] = r_sa;
   assign ja[3] = r_sb;

   assign ja[4] = ~l_dir;
   assign ja[5] = l_pwm;
   assign ja[6] = l_sb;
   assign ja[7] = l_sa;

   //-----------------------------------------------------------------------------
   // GPIO connection
   assign pio1  = 1'bz;
   assign pio2  = 1'bz;
   assign pio3  = 1'bz;
   assign pio4  = 1'bz;
   assign pio5  = 1'bz;
   assign pio6  = 1'bz;
   assign pio7  = 1'bz;
   assign pio8  = 1'bz;
   assign pio9  = 1'bz;
   assign pio10 = 1'bz;
   assign pio11 = 1'bz;
   assign pio12 = 1'bz;
   assign pio13 = 1'bz;
   assign pio14 = 1'bz;
   assign pio17 = 1'bz;
   assign pio18 = 1'bz;
   assign pio19 = 1'bz;
   assign pio20 = 1'bz;
   assign pio21 = 1'bz;
   assign pio22 = 1'bz;
   assign pio23 = 1'b1; // VUから電源が取れない(なぜ？)のでとりあえずここから取る
   assign pio26 = 1'bz;
   assign pio27 = 1'bz;
   assign pio28 = 1'bz;
   assign pio29 = 1'bz;
   assign pio30 = 1'bz;
   assign pio31 = 1'bz;
   assign pio32 = 1'bz;
   assign pio33 = 1'bz;
   assign pio34 = 1'bz;
   assign pio35 = 1'bz;
   assign pio36 = 1'bz;
   assign pio37 = 1'bz;
   assign pio38 = 1'bz;
   assign pio39 = 1'bz;
   assign pio40 = 1'bz;
   assign pio41 = 1'bz;
   assign pio42 = 1'bz;
   assign pio43 = 1'bz;
   assign pio44 = 1'bz;
   assign pio45 = 1'bz;
   assign pio46 = oled_txd;
   assign pio47 = uart_txd;
   assign pio48 = uart_rxd;

endmodule
