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
// module      : モータ制御モジュール
// description : 機能一覧
//               1. PWM波形生成
//               2. PID制御器によるフィードバック制御
//               3. ロータリーエンコーダのカウント
module motor_ctrl
  #(parameter
    CLK_FREQ                   = 100_000_000,
    READ_ROT_VELOCITY_INTERVAL = 100,
    READ_GAIN_INTERVAL         = 1000000,
    WRITE_ROT_CNT_INTERVAL     = 1000000,
    WRITE_ROT_CNT_OFFSET       = 0,
    ADDR_ROTATION_VELOCITY     = 8'h00,
    ADDR_PROPORTION_GAIN       = 8'h00,
    ADDR_INTEGRATION_GAIN      = 8'h00,
    ADDR_DERIVATIVE_GAIN       = 8'h00,
    ADDR_ROT_CNT               = 8'h00,
    localparam
    BAND_WIDTH                 = 32,
    ONE_ROTATION_PULSE         = 630,
    SAMPLING_RATE              = 100,
    PWM_PERIOD                 = 40,  // us
    PWM_PERIOD_CYCLE           = (CLK_FREQ / 100_000) * PWM_PERIOD)
   (w_busif.master                w_m,
    r_busif.master                r_m,
    output logic                  pwm,
    output logic                  dir,
    output logic [BAND_WIDTH-1:0] pulse_width,
    output logic [BAND_WIDTH-1:0] rot_cnt,
    output logic [31:0]           target_rot_v,
    input  logic                  sa,
    input  logic                  sb,
    input  logic                  clk,
    input  logic                  rstn);

   logic [BAND_WIDTH-1:0] pwm_pulse_width;
   logic [BAND_WIDTH-1:0] p_gain;
   logic [BAND_WIDTH-1:0] i_gain;
   logic [BAND_WIDTH-1:0] d_gain;

   write_param_ctrl #(.ADDR_ROT_CNT(ADDR_ROT_CNT),
                      .WRITE_ROT_CNT_INTERVAL(WRITE_ROT_CNT_INTERVAL),
                      .WRITE_ROT_CNT_OFFSET(WRITE_ROT_CNT_OFFSET))
   write_param_ctrl_inst(.w_m(w_m),
                         .rot_cnt(rot_cnt),
                         .clk(clk),
                         .rstn(rstn));

   read_param_ctrl #(.READ_ROT_VELOCITY_INTERVAL(READ_ROT_VELOCITY_INTERVAL),
                     .READ_GAIN_INTERVAL(READ_GAIN_INTERVAL),
                     .ADDR_ROTATION_VELOCITY(ADDR_ROTATION_VELOCITY),
                     .ADDR_PROPORTION_GAIN(ADDR_PROPORTION_GAIN),
                     .ADDR_INTEGRATION_GAIN(ADDR_INTEGRATION_GAIN),
                     .ADDR_DERIVATIVE_GAIN(ADDR_DERIVATIVE_GAIN))
   read_param_ctrl_inst(.r_m(r_m),
                        .rot_v(target_rot_v),
                        .p_gain(p_gain),
                        .i_gain(i_gain),
                        .d_gain(d_gain),
                        .clk(clk),
                        .rstn(rstn));

   pwm_generator #(.BAND_WIDTH(BAND_WIDTH),
                   .PWM_PERIOD_CYCLE(PWM_PERIOD_CYCLE))
   pwm_gen_inst(.width(pwm_pulse_width),
                .pwm(pwm),
                .clk(clk),
                .rstn(rstn));

   rot_encoder #(.BAND_WIDTH(BAND_WIDTH))
   rot_encoder_inst(.sa(sa),
                    .sb(sb),
                    .count(rot_cnt),
                    .clk(clk),
                    .rstn(rstn));

   pid_controller #(.BAND_WIDTH(BAND_WIDTH),
                    .CLK_FREQ(CLK_FREQ),
                    .ONE_ROTATION_PULSE(ONE_ROTATION_PULSE),
                    .SAMPLING_RATE(SAMPLING_RATE))
   pid_ctrl_inst(.target_rot_v(target_rot_v),
                 .p_gain(p_gain),
                 .i_gain(i_gain),
                 .d_gain(d_gain),
                 .rot_cnt(rot_cnt),
                 .pulse_width(pulse_width),
                 .clk(clk),
                 .rstn(rstn));

   assign dir             = pulse_width[31:31] ? 1'b1 : 1'b0;
   assign pwm_pulse_width = pulse_width[31:31] ? ~pulse_width : pulse_width;

endmodule
