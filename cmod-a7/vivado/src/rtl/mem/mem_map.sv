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
// RAM同期システムのアドレス定義

//--- Ultra96 -> CMod-A7
// 目標角速度
localparam ADDR_R_ROTATION_VELOCITY  = 8'h00;
localparam ADDR_L_ROTATION_VELOCITY  = 8'h01;

// PID制御の比例ゲイン(8ビット左シフト)
localparam ADDR_R_PROPORTION_GAIN    = 8'h02;
localparam ADDR_L_PROPORTION_GAIN    = 8'h03;

// PID制御の積分ゲイン(8ビット左シフト)
localparam ADDR_R_INTEGRATION_GAIN   = 8'h04;
localparam ADDR_L_INTEGRATION_GAIN   = 8'h05;

// PID制御の微分ゲイン(8ビット左シフト)
localparam ADDR_R_DERIVATIVE_GAIN    = 8'h06;
localparam ADDR_L_DERIVATIVE_GAIN    = 8'h07;

//--- CMod-A7 -> Ultra96
// ロータリーエンコーダのカウンタ値
localparam ADDR_R_ROT_CNT            = 8'h80;
localparam ADDR_L_ROT_CNT            = 8'h81;

// Cmod-A7bに搭載されているボタンの状態
localparam ADDR_CMOD_BTN             = 8'h90;
