/**
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#include <hls_stream.h>
#include "HlsImProc.hpp"

using namespace hls;
using namespace hlsimproc;

typedef im_axis<24> rgb_image;
typedef im_axis<8>  gray_image;

#define MAX_WIDTH  1280
#define MAX_HEIGHT 720

uint8_t      sat_fifo[MAX_WIDTH * MAX_HEIGHT];
ap_uint<1>   sat_bin_fifo[MAX_WIDTH * MAX_HEIGHT];
uint8_t      gray_fifo1[MAX_WIDTH * MAX_HEIGHT];
uint8_t      gray_fifo2[MAX_WIDTH * MAX_HEIGHT];
uint8_t      gray_fifo3[MAX_WIDTH * MAX_HEIGHT];
uint8_t      bin_fifo1[MAX_WIDTH * MAX_HEIGHT];
ap_uint<1>   bin_fifo2[MAX_WIDTH * MAX_HEIGHT];
uint8_t      edge_fifo1[MAX_WIDTH * MAX_HEIGHT];
vector_image edge_fifo2[MAX_WIDTH * MAX_HEIGHT];
uint8_t      edge_fifo3[MAX_WIDTH * MAX_HEIGHT];
uint8_t      edge_fifo4[MAX_WIDTH * MAX_HEIGHT];
uint8_t      edge_fifo5[MAX_WIDTH * MAX_HEIGHT];
ap_uint<1>   edge_fifo6[MAX_WIDTH * MAX_HEIGHT];

// Top Function
void preimproc(stream<rgb_image>&  axis_in,
               stream<gray_image>& axis_out,
               uint8_t&            sat_bin_thr,
               uint8_t&            gray_bin_thr,
               uint8_t&            hist_hthr,
               uint8_t&            hist_lthr) {
    #pragma HLS INTERFACE axis         port=axis_in
    #pragma HLS INTERFACE axis         port=axis_out
    #pragma HLS INTERFACE s_axilite    port=gray_bin_thr bundle=param clock=s_axi_aclk
    #pragma HLS INTERFACE s_axilite    port=sat_bin_thr  bundle=param clock=s_axi_aclk
    #pragma HLS INTERFACE s_axilite    port=hist_hthr    bundle=param clock=s_axi_aclk
    #pragma HLS INTERFACE s_axilite    port=hist_lthr    bundle=param clock=s_axi_aclk
    #pragma HLS INTERFACE ap_ctrl_none port=return

    #pragma HLS DATAFLOW

    #pragma HLS STREAM variable=sat_fifo     depth=1     dim=1
    #pragma HLS STREAM variable=sat_bin_fifo depth=10000 dim=1
    #pragma HLS STREAM variable=gray_fifo1   depth=1     dim=1
    #pragma HLS STREAM variable=gray_fifo2   depth=1     dim=1
    #pragma HLS STREAM variable=gray_fifo3   depth=10000 dim=1
    #pragma HLS STREAM variable=bin_fifo1    depth=1     dim=1
    #pragma HLS STREAM variable=bin_fifo2    depth=10000 dim=1
    #pragma HLS STREAM variable=edge_fifo1   depth=1     dim=1
    #pragma HLS STREAM variable=edge_fifo2   depth=1     dim=1
    #pragma HLS STREAM variable=edge_fifo3   depth=1     dim=1
    #pragma HLS STREAM variable=edge_fifo4   depth=1     dim=1
    #pragma HLS STREAM variable=edge_fifo5   depth=1     dim=1
    #pragma HLS STREAM variable=edge_fifo6   depth=1     dim=1

    HlsImProc<MAX_WIDTH, MAX_HEIGHT>::axis2GraySatArray(axis_in, gray_fifo1, sat_fifo);

    HlsImProc<MAX_WIDTH, MAX_HEIGHT>::gaussianBlur(gray_fifo1, gray_fifo2);

    // 画像ストリームを三分割
    HlsImProc<MAX_WIDTH, MAX_HEIGHT>::trifurcation(gray_fifo2, gray_fifo3, edge_fifo1, bin_fifo1);

    //-- 二値化（grayscale）
    HlsImProc<MAX_WIDTH, MAX_HEIGHT>::binarization(bin_fifo1, bin_fifo2, gray_bin_thr);

    //-- 二値化（saturation）
    HlsImProc<MAX_WIDTH, MAX_HEIGHT>::binarization(sat_fifo, sat_bin_fifo, sat_bin_thr);

    //-- Cannyエッジ検出
    // ソーベルフィルタ
    HlsImProc<MAX_WIDTH, MAX_HEIGHT>::sobel(edge_fifo1, edge_fifo2);

    // non-maximum suppression
    HlsImProc<MAX_WIDTH, MAX_HEIGHT>::nonMaxSuppression(edge_fifo2, edge_fifo3);

    // ヒステリシス閾値化
    HlsImProc<MAX_WIDTH, MAX_HEIGHT>::hystThreshold(edge_fifo3, edge_fifo4, hist_hthr, hist_lthr);

    // 境界ピクセルを0で埋める
    int border_size = 10;
    HlsImProc<MAX_WIDTH, MAX_HEIGHT>::borderIgnore(edge_fifo4, edge_fifo5, border_size);

    // 近傍ピクセルとの比較演算
    HlsImProc<MAX_WIDTH, MAX_HEIGHT>::hystThresholdComp(edge_fifo5, edge_fifo6);

    // グレイスケール画像, 彩度画像, 二値化画像, エッジ画像をAXI4-Streamに変換
    HlsImProc<MAX_WIDTH, MAX_HEIGHT>::preImProcResult2AXIS(gray_fifo3, bin_fifo2, edge_fifo6, sat_bin_fifo, axis_out);
}
