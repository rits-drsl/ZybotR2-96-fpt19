/**
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#ifndef HLS_IM_PROC_
#define HLS_IM_PROC_

#include <ap_axi_sdata.h>
#include <hls_stream.h>
#include <hls_math.h>

namespace hlsimproc {
    template<int D>
    struct im_axis{
        ap_uint<D> data;
        ap_uint<1> user;
        ap_uint<1> last;
    };

    //--- 勾配方向の定義
    const int DIR_0_   = 0;
    const int DIR_45_  = 1;
    const int DIR_90_  = 2;
    const int DIR_135_ = 3;

    //--- 勾配情報を持った画像データ配列の構造体
    struct vector_image {
        unsigned char value;
        unsigned char grad;
    };

    //--- 座標とSADの値を表す構造体(for template_matching)
    // 20180815 任意精度型を使うと左シフト時に値が失われてしまう
    struct coordinate {
        uint32_t x;
        uint32_t y;
        uint32_t sad;
    };

    //--- 画像処理実行クラス
    template<int WIDTH, int HEIGHT>
    class HlsImProc {
    public:
        // AXI4-Stream形式の画像をGrayScale化してunsigned char配列に格納する
        static void axis2GrayArray(hls::stream<im_axis<24> >& axis_src, unsigned char* dst);
        // AXI4-Stream形式の画像をGrayScale画像，彩度画像に格納する
        static void axis2GraySatArray(hls::stream<im_axis<24> >& axis_src, unsigned char* dst_gray, unsigned char* dst_sat);
        // GrayScale化されたunsigned char配列をAXI4-Stream形式の画像に変換する
        static void grayArray2rgbAXIS(unsigned char* src, hls::stream<ap_axiu<24,1,1,1> >& axis_dst);
        // GrayScale画像と二値化された３つの画像データを統合してAXI4-Stream形式に変換する
        static void preImProcResult2AXIS(unsigned char* gray_src, ap_uint<1>* src1, ap_uint<1>* src2, ap_uint<1>* src3, hls::stream<im_axis<8> >& axis_dst);
        // 画像ストリームを二分割する
        static void duplicate(unsigned char* src, unsigned char* dst1, unsigned char* dst2);
        // 画像ストリームを三分割する
        static void trifurcation(unsigned char* src, unsigned char* dst1, unsigned char* dst2, unsigned char* dst3);
        // 画像ストリームを二値画像として八分割する
        static void eightDivision(unsigned char* src, ap_uint<1>* dst1, ap_uint<1>* dst2, ap_uint<1>* dst3, ap_uint<1>* dst4, ap_uint<1>* dst5, ap_uint<1>* dst6, ap_uint<1>* dst7, ap_uint<1>* dst8);
        // ガウシアンフィルタ（5x5）を実行する
        static void gaussianBlur(unsigned char* src, unsigned char* dst);
        // 二値化処理を行う
        static void binarization(unsigned char* src, ap_uint<1>* dst, unsigned char thr);
        // ソーベルフィルタを実行する
        static void sobel(unsigned char* src, vector_image* dst);
        // non-maximum suppression（非極大抑制）を実行する
        static void nonMaxSuppression(vector_image* src, unsigned char* dst);
        // ヒステリシス閾値化を実行する
        static void hystThreshold(unsigned char* src, unsigned char* dst, unsigned char hthr, unsigned char lthr);
        // ヒステリシス閾値化後の画像に対して近傍ピクセルとの比較演算を実行する
        static void hystThresholdComp(unsigned char* src, ap_uint<1>* dst);
        // 境界ピクセルを0で埋める
        static void borderIgnore(unsigned char* src, unsigned char* dst, int size);
        // テンプレートマッチングを実行する
        template<int PATTERN_SIZE>
        static coordinate templateMatching(ap_uint<1>* src, ap_uint<1> pattern[PATTERN_SIZE][PATTERN_SIZE], uint16_t sad_thr);
    };

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::axis2GrayArray(hls::stream<im_axis<24> >& axis_src, unsigned char* dst) {
        im_axis<24> axis_reader; // AXI4-Streamの読み取り用変数
        bool sof = false;              // Start of Frame
        bool eol = false;              // End of Line

        // user信号のアサートを待つ
        while (!sof) {
            #pragma HLS PIPELINE II=1
            #pragma HLS LOOP_TRIPCOUNT avg=0 max=0
            axis_src >> axis_reader;
            sof = axis_reader.user.to_int();
        }

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            eol = false;
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                // last信号がアサートされるまでpix取得
                if(sof || eol) {
                    // user信号がアサートされているpxは取得済み
                    // WIDTHが実際のフレームサイズ以上に設定された場合
                    sof = false;
                    eol = axis_reader.last.to_int();
                }
                else {
                    axis_src >> axis_reader;
                    eol = axis_reader.last.to_int();
                }

                //--- グレイスケール処理
                int pix_gray;  // 出力画素値

                // Y = B*0.144 + G*0.587 + R*0.299
                // 16ビット左シフトさせた近似値を使用
                pix_gray =  9437* (axis_reader.data & 0x0000ff)
                    + 38469*((axis_reader.data & 0x00ff00) >> 8 )
                    + 19595*((axis_reader.data & 0xff0000) >> 16);

                pix_gray >>= 16;

                // 飽和処理（丸め誤差によるオーバーフロー対策）
                if(pix_gray < 0) {
                    pix_gray = 0;
                }
                else if(pix_gray > 255) {
                    pix_gray = 255;
                }

                // dataを書き込む
                dst[xi + yi*WIDTH] = pix_gray;
            }

            // WIDTHが実際のフレームサイズ以下に設定された場合
            // last信号がアサートするまで読み込む
            while (!eol) {
                #pragma HLS pipeline II=1
                #pragma HLS loop_tripcount avg=0 max=0
                axis_src >> axis_reader;
                eol = axis_reader.last.to_int();
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::axis2GraySatArray(hls::stream<im_axis<24> >& axis_src, unsigned char* dst_gray, unsigned char* dst_sat) {
        im_axis<24> axis_reader; // AXI4-Streamの読み取り用変数
        bool        sof = false; // Start of Frame
        bool        eol = false; // End of Line

        // user信号のアサートを待つ
        while (!sof) {
            #pragma HLS PIPELINE II=1
            #pragma HLS LOOP_TRIPCOUNT avg=0 max=0
            axis_src >> axis_reader;
            sof = axis_reader.user.to_int();
        }

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            eol = false;
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                // last信号がアサートされるまでpix取得
                if(sof || eol) {
                    // user信号がアサートされているpxは取得済み
                    // WIDTHが実際のフレームサイズ以上に設定された場合
                    sof = false;
                    eol = axis_reader.last.to_int();
                }
                else {
                    axis_src >> axis_reader;
                    eol = axis_reader.last.to_int();
                }

                unsigned char pix_r = axis_reader.data & 0x0000ff;
                unsigned char pix_g = (axis_reader.data & 0x00ff00) >> 8;
                unsigned char pix_b = (axis_reader.data & 0xff0000) >> 16;

                //--- グレイスケール処理
                int pix_gray;  // 出力画素値

                // Y = B*0.144 + G*0.587 + R*0.299
                // 16ビット左シフトさせた近似値を使用
                pix_gray = 9437 * pix_r + 38469 * pix_g + 19595 * pix_b;

                pix_gray >>= 16;

                // 飽和処理（丸め誤差によるオーバーフロー対策）
                if(pix_gray < 0) {
                    pix_gray = 0;
                }
                else if(pix_gray > 255) {
                    pix_gray = 255;
                }

                // dataを書き込む
                dst_gray[xi + yi*WIDTH] = pix_gray;

                //--- HSV変換
                unsigned char max = 0;
                if(max < pix_r) {
                    max = pix_r;
                }
                if(max < pix_g) {
                    max = pix_g;
                }
                if(max < pix_b) {
                    max = pix_b;
                }

                unsigned char min = 255;
                if(pix_r < min) {
                    min = pix_r;
                }
                if(pix_g < min) {
                    min = pix_g;
                }
                if(pix_b < min) {
                    min = pix_b;
                }

                // 彩度出力
                dst_sat[xi + yi*WIDTH] = max - min;
            }

            // WIDTHが実際のフレームサイズ以下に設定された場合
            // last信号がアサートするまで読み込む
            while (!eol) {
                #pragma HLS pipeline II=1
                #pragma HLS loop_tripcount avg=0 max=0
                axis_src >> axis_reader;
                eol = axis_reader.last.to_int();
            }
        }

    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::grayArray2rgbAXIS(unsigned char* src, hls::stream<ap_axiu<24,1,1,1> >& axis_dst) {
        ap_axiu<24,1,1,1> axis_writer; // AXI4-Streamの書き込み用変数

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                // dataを書き込む
                int pix_out = src[xi + yi*WIDTH];
                axis_writer.data = pix_out << 16 | pix_out << 8 | pix_out;

                // フレームの先頭でuser信号をアサートする
                if (xi == 0 && yi == 0) {
                    axis_writer.user = 1;
                }
                else {
                    axis_writer.user = 0;
                }
                // 各ラインの末尾でlast信号をアサートする
                if (xi == (WIDTH - 1)) {
                    axis_writer.last = 1;
                }
                else {
                    axis_writer.last = 0;
                }

                // AXI4-Stream出力
                axis_dst << axis_writer;
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::preImProcResult2AXIS(unsigned char* gray_src, ap_uint<1>* src1, ap_uint<1>* src2, ap_uint<1>* src3, hls::stream<im_axis<8> >& axis_dst) {
        im_axis<8> axis_writer; // AXI4-Streamの書き込み用変数

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                // dataを書き込む
                ap_uint<8>  pix_out = 0;

                pix_out |= gray_src[xi + yi*WIDTH] & 0b11111000;
                if(src1[xi + yi*WIDTH] == 0b1) {
                    pix_out |= 0b00000001;
                }
                if(src2[xi + yi*WIDTH] == 0b1) {
                    pix_out |= 0b00000010;
                }
                if(src3[xi + yi*WIDTH] == 0b1) {
                    pix_out |= 0b00000100;
                }

                axis_writer.data = pix_out;

                // フレームの先頭でuser信号をアサートする
                if (xi == 0 && yi == 0) {
                    axis_writer.user = 1;
                }
                else {
                    axis_writer.user = 0;
                }
                // 各ラインの末尾でlast信号をアサートする
                if (xi == (WIDTH - 1)) {
                    axis_writer.last = 1;
                }
                else {
                    axis_writer.last = 0;
                }

                // AXI4-Stream出力
                axis_dst << axis_writer;
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::duplicate(unsigned char* src, unsigned char* dst1, unsigned char* dst2) {

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                dst1[xi + yi*WIDTH] = src[xi + yi*WIDTH];
                dst2[xi + yi*WIDTH] = src[xi + yi*WIDTH];
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::trifurcation(unsigned char* src, unsigned char* dst1, unsigned char* dst2, unsigned char* dst3) {
        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                dst1[xi + yi*WIDTH] = src[xi + yi*WIDTH];
                dst2[xi + yi*WIDTH] = src[xi + yi*WIDTH];
                dst3[xi + yi*WIDTH] = src[xi + yi*WIDTH];
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::eightDivision(unsigned char* src, ap_uint<1>* dst1, ap_uint<1>* dst2, ap_uint<1>* dst3, ap_uint<1>* dst4, ap_uint<1>* dst5, ap_uint<1>* dst6, ap_uint<1>* dst7, ap_uint<1>* dst8) {
        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                dst1[xi + yi*WIDTH] = (src[xi + yi*WIDTH] != 0) ? 1 : 0;
                dst2[xi + yi*WIDTH] = (src[xi + yi*WIDTH] != 0) ? 1 : 0;
                dst3[xi + yi*WIDTH] = (src[xi + yi*WIDTH] != 0) ? 1 : 0;
                dst4[xi + yi*WIDTH] = (src[xi + yi*WIDTH] != 0) ? 1 : 0;
                dst5[xi + yi*WIDTH] = (src[xi + yi*WIDTH] != 0) ? 1 : 0;
                dst6[xi + yi*WIDTH] = (src[xi + yi*WIDTH] != 0) ? 1 : 0;
                dst7[xi + yi*WIDTH] = (src[xi + yi*WIDTH] != 0) ? 1 : 0;
                dst8[xi + yi*WIDTH] = (src[xi + yi*WIDTH] != 0) ? 1 : 0;
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::gaussianBlur(unsigned char* src, unsigned char* dst) {
        const int kernel_size = 5;                              // カーネルサイズ

        unsigned char line_buf[kernel_size][WIDTH];             // ラインバッファ
        unsigned char window_buf[kernel_size][kernel_size];     // ウィンドウバッファ

        //-- 5x5 Gaussian kernel (8bit left shift)
        const int gauss_kernel[kernel_size][kernel_size] = { {1,  4,  6,  4, 1},
                                                             {4, 16, 24, 16, 4},
                                                             {6, 24, 36, 24, 6},
                                                             {4, 16, 24, 16, 4},
                                                             {1,  4,  6,  4, 1} };

        #pragma HLS ARRAY_RESHAPE variable=line_buf complete dim=1
        #pragma HLS ARRAY_PARTITION variable=window_buf complete dim=0
        #pragma HLS ARRAY_PARTITION variable=gauss_kernel complete dim=0

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- ガウシアンフィルタ
                int pix_gauss; // 出力画素値

                //-- ラインバッファ（シフトレジスタ）
                for(int yl = 0; yl < kernel_size - 1; yl++) {
                    line_buf[yl][xi] = line_buf[yl + 1][xi];
                }

                // 入力
                line_buf[kernel_size - 1][xi] = src[xi + yi*WIDTH];

                //-- ウィンドウバッファ（シフトレジスタ）
                for(int yw = 0; yw < kernel_size; yw++) {
                    for(int xw = 0; xw < kernel_size - 1; xw++) {
                        window_buf[yw][xw] = window_buf[yw][xw + 1];
                    }
                }

                // ラインバッファの各列を入力
                for(int yw = 0; yw < kernel_size; yw++) {
                    window_buf[yw][kernel_size - 1] = line_buf[yw][xi];
                }

                //-- 畳み込み演算
                pix_gauss = 0;
                for(int yw = 0; yw < kernel_size; yw++) {
                    for(int xw = 0; xw < kernel_size; xw++) {
                        pix_gauss += window_buf[yw][xw] * gauss_kernel[yw][xw];
                    }
                }

                // 8bit right shift
                pix_gauss >>= 8;

                // 出力
                dst[xi + yi*WIDTH] = pix_gauss;
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::binarization(unsigned char* src, ap_uint<1>* dst, unsigned char thr) {
        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- 二値化処理
                ap_uint<1> pix_bin; // 出力画素値

                if(src[xi + yi*WIDTH] < thr) {
                    pix_bin = 0;
                }
                else {
                    pix_bin = 1;
                }

                // 出力
                dst[xi + yi*WIDTH] = pix_bin;
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::sobel(unsigned char* src, vector_image* dst) {
        const int kernel_size = 3;                              // カーネルサイズ

        unsigned char line_buf[kernel_size][WIDTH];             // ラインバッファ
        unsigned char window_buf[kernel_size][kernel_size];     // ウィンドウバッファ

        //-- 3x3 Horizontal Sobel kernel
        const int h_sobel_kernel[kernel_size][kernel_size] = {  { 1,  0, -1},
                                                                { 2,  0, -2},
                                                                { 1,  0, -1}   };
        //-- 3x3 vertical Sobel kernel
        const int v_sobel_kernel[kernel_size][kernel_size] = {  { 1,  2,  1},
                                                                { 0,  0,  0},
                                                                {-1, -2, -1}   };

        #pragma HLS ARRAY_RESHAPE variable=line_buf complete dim=1
        #pragma HLS ARRAY_PARTITION variable=window_buf complete dim=0
        #pragma HLS ARRAY_PARTITION variable=h_sobel_kernel complete dim=0
        #pragma HLS ARRAY_PARTITION variable=v_sobel_kernel complete dim=0

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- ソーベルフィルタ
                int pix_sobel;  // 出力画素値
                int grad_sobel; // 勾配方向

                //-- ラインバッファ（シフトレジスタ）
                for(int yl = 0; yl < kernel_size - 1; yl++) {
                    line_buf[yl][xi] = line_buf[yl + 1][xi];
                }
                // 入力
                line_buf[kernel_size - 1][xi] = src[xi + yi*WIDTH];

                //-- ウィンドウバッファ（シフトレジスタ）
                for(int yw = 0; yw < kernel_size; yw++) {
                    for(int xw = 0; xw < kernel_size - 1; xw++) {
                        window_buf[yw][xw] = window_buf[yw][xw + 1];
                    }
                }
                // ラインバッファの各列を入力
                for(int yw = 0; yw < kernel_size; yw++) {
                    window_buf[yw][kernel_size - 1] = line_buf[yw][xi];
                }

                //-- 畳み込み演算
                // 20180510 halfにするとノルム算出時にオーバーフローが起こる
                int pix_h_sobel = 0;
                int pix_v_sobel = 0;

                // 水平方向の畳み込み
                for(int yw = 0; yw < kernel_size; yw++) {
                    for(int xw = 0; xw < kernel_size; xw++) {
                        pix_h_sobel += window_buf[yw][xw] * h_sobel_kernel[yw][xw];
                    }
                }

                // 垂直方向の畳み込み
                for(int yw = 0; yw < kernel_size; yw++) {
                    for(int xw = 0; xw < kernel_size; xw++) {
                        pix_v_sobel += window_buf[yw][xw] * v_sobel_kernel[yw][xw];
                    }
                }

                // 出力画素値を算出
                pix_sobel = hls::sqrt(float(pix_h_sobel * pix_h_sobel + pix_v_sobel * pix_v_sobel));

                // 飽和処理
                if(pix_sobel > 255) {
                    pix_sobel = 255;
                }

                // 勾配方向を算出
                int t_int = pix_v_sobel * 256 / pix_h_sobel;

                // 112.5° ~ 157.5° (tan 112.5° ~= -2.4142, tan 157.5° ~= -0.4142)
                if(-618 < t_int && t_int <= -106) {
                    grad_sobel = DIR_135_;
                }
                // -22.5° ~ 22.5° (tan -22.5° ~= -0.4142, tan 22.5° = 0.4142)
                else if(-106 < t_int && t_int <= 106) {
                    grad_sobel = DIR_0_;
                }
                // 22.5° ~ 67.5° (tan 22.5° ~= 0.4142, tan 67.5° = 2.4142)
                else if(106 < t_int && t_int < 618) {
                    grad_sobel = DIR_45_;
                }
                // 67.5° ~ 112.5° (to inf)
                else {
                    grad_sobel = DIR_90_;
                }

                // 出力
                if((kernel_size < xi && xi < WIDTH - kernel_size) &&
                   (kernel_size < yi && yi < HEIGHT - kernel_size)) {
                    dst[xi + yi*WIDTH].value = pix_sobel;
                    dst[xi + yi*WIDTH].grad  = grad_sobel;
                }
                // 境界ピクセルは0を出力する
                else {
                    dst[xi + yi*WIDTH].value = 0;
                    dst[xi + yi*WIDTH].grad  = 0;
                }
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::nonMaxSuppression(vector_image* src, unsigned char* dst) {
        const int window_size = 3;                              // カーネルサイズ

        vector_image line_buf[window_size][WIDTH];              // ラインバッファ
        vector_image window_buf[window_size][window_size];      // ウィンドウバッファ

        #pragma HLS ARRAY_RESHAPE variable=line_buf complete dim=1
        #pragma HLS ARRAY_PARTITION variable=window_buf complete dim=0

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- non-maximum suppression
                unsigned char value_nms;  // 出力画素値
                unsigned char grad_nms;   // 出力画素の勾配方向

                //-- ラインバッファ（シフトレジスタ）
                for(int yl = 0; yl < window_size - 1; yl++) {
                    line_buf[yl][xi] = line_buf[yl + 1][xi];
                }
                // 入力
                line_buf[window_size - 1][xi] = src[xi + yi*WIDTH];

                //-- ウィンドウバッファ（シフトレジスタ）
                for(int yw = 0; yw < window_size; yw++) {
                    for(int xw = 0; xw < window_size - 1; xw++) {
                        window_buf[yw][xw] = window_buf[yw][xw + 1];
                    }
                }
                // ラインバッファの各列を入力
                for(int yw = 0; yw < window_size; yw++) {
                    window_buf[yw][window_size - 1] = line_buf[yw][xi];
                }

                //-- 勾配方向に対する比較演算
                value_nms = window_buf[window_size / 2][window_size / 2].value;
                grad_nms = window_buf[window_size / 2][window_size / 2].grad;
                // grad 0° -> 左, 右
                if(grad_nms == DIR_0_) {
                    if(value_nms < window_buf[window_size / 2][0].value ||
                       value_nms < window_buf[window_size / 2][window_size - 1].value) {
                        value_nms = 0;
                    }
                }
                // grad 45° -> 左上, 右下
                else if(grad_nms == DIR_45_) {
                    if(value_nms < window_buf[0][0].value ||
                       value_nms < window_buf[window_size - 1][window_size - 1].value) {
                        value_nms = 0;
                    }
                }
                // grad 90° -> 上, 下
                else if(grad_nms == DIR_90_) {
                    if(value_nms < window_buf[0][window_size - 1].value ||
                       value_nms < window_buf[window_size - 1][window_size / 2].value) {
                        value_nms = 0;
                    }
                }
                // grad 135° -> 左下, 右上
                else if(grad_nms == DIR_135_) {
                    if(value_nms < window_buf[window_size - 1][0].value ||
                       value_nms < window_buf[0][window_size - 1].value) {
                        value_nms = 0;
                    }
                }

                // 出力
                if((window_size < xi && xi < WIDTH - window_size) &&
                   (window_size < yi && yi < HEIGHT - window_size)) {
                    dst[xi + yi*WIDTH] = value_nms;
                } else {
                    // 境界ピクセルは0を出力する
                    dst[xi + yi*WIDTH] = 0;
                }
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::hystThreshold(unsigned char* src, unsigned char* dst, unsigned char hthr, unsigned char lthr) {
        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- ヒステリシス閾値化
                int pix_hyst;  // 出力画素値

                if(src[xi + yi*WIDTH] < lthr) {
                    pix_hyst = 0;
                }
                else if(src[xi + yi*WIDTH] > hthr) {
                    pix_hyst = 255;
                }
                else {
                    pix_hyst = 1;
                }
                // 出力
                dst[xi + yi*WIDTH] = pix_hyst;
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::hystThresholdComp(unsigned char* src, ap_uint<1>* dst) {
        const int window_size = 3;                              // カーネルサイズ

        unsigned char line_buf[window_size][WIDTH];             // ラインバッファ
        unsigned char window_buf[window_size][window_size];     // ウィンドウバッファ

        #pragma HLS ARRAY_RESHAPE variable=line_buf complete dim=1
        #pragma HLS ARRAY_PARTITION variable=window_buf complete dim=0

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //--- non-maximum suppression
                ap_uint<1> pix_hyst = 0;  // 出力画素値

                //-- ラインバッファ（シフトレジスタ）
                for(int yl = 0; yl < window_size - 1; yl++) {
                    line_buf[yl][xi] = line_buf[yl + 1][xi];
                }
                // 入力
                line_buf[window_size - 1][xi] = src[xi + yi*WIDTH];

                //-- ウィンドウバッファ（シフトレジスタ）
                for(int yw = 0; yw < window_size; yw++) {
                    for(int xw = 0; xw < window_size - 1; xw++) {
                        window_buf[yw][xw] = window_buf[yw][xw + 1];
                    }
                }
                // ラインバッファの各列を入力
                for(int yw = 0; yw < window_size; yw++) {
                    window_buf[yw][window_size - 1] = line_buf[yw][xi];
                }

                //-- 比較演算
                for(int yw = 0; yw < window_size; yw++) {
                    for(int xw = 0; xw < window_size; xw++) {
                        if(window_buf[window_size / 2][window_size / 2] != 0) {
                            if(window_buf[yw][xw] == 255) {
                                pix_hyst = 1;
                            }
                        }
                    }
                }

                // 出力
                dst[xi + yi*WIDTH] = pix_hyst;
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    inline void HlsImProc<WIDTH, HEIGHT>::borderIgnore(unsigned char* src, unsigned char* dst, int size) {

        // 画像処理ループ
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                // 出力
                unsigned char pix = src[xi + yi*WIDTH];
                if((size < xi && xi < WIDTH - size) &&
                   (size < yi && yi < HEIGHT - size)) {
                    dst[xi + yi*WIDTH] = pix;
                }
                else {
                    dst[xi + yi*WIDTH] = 0;
                }
            }
        }
    }

    template<int WIDTH, int HEIGHT>
    template<int PATTERN_SIZE>
    inline coordinate HlsImProc<WIDTH, HEIGHT>::templateMatching(ap_uint<1>* src, ap_uint<1> pattern[PATTERN_SIZE][PATTERN_SIZE], uint16_t sad_thr) {
        ap_uint<1> line_buf[PATTERN_SIZE][WIDTH];
        ap_uint<1> window_buf[PATTERN_SIZE][PATTERN_SIZE];

        #pragma HLS ARRAY_RESHAPE variable=line_buf complete dim=1
        #pragma HLS ARRAY_PARTITION variable=window_buf complete dim=0

        // 画像処理ループ
        coordinate result;
        result.x = 0;
        result.y = 0;
        result.sad = PATTERN_SIZE*PATTERN_SIZE;
        for(int yi = 0; yi < HEIGHT; yi++) {
            for(int xi = 0; xi < WIDTH; xi++) {
                #pragma HLS PIPELINE II=1
                #pragma HLS LOOP_FLATTEN off

                //-- ラインバッファ（シフトレジスタ）
                for(int yl = 0; yl < PATTERN_SIZE - 1; yl++) {
                    line_buf[yl][xi] = line_buf[yl + 1][xi];
                }
                // 入力
                line_buf[PATTERN_SIZE - 1][xi] = src[xi + yi*WIDTH];

                //-- ウィンドウバッファ（シフトレジスタ）
                for(int yw = 0; yw < PATTERN_SIZE; yw++) {
                    for(int xw = 0; xw < PATTERN_SIZE - 1; xw++) {
                        window_buf[yw][xw] = window_buf[yw][xw + 1];
                    }
                }
                // ラインバッファの各列を入力
                for(int yw = 0; yw < PATTERN_SIZE; yw++) {
                    window_buf[yw][PATTERN_SIZE - 1] = line_buf[yw][xi];
                }

                //-- 二値化画像に対するsad（xor）
                if(PATTERN_SIZE - 1 <= xi && PATTERN_SIZE - 1 <= yi) {
                    uint32_t sad = 0;
                    for(int yw = 0; yw < PATTERN_SIZE; yw++) {
                        for(int xw = 0; xw < PATTERN_SIZE; xw++) {
                            sad += window_buf[yw][xw] ^ pattern[yw][xw];
                        }
                    }

                    if(sad < result.sad && sad < sad_thr) {
                        result.x = xi;
                        result.y = yi;
                        result.sad  = sad;
                    }
                }
            }
        }
        return result;
    }
}

#endif // HLS_IM_PROC_
