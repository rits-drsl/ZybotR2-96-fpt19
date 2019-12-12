/**
 *  LabelingExecutor: ラベリングを実行するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *
 */

#ifndef IMPROC_INCLUDE_LABELINGEXECUTOR_LABELINGEXECUTOR_H_
#define IMPROC_INCLUDE_LABELINGEXECUTOR_LABELINGEXECUTOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace improc {
    class LabelingExecutor {
    public:
        // 2つのラベル番号が対応していることを表すLUT
        // (smaller label number, bigger label number) -> has referenced
        using pair_lut   = std::map<std::tuple<uint16_t, uint16_t>, bool>;
        using pair_lut_e = enum { SMALLER_LABEL, BIGGER_LABEL };

        struct Info {
            cv::Point begin;      // 領域の左上の座標
            cv::Point center;     // 領域の中心の座標
            cv::Point end;        // 領域の右下の座標
            uint16_t  area;       // 面積
            uint16_t  cog;        // 重心
            uint8_t   brightness; // 平均画素値
        };

        LabelingExecutor()  = delete;
        ~LabelingExecutor() = delete;

        /**
         *  ラベリングを実行する
         *  @src            : 入力画像
         *  @dst            : 出力画像(二値化した後、それぞれの領域にラベル番号が書き込まれている)
         *  @label_info_map : 各ラベル領域の情報
         *  @bin_thr        : 二値化する際のしきい値
         */
        static std::map<uint16_t, Info> execute(const cv::Mat& src,
                                                const uint8_t& bin_thr = 127);

        static std::map<uint16_t, Info> execute(const cv::Mat& src,
                                                cv::Mat&       dst,
                                                const uint8_t& bin_thr = 127);

        static void execute(const cv::Mat&            src,
                            cv::Mat&                  dst,
                            std::map<uint16_t, Info>& label_info_map,
                            const uint8_t&            bin_thr = 127);

    private:
        // ラベリングにおいて、ラベルの対応関係を解決する再帰関数
        static void solveLUT(pair_lut&              lut_1,
                             std::vector<uint16_t>& lut_2,
                             uint16_t               key,
                             uint16_t               value);
    };
}

#endif /* IMPROC_INCLUDE_LABELINGEXECUTOR_LABELINGEXECUTOR_H_ */
