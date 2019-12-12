/**
 *  ImageCorrector: 歪曲収差補正・射影変換を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef IMPROC_INCLUDE_IMAGECORRECTOR_IMAGECORRECTOR_H_
#define IMPROC_INCLUDE_IMAGECORRECTOR_IMAGECORRECTOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace improc {
    class ImageCorrector {
    public:
        cv::Mat map; // 変換先のindexを保持しているmap

        /**
         *  歪曲収差補正・射影変換を行うクラス
         *  @image_size: 画像サイズ
         *  @cv_calibration_xml_file_path: OpenCVでチェッカーボードを使った補正により生成することの出来る
         *                                 歪曲収差補正式のパラメータを持ったXMLファイルのパス
         *  @hom_trans_matrix: 射影変換行列(option)
         */
        ImageCorrector(const cv::Size&    image_size,
                       const std::string& cv_calibration_xml_file_path,
                       const double*      hom_trans_matrix = nullptr);
        ~ImageCorrector();

        ImageCorrector(const ImageCorrector& obj) = delete;
        ImageCorrector &operator=(const ImageCorrector& obj) = delete;

        /**
         *  保持しているmapを利用して座標変換を実行する
         *  @src: 入力画像
         *  @dst: 出力画像
         */
        void execute(const cv::Mat& src, cv::Mat& dst) const;

        /**
         *  射影変換を実行する
         *  @src: 入力画像
         *  @dst: 出力画像
         *  @H:   射影変換行列
         */
        void homography(const cv::Mat& src, cv::Mat& dst, const double* H) const;
    };
}

#endif /* IMPROC_INCLUDE_IMAGECORRECTOR_IMAGECORRECTOR_H */
