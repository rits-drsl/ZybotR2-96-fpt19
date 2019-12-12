/**
 *  PcamImageCorrector: Pcamから取得した画像の前処理として、
 *                      歪曲収差補正・射影変換を行いつつ画像を縮小する
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_HWCONTROLLER_PCAMIMAGECORRECTOR_PCAMIMAGECORRECTOR_H_
#define FAD_SRC_FAD_HWCONTROLLER_PCAMIMAGECORRECTOR_PCAMIMAGECORRECTOR_H_

#include <improc.h>

namespace fad {
    class PcamImageCorrector : public improc::ImageCorrector {
    public:
        PcamImageCorrector(const cv::Size&    image_size,
                           const std::string& cv_calibration_xml_file_path,
                           const double*      hom_trans_matrix);
        ~PcamImageCorrector();

        void execute(const cv::Mat& src,
                     cv::Mat&       dst_gray,
                     cv::Mat&       dst_sat,
                     cv::Mat&       dst_edge,
                     cv::Mat&       dst_bin,
                     double         ratio = 0.0) const;

    private:
        const uint8_t PCAM_IMG_GRAY_MASK = 0b11111000;
        const uint8_t PCAM_IMG_SAT_BIT   = 0b00000100;
        const uint8_t PCAM_IMG_EDGE_BIT  = 0b00000010;
        const uint8_t PCAM_IMG_BIN_BIT   = 0b00000001;

        cv::Mat dst_gray_buf_;
        cv::Mat dst_sat_buf_;
        cv::Mat dst_edge_buf_;
        cv::Mat dst_bin_buf_;
    };
}

#endif /* FAD_SRC_FAD_HWCONTROLLER_PCAMIMAGECORRECTOR_PCAMIMAGECORRECTOR_H_ */
