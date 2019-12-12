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

#include "PcamImageCorrector.h"

namespace fad {
    PcamImageCorrector::PcamImageCorrector(const cv::Size&    image_size,
                                           const std::string& cv_calibration_xml_file_path,
                                           const double*      hom_trans_matrix) :
        improc::ImageCorrector(image_size,
                               cv_calibration_xml_file_path,
                               hom_trans_matrix),
        dst_gray_buf_(cv::Mat(image_size, CV_8UC1)),
        dst_sat_buf_(cv::Mat(image_size, CV_8UC1)),
        dst_edge_buf_(cv::Mat(image_size, CV_8UC1)),
        dst_bin_buf_(cv::Mat(image_size, CV_8UC1)) {
    }

    PcamImageCorrector::~PcamImageCorrector() {
    }

    void PcamImageCorrector::execute(const cv::Mat& src,
                                     cv::Mat&       dst_gray,
                                     cv::Mat&       dst_sat,
                                     cv::Mat&       dst_edge,
                                     cv::Mat&       dst_bin,
                                     double         ratio) const {
        if(src.size != map.size) {
            throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                        "src image size is invalid");
        }
        ratio = (1 < ratio) ? 1 : ratio;

        const int HEIGHT       = map.size().height;
        const int WIDTH        = map.size().width;
        const int RD_IM_WIDTH  = WIDTH  * ratio;
        const int RD_IM_HEIGHT = HEIGHT * ratio;

        for(int yi = 0; yi < HEIGHT; yi++) {
            auto map_yi = map.ptr<int>(yi);
            for(int xi = 0; xi < WIDTH; xi++) {
                int src_index = map_yi[xi];
                if(0 <= src_index) {
                    dst_gray_buf_.data[xi + yi * WIDTH] =  src.data[src_index] & PCAM_IMG_GRAY_MASK;
                    dst_sat_buf_.data[xi + yi * WIDTH]  = (src.data[src_index] & PCAM_IMG_SAT_BIT)  ? 0xFF : 0x00;
                    dst_edge_buf_.data[xi + yi * WIDTH] = (src.data[src_index] & PCAM_IMG_EDGE_BIT) ? 0xFF : 0x00;
                    dst_bin_buf_.data[xi + yi * WIDTH]  = (src.data[src_index] & PCAM_IMG_BIN_BIT)  ? 0xFF : 0x00;
                }
                else {
                    dst_gray_buf_.data[xi + yi * WIDTH] = 0x00;
                    dst_sat_buf_.data[xi + yi * WIDTH]  = 0x00;
                    dst_edge_buf_.data[xi + yi * WIDTH] = 0x00;
                    dst_bin_buf_.data[xi + yi * WIDTH]  = 0x00;
                }
            }
        }

        cv::resize(dst_gray_buf_, dst_gray, cv::Size(RD_IM_WIDTH, RD_IM_HEIGHT), 0, 0, cv::INTER_CUBIC);
        cv::resize(dst_sat_buf_,  dst_sat,  cv::Size(RD_IM_WIDTH, RD_IM_HEIGHT), 0, 0, cv::INTER_NEAREST);
        cv::resize(dst_edge_buf_, dst_edge, cv::Size(RD_IM_WIDTH, RD_IM_HEIGHT), 0, 0, cv::INTER_CUBIC);
        cv::resize(dst_bin_buf_,  dst_bin,  cv::Size(RD_IM_WIDTH, RD_IM_HEIGHT), 0, 0, cv::INTER_NEAREST);
    }
}
