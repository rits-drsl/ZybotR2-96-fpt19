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

#include <ImageCorrector/ImageCorrector.h>

namespace improc {
    ImageCorrector::ImageCorrector(const cv::Size&    image_size,
                                   const std::string& cv_calibration_xml_file_path,
                                   const double*      hom_trans_matrix) {
        // mapの初期化
        map = cv::Mat(image_size, CV_32SC1);

        for(int yi = 0; yi < image_size.height; yi++) {
            int* map_yi = map.ptr<int>(yi);
            for(int xi = 0; xi < image_size.width; xi++) {
                map_yi[xi] = xi + yi * image_size.width;
            }
        }

        cv::FileStorage fs(cv_calibration_xml_file_path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "File can not be opened.");
        }
        else {
            cv::Mat cameraMatrix;
            cv::Mat distCoeffs;
            fs["cameraMatrix"] >> cameraMatrix;
            fs["distCoeffs"]   >> distCoeffs;

            cv::Mat x_map, y_map;
            initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, image_size, CV_32FC1, x_map, y_map);
            cv::remap(map, map, x_map, y_map, cv::INTER_NEAREST, -1);

            if(hom_trans_matrix != nullptr) {
                cv::Mat tmp_map = map.clone();
                homography(tmp_map, map, hom_trans_matrix);
            }
        }
        fs.release();
    }

    ImageCorrector::~ImageCorrector() {
    }

    void ImageCorrector::execute(const cv::Mat& src, cv::Mat& dst) const {
        const int HEIGHT  = dst.size().height;
        const int WIDTH   = dst.size().width;
        const int CHANNEL = dst.channels();

        for(int yi = 0; yi < HEIGHT; yi++) {
            auto map_yi = map.ptr<int>(yi);
            for(int xi = 0; xi < WIDTH; xi++) {
                for(int ci = 0; ci < CHANNEL; ci++) {
                    int dst_index = (xi + yi * WIDTH) * CHANNEL + ci;

                    if(map_yi[xi] != -1) {
                        int src_index = map_yi[xi] * CHANNEL + ci;
                        dst.data[dst_index] = src.data[src_index];
                    }
                    else {
                        dst.data[dst_index] = 0;
                    }
                }
            }
        }
    }

    void ImageCorrector::homography(const cv::Mat& src, cv::Mat& dst, const double* H) const {
        const int HEIGHT = dst.size().height;
        const int WIDTH  = dst.size().width;

        for(int yi = 0; yi < HEIGHT; yi++) {
            auto dst_yi = dst.ptr<int>(yi);
            for(int xi = 0; xi < WIDTH; xi++) {

                // 射影変換
                // |u|   |h00 h01 h02||x|
                // |v| = |h10 h11 h12||y|
                // |1|   |h20 h21 h22||1|
                double denominator = H[6]*xi + H[7]*yi + H[8];
                int xh = std::round((H[0]*xi + H[1]*yi + H[2]) / denominator);
                int yh = std::round((H[3]*xi + H[4]*yi + H[5]) / denominator);

                if(0 <= xh && xh < WIDTH && 0 <= yh && yh < HEIGHT) {
                    dst_yi[xi] = src.ptr<int>(yh)[xh];
                }
                else {
                    dst_yi[xi] = -1;
                }
            }
        }
    }
}
