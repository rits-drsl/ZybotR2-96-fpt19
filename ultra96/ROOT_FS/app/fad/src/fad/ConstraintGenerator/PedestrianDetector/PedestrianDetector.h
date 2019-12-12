/**
 *  PedestrianDetector: 歩行者の認識を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_CONSTRAINTGENERATOR_PEDESTRIANDETECTOR_PEDESTRIANDETECTOR_H_
#define FAD_SRC_FAD_CONSTRAINTGENERATOR_PEDESTRIANDETECTOR_PEDESTRIANDETECTOR_H_

#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <YAMLHelper.hpp>
#include <TrafficLightStatus.hpp>
#include <Util.hpp>
#include <PedestrianStatus.hpp>

#include <improc.h>

namespace fad {
    class PedestrianDetector {
    public:
        PedestrianDetector();
        ~PedestrianDetector();

        PedestrianDetector(const PedestrianDetector& obj) = delete;
        PedestrianDetector &operator=(const PedestrianDetector& PedestrianDetector) = delete;

        /**
         *  @brief 認識の結果を取得する
         */
        const core::PedestrianStatus& getResult() const;

        /**
         *  @brief 歩行者認識を実行する
         *  @param (color_img) Webカメラから取得した前方画像
         *  @return 認識された場合trueを返す
         */
        bool detect(const cv::Mat& color_img);

    private:
        const std::string PARAM_YAML_PATH = "/data/ConstraintGenerator/PedestrianDetector/param.yaml";

        class PedestrianParam : public core::YAMLHelper::ParamBase {
        public:
            std::string pedestrian_haar_like_cascade_file_path;
            double      haar_like_scale_factor;
            uint32_t    haar_like_min_neighbors;
            uint32_t    y_of_top_of_detection_area;
            uint32_t    y_of_bottom_of_detection_area;
            uint32_t    x_of_left_of_detection_area;
            uint32_t    x_of_right_of_detection_area;
            uint32_t    gaussian_kernel_size;
            uint32_t    nof_closing;
            uint8_t     hsv_trans_min_h;
            uint8_t     hsv_trans_min_s;
            uint8_t     hsv_trans_min_v;
            uint8_t     hsv_trans_max_h;
            uint8_t     hsv_trans_max_s;
            uint8_t     hsv_trans_max_v;
            double      ratio_of_labeled_region_lower;
            double      ratio_of_labeled_region_upper;
            double      aspect_ratio_lower;
            double      aspect_ratio_upper;

            void read(const cv::FileNode& node) override {
                pedestrian_haar_like_cascade_file_path = (std::string)node["pedestrian_haar_like_cascade_file_path"];
                haar_like_scale_factor                 = (double)node["haar_like_scale_factor"];
                haar_like_min_neighbors                = (int)node["haar_like_min_neighbors"];
                y_of_top_of_detection_area             = (int)node["y_of_top_of_detection_area"];
                y_of_bottom_of_detection_area          = (int)node["y_of_bottom_of_detection_area"];
                x_of_left_of_detection_area            = (int)node["x_of_left_of_detection_area"];
                x_of_right_of_detection_area           = (int)node["x_of_right_of_detection_area"];
                gaussian_kernel_size                   = (int)node["gaussian_kernel_size"];
                nof_closing                            = (int)node["nof_closing"];
                hsv_trans_min_h                        = (int)node["hsv_trans_min_h"];
                hsv_trans_min_s                        = (int)node["hsv_trans_min_s"];
                hsv_trans_min_v                        = (int)node["hsv_trans_min_v"];
                hsv_trans_max_h                        = (int)node["hsv_trans_max_h"];
                hsv_trans_max_s                        = (int)node["hsv_trans_max_s"];
                hsv_trans_max_v                        = (int)node["hsv_trans_max_v"];
                ratio_of_labeled_region_lower          = (double)node["ratio_of_labeled_region_lower"];
                ratio_of_labeled_region_upper          = (double)node["ratio_of_labeled_region_upper"];
                aspect_ratio_lower                     = (double)node["aspect_ratio_lower"];
                aspect_ratio_upper                     = (double)node["aspect_ratio_upper"];
            }
        };

        PedestrianParam param_;
        cv::CascadeClassifier pedestrian_classifier_;
        core::PedestrianStatus result_;
    };
}

#endif /* FAD_SRC_FAD_CONSTRAINTGENERATOR_PEDESTRIANDETECTOR_PEDESTRIANDETECTOR_H_ */
