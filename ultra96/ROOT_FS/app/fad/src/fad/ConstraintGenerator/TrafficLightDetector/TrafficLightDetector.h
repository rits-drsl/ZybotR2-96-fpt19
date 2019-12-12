/**
 *  TrafficLightDetector: 信号機の認識を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_CONSTRAINTGENERATOR_TRAFFICLIGHTDETECTOR_TRAFFICLIGHTDETECTOR_H
#define FAD_SRC_FAD_CONSTRAINTGENERATOR_TRAFFICLIGHTDETECTOR_TRAFFICLIGHTDETECTOR_H

#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <YAMLHelper.hpp>
#include <TrafficLightStatus.hpp>

namespace fad {
    class TrafficLightDetector {
    public:
        struct TLDetectInfo {
            enum class Type { NONE, RED };
            using scs = std::chrono::system_clock;
            cv::Rect  rect;        // 入力画像中の検出位置
            Type      type;        // 検出結果
            double    val;         // テンプレートマッチングの出力値
            scs::time_point time;  // 検出時間
            TLDetectInfo(const cv::Rect&         _rect = cv::Rect(),
                         const Type&             _type = Type::NONE,
                         const double&           _val  = 0.0,
                         const scs::time_point&  _time = scs::now()) :
                rect(_rect), type(_type), val(_val), time(_time) { }
        };

        TrafficLightDetector();
        ~TrafficLightDetector();

        TrafficLightDetector(const TrafficLightDetector& obj) = delete;
        TrafficLightDetector &operator=(const TrafficLightDetector& TrafficLightDetector) = delete;

        /**
         *  @brief 信号認識の結果を取得する
         */
        const TLDetectInfo& getResult() const;

        /**
         *  @brief 信号認識を実行する
         *  @param (color_img)      Webカメラから取得した前方画像
         *  @param (distance_to_tl) 現状態から信号までの距離
         *  @return 画像中に赤信号が認識された場合のみtrueを返す
         */
        bool detect(const cv::Mat& color_img,
                    const double&  distance_to_tl = -1);

    private:
        const std::string PARAM_YAML_PATH = "/data/ConstraintGenerator/TrafficLightDetector/param.yaml";

        class TrafficLightParam : public core::YAMLHelper::ParamBase {
        public:
            int         opencv_tm_method;
            std::string red_tl_haar_like_cascade_file_path;
            std::string red_template_file_path;
            uint32_t    y_of_top_of_detection_area;
            uint32_t    y_of_bottom_of_detection_area;
            uint32_t    x_of_left_of_detection_area;
            uint32_t    x_of_right_of_detection_area;
            double      tm_max_val_thr;
            double      region_width_ratio_with_tm;
            double      region_height_ratio_with_tm;
            double      haar_like_scale_factor;
            uint32_t    haar_like_min_neighbors;

            void read(const cv::FileNode& node) override {
                opencv_tm_method                   = (int)node["opencv_tm_method"];
                red_template_file_path             = (std::string)node["red_template_file_path"];
                red_tl_haar_like_cascade_file_path = (std::string)node["red_tl_haar_like_cascade_file_path"];
                y_of_top_of_detection_area         = (int)node["y_of_top_of_detection_area"];
                y_of_bottom_of_detection_area      = (int)node["y_of_bottom_of_detection_area"];
                x_of_left_of_detection_area        = (int)node["x_of_left_of_detection_area"];
                x_of_right_of_detection_area       = (int)node["x_of_right_of_detection_area"];
                tm_max_val_thr                     = (double)node["tm_max_val_thr"];
                region_width_ratio_with_tm         = (double)node["region_width_ratio_with_tm"];
                region_height_ratio_with_tm        = (double)node["region_height_ratio_with_tm"];
                haar_like_scale_factor             = (double)node["haar_like_scale_factor"];
                haar_like_min_neighbors            = (int)node["haar_like_min_neighbors"];
            }
        };

        class TemplateParam : public core::YAMLHelper::ParamBase {
        public:
            double      distance;
            std::string file_path;

            void read(const cv::FileNode& node) override {
                distance  = (double)node["distance"];
                file_path = (std::string)node["file_path"];
            }
        };

        TrafficLightParam tl_param_;
        std::vector<std::pair<double, cv::Mat>> red_templates_; // first: 距離, second: テンプレート画像
        cv::CascadeClassifier red_tl_classifier_;
        TLDetectInfo result_;
    };
}

#endif /* FAD_SRC_FAD_CONSTRAINTGENERATOR_TRAFFICLIGHTDETECTOR_TRAFFICLIGHTDETECTOR_H */
