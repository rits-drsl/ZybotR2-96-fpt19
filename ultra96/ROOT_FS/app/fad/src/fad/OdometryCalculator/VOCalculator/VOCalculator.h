/**
 *  VOCalculator: VisualOdometryを導出するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_ODOMETRYCALCULATOR_VOCALCULATOR_VOCALCULATOR_H_
#define FAD_SRC_FAD_ODOMETRYCALCULATOR_VOCALCULATOR_VOCALCULATOR_H_

#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <VehicleState.hpp>
#include <VisualOdometry.hpp>
#include <YAMLHelper.hpp>

namespace fad {
    class VOCalculator {
    public:
        VOCalculator(const cv::Mat&            init_frame,
                     const core::VehicleState& init_state);
        ~VOCalculator();

        VOCalculator(const VOCalculator& obj) = delete;
        VOCalculator &operator=(const VOCalculator& obj) = delete;

        VOCalculator(VOCalculator&&) = default;

        void init(const cv::Mat&            init_frame,
                  const core::VehicleState& init_state);

        const core::VisualOdometry& get() const;

        bool run(const cv::Mat&            current_frame,
                 const core::VehicleState& current_state);

    private:
        const std::string VO_PARAM_YAML_PATH = "/data/OdometryCalculator/VOCalculator/param.yaml";

        class VOParam : public core::YAMLHelper::ParamBase {
        public:
            uint32_t feature_pt_min_num_feat;
            uint32_t vo_queue_depth;
            double   vo_dtheta_valid_norm_threshold;

            void read(const cv::FileNode& node) override {
                feature_pt_min_num_feat        = (int)node["feature_pt_min_num_feat"];
                vo_queue_depth                 = (int)node["vo_queue_depth"];
                vo_dtheta_valid_norm_threshold = (double)node["vo_dtheta_valid_norm_threshold"];
            }
        };

        class FastParam : public core::YAMLHelper::ParamBase {
        public:
            bool     use_nms;
            uint32_t threshold;

            void read(const cv::FileNode& node) override {
                use_nms   = (int)node["use_nms"];
                threshold = (int)node["threshold"];
            }
        };

        class OpticalFlowParam : public core::YAMLHelper::ParamBase {
        public:
            uint32_t win_size;
            uint32_t tarm_criteria_max_count;
            double   tarm_criteria_epsilon;
            uint32_t max_level;
            double   min_eig_threshold;

            void read(const cv::FileNode& node) override {
                win_size                = (int)node["win_size"];
                tarm_criteria_max_count = (int)node["tarm_criteria_max_count"];
                tarm_criteria_epsilon   = (double)node["tarm_criteria_epsilon"];
                max_level               = (int)node["max_level"];
                min_eig_threshold       = (double)node["min_eig_threshold"];
            }
        };

        class FivePtParam : public core::YAMLHelper::ParamBase {
        public:
            double focal;
            double pp_x;
            double pp_y;
            double prob;
            double threshold;

            void read(const cv::FileNode& node) override {
                focal     = (double)node["focal"];
                pp_x      = (double)node["pp_x"];
                pp_y      = (double)node["pp_y"];
                prob      = (double)node["prob"];
                threshold = (double)node["threshold"];
            }
        };

        VOParam          vo_param_;
        FastParam        fast_param_;
        FivePtParam      five_pt_param_;
        OpticalFlowParam optical_flow_param_;

        core::VisualOdometry              vo_;
        std::vector<core::VisualOdometry> vo_q_;
        size_t                            vo_q_front_index_;

        cv::Mat R_;

        cv::Mat prev_frame_;
        cv::Mat gray_frame_;

        core::VehicleState       prev_state_;
        std::vector<cv::Point2f> prev_feature_pt_;
        double                   prev_theta_;

        void trackFeaturePt(const cv::Mat&            prev_img,
                            const cv::Mat&            next_img,
                            std::vector<cv::Point2f>& prev_pt,
                            std::vector<cv::Point2f>& next_pt) const;

        void detectFeaturePt(const cv::Mat&            prev_img,
                             std::vector<cv::Point2f>& prev_pt) const;
    };
}

#endif /* FAD_SRC_FAD_ODOMETRYCALCULATOR_VOCALCULATOR_VOCALCULATOR_H_ */

