/**
 *  RoadSurfaceVOCalculator: 路面の俯瞰画像からVisualOdometryを導出するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_ODOMETRYCALCULATOR_ROADSURFACEVOCALCULATOR_ROADSURFACEVOCALCULATOR_H_
#define FAD_SRC_FAD_ODOMETRYCALCULATOR_ROADSURFACEVOCALCULATOR_ROADSURFACEVOCALCULATOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <YAMLHelper.hpp>
#include <VehicleState.hpp>
#include <VisualOdometry.hpp>

#include <improc.h>

namespace fad {
    class RoadSurfaceVOCalculator {
    public:
        RoadSurfaceVOCalculator(const cv::Mat& init_frame);
        ~RoadSurfaceVOCalculator();

        RoadSurfaceVOCalculator(const RoadSurfaceVOCalculator& obj) = delete;
        RoadSurfaceVOCalculator &operator=(const RoadSurfaceVOCalculator& obj) = delete;

        RoadSurfaceVOCalculator(RoadSurfaceVOCalculator&&) = default;

        void init(const cv::Mat& init_frame);

        const core::VisualOdometry& get() const;

        bool run(const cv::Mat& current_frame);

    private:
        const std::string RSVOC_PARAM_YAML_PATH           = "/data/OdometryCalculator/RoadSurfaceVOCalculator/param.yaml";
        const std::string PATTERN_MATCHER_PARAM_YAML_PATH = "/data/OdometryCalculator/RoadSurfaceVOCalculator/pattern_matcher.yaml";

        class RSVOCParam : public core::YAMLHelper::ParamBase {
        public:
            double pattern_matcher_delta_threshord;
            void read(const cv::FileNode& node) override {
                pattern_matcher_delta_threshord = (double)node["pattern_matcher_delta_threshord"];
            }
        };

        class AKAZEParam : public core::YAMLHelper::ParamBase {
        public:
            int   descriptor_type;
            int   descriptor_size;
            int   descriptor_channels;
            float threshold;
            int   nOctaves;
            int   nOctaveLayers;
            int   diffusivity;

            void read(const cv::FileNode& node) override {
                descriptor_type     = (int)node["descriptor_type"];
                descriptor_size     = (int)node["descriptor_size"];
                descriptor_channels = (int)node["descriptor_channels"];
                threshold           = (float)node["threshold"];
                nOctaves            = (int)node["nOctaves"];
                nOctaveLayers       = (int)node["nOctaveLayers"];
                diffusivity         = (int)node["diffusivity"];
            }
        };

        class MatcherParam : public core::YAMLHelper::ParamBase {
        public:
            std::string DescriptorMatcherType;
            double      NNDR;
            float       matching_dist_thr;

            void read(const cv::FileNode& node) override {
                DescriptorMatcherType = (std::string)node["DescriptorMatcherType"];
                NNDR                  = (double)node["NNDR"];
                matching_dist_thr     = (float)node["matching_dist_thr"];
            }
        };

        core::VisualOdometry                     vo_;
        improc::AKAZEPatternMatcher::FeatureInfo prev_feature_;

        RSVOCParam param_;

        std::unique_ptr<improc::AKAZEPatternMatcher> pattern_matcher_;
    };
}

#endif /* FAD_SRC_FAD_ODOMETRYCALCULATOR_ROADSURFACEVOCALCULATOR_ROADSURFACEVOCALCULATOR_H_ */

