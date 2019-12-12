/**
 *  AKAZEPatternMatcher: 路面画像と地図画像のマッチングを
 *                       AKAZE特徴量マッチングによって行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_PATTERNMATCHINGVOCALCULATOR_PATTERNMATCHER_AKAZEPATTERNMATCHER_AKAZEPATTERNMATCHER_H_
#define FAD_SRC_FAD_PATTERNMATCHINGVOCALCULATOR_PATTERNMATCHER_AKAZEPATTERNMATCHER_AKAZEPATTERNMATCHER_H_

#include <YAMLHelper.hpp>

#include <improc.h>

#include "../PatternMatcherBase.h"

namespace fad {
    class AKAZEPatternMatcher : public base::PatternMatcherBase {
    public:
        AKAZEPatternMatcher();
        ~AKAZEPatternMatcher();

        bool solve(const cv::Mat&   pattern_img,
                   const cv::Mat&   target_img,
                   const cv::Point& point,
                   const double&    max_dist_thr) override;
    private:
        const std::string PARAM_YAML_PATH = "/data/OdometryCalculator/PatternMatchingVOCalculator/PatternMatcher/AKAZEPatternMatcher/param.yaml";

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

        std::unique_ptr<improc::AKAZEPatternMatcher> pattern_matcher_;
    };
}

#endif /* FAD_SRC_FAD_PATTERNMATCHINGVOCALCULATOR_PATTERNMATCHER_AKAZEPATTERNMATCHER_AKAZEPATTERNMATCHER_H_ */
