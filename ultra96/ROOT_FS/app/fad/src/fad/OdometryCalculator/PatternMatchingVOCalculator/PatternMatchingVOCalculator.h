/**
 *  PatternMatchingVOCalculator: 路面の俯瞰画像からVisualOdometryを導出するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_ODOMETRYCALCULATOR_PATTERNMATCHINGVOCALCULATOR_PATTERNMATCHINGVOCALCULATOR_H_
#define FAD_SRC_FAD_ODOMETRYCALCULATOR_PATTERNMATCHINGVOCALCULATOR_PATTERNMATCHINGVOCALCULATOR_H_

#include <Util.hpp>
#include <YAMLHelper.hpp>
#include <VehicleState.hpp>
#include <VisualOdometry.hpp>
#include <EnvironmentMap.hpp>
#include <RoadType.hpp>

#include "PatternMatcher/AKAZEPatternMatcher/AKAZEPatternMatcher.h"
#include "PatternMatcher/TMPatternMatcher/TMPatternMatcher.h"

namespace fad {
    class PatternMatchingVOCalculator {
    public:
        PatternMatchingVOCalculator();
        ~PatternMatchingVOCalculator();

        PatternMatchingVOCalculator(const PatternMatchingVOCalculator& obj) = delete;
        PatternMatchingVOCalculator &operator=(const PatternMatchingVOCalculator& obj) = delete;

        PatternMatchingVOCalculator(PatternMatchingVOCalculator&&) = default;

        void init(const core::EnvironmentMap<core::RoadType>& ref_world);

        core::VisualOdometry get(const bool reset = true);

        void update(const core::VehicleState&            current_state,
                    const core::EnvironmentMap<uint8_t>& bird_eye_bin_img,
                    const core::EnvironmentMap<uint8_t>& bird_eye_edge_img);

    private:
        const std::string PARAM_YAML_PATH = "/data/OdometryCalculator/PatternMatchingVOCalculator/param.yaml";

        class VOCalcParam : public core::YAMLHelper::ParamBase {
        public:
            std::string match_method;
            double      line_area_thr;
            double      cutting_size_of_ref_map;
            double      pattern_matcher_delta_threshord;
            double      cutting_left_ratio_of_pattern_image;
            double      cutting_right_ratio_of_pattern_image;
            double      cutting_above_ratio_of_pattern_image;
            double      scale_ratio;

            void read(const cv::FileNode& node) override {
                match_method                         = (std::string)node["match_method"];
                line_area_thr                        = (double)node["line_area_thr"];
                cutting_size_of_ref_map              = (double)node["cutting_size_of_ref_map"];
                pattern_matcher_delta_threshord      = (double)node["pattern_matcher_delta_threshord"];
                cutting_left_ratio_of_pattern_image  = (double)node["cutting_left_ratio_of_pattern_image"];
                cutting_right_ratio_of_pattern_image = (double)node["cutting_right_ratio_of_pattern_image"];
                cutting_above_ratio_of_pattern_image = (double)node["cutting_above_ratio_of_pattern_image"];
                scale_ratio                          = (double)node["scale_ratio"];
            }
        };

        VOCalcParam param_;

        core::VisualOdometry vo_;
        core::EnvironmentMap<core::RoadType> ref_world_;
        std::unique_ptr<base::PatternMatcherBase> pattern_matcher_;
    };
}

#endif /* FAD_SRC_FAD_ODOMETRYCALCULATOR_PATTERNMATCHINGVOCALCULATOR_PATTERNMATCHINGVOCALCULATOR_H_ */

