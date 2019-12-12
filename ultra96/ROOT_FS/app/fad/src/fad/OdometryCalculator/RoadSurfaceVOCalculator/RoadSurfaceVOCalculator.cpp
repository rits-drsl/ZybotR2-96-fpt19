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

#include "RoadSurfaceVOCalculator.h"

namespace fad {
    RoadSurfaceVOCalculator::RoadSurfaceVOCalculator(const cv::Mat& init_frame) {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        AKAZEParam   akaze_param;
        MatcherParam matcher_param;
        core::YAMLHelper::readStruct(root_path + PATTERN_MATCHER_PARAM_YAML_PATH, akaze_param, "AKAZE");
        core::YAMLHelper::readStruct(root_path + PATTERN_MATCHER_PARAM_YAML_PATH, matcher_param, "Matcher");
        core::YAMLHelper::readStruct(root_path + RSVOC_PARAM_YAML_PATH, param_, "Basis");

        pattern_matcher_ = std::make_unique<improc::AKAZEPatternMatcher>(akaze_param.descriptor_type,
                                                                         akaze_param.descriptor_size,
                                                                         akaze_param.descriptor_channels,
                                                                         akaze_param.threshold,
                                                                         akaze_param.nOctaves,
                                                                         akaze_param.nOctaveLayers,
                                                                         akaze_param.diffusivity,
                                                                         matcher_param.DescriptorMatcherType,
                                                                         matcher_param.NNDR,
                                                                         matcher_param.matching_dist_thr);
        pattern_matcher_->computeFeatureInfo(init_frame, prev_feature_);
    }

    RoadSurfaceVOCalculator::~RoadSurfaceVOCalculator() {
    }

    void RoadSurfaceVOCalculator::init(const cv::Mat& init_frame) {
        vo_ = core::VisualOdometry();
        pattern_matcher_->computeFeatureInfo(init_frame, prev_feature_);
    }

    const core::VisualOdometry& RoadSurfaceVOCalculator::get() const {
        return vo_;
    }

    bool RoadSurfaceVOCalculator::run(const cv::Mat& current_frame) {
        improc::AKAZEPatternMatcher::FeatureInfo feature;

        auto status = pattern_matcher_->computeFeatureInfo(current_frame, feature);
        if(status) {
            const auto pt = cv::Point(current_frame.cols / 2, current_frame.rows / 2);
            status = pattern_matcher_->execute(feature, current_frame.size(), pt,
                                               prev_feature_, current_frame.size(),
                                               param_.pattern_matcher_delta_threshord);
        }

        if(status) {
            const auto& result = pattern_matcher_->getResult();
            vo_.x = result.x;
            vo_.y = result.y;
            vo_.t = core::Theta(result.theta);
        }

        prev_feature_ = std::move(feature);

        return status;
    }
}
