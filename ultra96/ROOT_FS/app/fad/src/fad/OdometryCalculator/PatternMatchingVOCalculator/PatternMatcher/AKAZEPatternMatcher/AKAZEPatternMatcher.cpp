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

#include "AKAZEPatternMatcher.h"

namespace fad {
    AKAZEPatternMatcher::AKAZEPatternMatcher() {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        AKAZEParam   akaze_param;
        MatcherParam matcher_param;
        core::YAMLHelper::readStruct(root_path + PARAM_YAML_PATH, akaze_param, "AKAZE");
        core::YAMLHelper::readStruct(root_path + PARAM_YAML_PATH, matcher_param, "Matcher");

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
    }

    AKAZEPatternMatcher::~AKAZEPatternMatcher() {
    }

    bool AKAZEPatternMatcher::solve(const cv::Mat&   pattern_img,
                                    const cv::Mat&   target_img,
                                    const cv::Point& point,
                                    const double&    max_dist_thr) {
        const auto status = pattern_matcher_->execute(target_img,
                                                      point,
                                                      pattern_img,
                                                      max_dist_thr);
        if(status) {
            const auto& result = pattern_matcher_->getResult();
            result_ = core::PatternMatcherDelta(result.x, result.y, result.theta);
        }
        return status;
    }
}
