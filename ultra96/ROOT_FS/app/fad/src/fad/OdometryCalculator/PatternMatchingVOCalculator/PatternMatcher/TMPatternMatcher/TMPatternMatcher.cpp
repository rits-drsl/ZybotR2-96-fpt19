/**
 *  TMPatternMatcher: 路面画像と地図画像のマッチングを
 *                    テンプレートマッチングによって行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "TMPatternMatcher.h"

namespace fad {
    TMPatternMatcher::TMPatternMatcher() {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");
        core::YAMLHelper::readStruct(root_path + PARAM_YAML_PATH, tm_param_, "param");
    }

    TMPatternMatcher::~TMPatternMatcher() {
    }

    bool TMPatternMatcher::solve(const cv::Mat&   pattern_img,
                                 const cv::Mat&   target_img,
                                 const cv::Point& point,
                                 const double&    max_dist_thr) {
        cv::Point2f pattern_img_center = cv::Point2f(static_cast<float>(pattern_img.cols / 2),
                                                     static_cast<float>(pattern_img.rows / 2));

        auto total_max_val = 0.0;
        auto rot_degree    = -tm_param_.search_range_degree;
        auto resolution    = (2 * tm_param_.search_range_degree) / tm_param_.nof_divisions;
        for(size_t i = 0; i <= tm_param_.nof_divisions; i++) {
            cv::Mat affine;
            cv::getRotationMatrix2D(pattern_img_center, rot_degree, 1.0).copyTo(affine);

            cv::Mat rot_pattern_img;
            cv::warpAffine(pattern_img, rot_pattern_img, affine, pattern_img.size(), cv::INTER_CUBIC);

            cv::Mat result;
            cv::matchTemplate(rot_pattern_img, target_img, result, tm_param_.opencv_tm_method);

            cv::Point max_pt;
            double    max_val;
            cv::minMaxLoc(result, NULL, &max_val, NULL, &max_pt);

            max_pt += cv::Point(pattern_img_center);
            if(tm_param_.max_val_thr < max_val && cv::norm(point - max_pt) <= max_dist_thr && total_max_val < max_val) {
                result_ = core::PatternMatcherDelta(max_pt.x - point.x, max_pt.y - point.y, -1 * (rot_degree / 180.0) * core::PI);
                total_max_val = max_val;
            }
            rot_degree += resolution;
        }

        return (total_max_val != 0.0) ? true : false;
    }
}
