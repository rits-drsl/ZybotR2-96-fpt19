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

#ifndef FAD_SRC_FAD_PATTERNMATCHINGVOCALCULATOR_PATTERNMATCHER_TMPATTERNMATCHER_TMPATTERNMATCHER_H_
#define FAD_SRC_FAD_PATTERNMATCHINGVOCALCULATOR_PATTERNMATCHER_TMPATTERNMATCHER_TMPATTERNMATCHER_H_

#include <YAMLHelper.hpp>

#include <improc.h>

#include "../PatternMatcherBase.h"

namespace fad {
    class TMPatternMatcher : public base::PatternMatcherBase {
    public:
        TMPatternMatcher();
        ~TMPatternMatcher();

        bool solve(const cv::Mat&   pattern_img,
                   const cv::Mat&   target_img,
                   const cv::Point& point,
                   const double&    max_dist_thr) override;
    private:
        const std::string PARAM_YAML_PATH = "/data/OdometryCalculator/PatternMatchingVOCalculator/PatternMatcher/TMPatternMatcher/param.yaml";

        class TMParam : public core::YAMLHelper::ParamBase {
        public:
            uint32_t opencv_tm_method; // CV_TM_SQDIFF = 0, CV_TM_SQDIFF_NORMED = 1, CV_TM_CCORR = 2, CV_TM_CCORR_NORMED = 3, CV_TM_CCOEFF = 4, CV_TM_CCOEFF_NORMED = 5
            double   search_range_degree;
            uint32_t nof_divisions;
            double   max_val_thr;

            void read(const cv::FileNode& node) override {
                opencv_tm_method    = (int)node["opencv_tm_method"];
                search_range_degree = (double)node["search_range_degree"];
                nof_divisions       = (int)node["nof_divisions"];
                max_val_thr         = (double)node["max_val_thr"];
            }
        };
        TMParam tm_param_;
    };
}

#endif /* FAD_SRC_FAD_PATTERNMATCHINGVOCALCULATOR_PATTERNMATCHER_TMPATTERNMATCHER_TMPATTERNMATCHER_H_ */
