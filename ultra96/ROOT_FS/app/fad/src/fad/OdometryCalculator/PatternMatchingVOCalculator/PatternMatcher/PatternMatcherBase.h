/**
 *  PatternMatcherBase: 路面画像と地図画像のマッチングを行うクラスの基底クラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_PATTERNMATCHINGVOCALCULATOR_PATTERNMATCHER_PATTERNMATCHERBASE_H_
#define FAD_SRC_FAD_PATTERNMATCHINGVOCALCULATOR_PATTERNMATCHER_PATTERNMATCHERBASE_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <PatternMatcherDelta.hpp>

namespace fad {
    namespace base {
        class PatternMatcherBase {
        public:
            PatternMatcherBase();
            virtual ~PatternMatcherBase();

            virtual bool solve(const cv::Mat&   pattern_img,
                               const cv::Mat&   target_img,
                               const cv::Point& point,
                               const double&    max_dist_thr) = 0;
            const core::PatternMatcherDelta& getResult() const;

        protected:
            core::PatternMatcherDelta result_;
        };
    }
}

#endif /* FAD_SRC_FAD_PATTERNMATCHINGVOCALCULATOR_PATTERNMATCHER_PATTERNMATCHERBASE_H_ */

