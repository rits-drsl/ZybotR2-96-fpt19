/**
 *  PedestrianStatus: 歩行者の検出状態を表すクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_PEDESTRIANSTATUS_HPP_
#define INCLUDE_PEDESTRIANSTATUS_HPP_

#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace core {
    class PedestrianStatus {
        public:
        cv::Rect    rect;
        cv::Mat     region;
        std::string comment;
        PedestrianStatus(const cv::Rect&    _rect    = cv::Rect(),
                         const cv::Mat&     _region  = cv::Mat(),
                         const std::string& _comment = "") :
            rect(_rect), region(_region), comment(_comment) { }
    };
}

#endif /* INCLUDE_PEDESTRIANSTATUS_HPP_ */
