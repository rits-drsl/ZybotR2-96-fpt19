/**
 *  TrafficLightStatus : 信号の状態を表現する
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_TRAFFICLIGHTSTATUS_HPP_
#define INCLUDE_TRAFFICLIGHTSTATUS_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace core {
    class TrafficLightStatus {
    public:
        enum class Type {
            NO_DETECT,
            GREEN,
            TURNING_RED,
            RED,
        };

        Type        type;
        cv::Rect    rect;
        std::string comment;

        TrafficLightStatus() = default;
        TrafficLightStatus(const Type&        _type,
                           const cv::Rect&    _rect,
                           const std::string& _comment) :
            type(_type), rect(_rect), comment(_comment) { }
        ~TrafficLightStatus() { }
    };
}

#endif /* INCLUDE_TRAFFICLIGHTSTATUS_HPP_ */
