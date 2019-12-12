/**
 *  EnvironmentMap: ２次元の環境地図を定義するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_ENVIRONMENTMAP_HPP_
#define INCLUDE_ENVIRONMENTMAP_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace core {
    template<typename T>
    class EnvironmentMap {
    public:
        cv::Mat     map;
        double      ratio;   // 1pixあたりの実空間における大きさ(メートル)
        cv::Point2d offset;  // 実空間におけるx, y方向のオフセット

        EnvironmentMap() : ratio(1.0), offset(cv::Point(0.0, 0.0)) { }
        EnvironmentMap(const cv::Mat& _map, const double& _ratio, const cv::Point2d& _offset) :
            map(_map), ratio(_ratio), offset(_offset) { }
        ~EnvironmentMap() { }

        EnvironmentMap<T> clone() const {
            return { map.clone(), ratio, offset };
        }

        double getRealWidth() const {
            return map.cols * ratio;
        }

        double getRealHeight() const {
            return map.rows * ratio;
        }

        cv::Point2d getRealPoint(const uint32_t& x, const uint32_t& y) const {
            return cv::Point2d(x * ratio, y * ratio);
        }

        cv::Point2d getRealPoint(const cv::Point& pt) const {
            return cv::Point2d(pt) * ratio;
        }

        uint32_t getCorrespondPixNum(const double& val) const {
            return val / ratio;
        }

        cv::Point getPixPoint(const double& x, const double& y) const {
            return cv::Point(x / ratio, y / ratio);
        }

        cv::Point getPixPoint(const cv::Point2d& pt) const {
            return cv::Point(pt.x / ratio, pt.y / ratio);
        }

        cv::Rect validateRegion(const cv::Rect& region) const {
            auto ret = region;
            if(ret.x < 0) {
                ret.x = 0;
            }
            if(ret.y < 0) {
                ret.y = 0;
            }
            if(map.cols <= ret.br().x) {
                ret.width  -= ret.br().x - map.cols + 1;
            }
            if(map.rows <= ret.br().y) {
                ret.height -= ret.br().y - map.rows + 1;
            }
            return ret;
        }

        double ratioDifferenceFrom(const EnvironmentMap<T>& other) const {
            return ratio / other.ratio;
        };
    };
}

#endif /* INCLUDE_ENVIRONMENTMAP_HPP_ */
