/**
 *  RouteInfo: パスプランニングのための制約を生成する際に必要な情報を持つクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_ROUTEINFO_HPP_
#define INCLUDE_ROUTEINFO_HPP_

#include <iostream>
#include <cmath>

#include <VehicleState.hpp>
#include <EnvironmentMap.hpp>
#include <ConstraintType.hpp>

namespace core {
    class RouteInfo {
    public:
        core::VehicleState                         current_state;  // 現状態
        std::vector<core::VehicleState>            target_states{std::vector<core::VehicleState>(1)};  // 目標状態の系列
        core::EnvironmentMap<core::ConstraintType> constraint;  // 制約条件
        cv::Rect2d                                 region;  // パスプランニングの対象とする領域(実空間)
        bool                                       stop_line{false};  // 停止線であるかどうかを示すフラグ
        bool                                       can_line_trace{false};  // ライントレースが可能なポイントかどうかを示す

        RouteInfo() = default;
        RouteInfo(const core::VehicleState&                         _current_state,
                  const std::vector<core::VehicleState>&            _target_states,
                  const core::EnvironmentMap<core::ConstraintType>& _constraint,
                  const cv::Rect2d&                                 _region,
                  const bool&                                       _stop_line,
                  const bool&                                       _can_line_trace) :
            current_state(_current_state),
            target_states(_target_states),
            constraint(_constraint),
            region(_region),
            stop_line(_stop_line),
            can_line_trace(_can_line_trace) { }
        ~RouteInfo() { }
    };
}

#endif /* INCLUDE_ROUTEINFO_HPP_ */
