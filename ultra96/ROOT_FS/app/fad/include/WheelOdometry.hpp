/**
 *  WheelOdometry: 車輪オドメトリを表現するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_WHEELODOMETRY_HPP_
#define INCLUDE_WHEELODOMETRY_HPP_

#include <iostream>
#include <cmath>

#include <base/StateBase.hpp>

namespace core {
    class WheelOdometry : public base::StateBase {
    public:
        WheelOdometry() = default;
        constexpr WheelOdometry(const double& _x,
                                const double& _y,
                                const double& _t = 0,
                                const double& _v = 0) :
            base::StateBase(_x, _y, _t, _v) { }

        constexpr WheelOdometry(const double& _x,
                                const double& _y,
                                const Theta&  _t,
                                const double& _v = 0) :
            base::StateBase(_x, _y, _t, _v) { }

        constexpr explicit WheelOdometry(const base::StateBase& state) :
            base::StateBase(state) { }
    };
}

#endif /* INCLUDE_WHEELODOMETRY_HPP_ */
