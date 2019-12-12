/**
 *  InertialOdometry: 慣性オドメトリを表現するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_INERTIALODOMETRY_HPP_
#define INCLUDE_INERTIALODOMETRY_HPP_

#include <iostream>
#include <cmath>

#include <base/StateBase.hpp>

namespace core {
    class InertialOdometry : public base::StateBase {
    public:
        InertialOdometry() = default;
        constexpr InertialOdometry(const double& _x,
                                   const double& _y,
                                   const double& _t = 0,
                                   const double& _v = 0) :
            base::StateBase(_x, _y, _t, _v) { }

        constexpr InertialOdometry(const double& _x,
                                   const double& _y,
                                   const Theta&  _t,
                                   const double& _v = 0) :
            base::StateBase(_x, _y, _t, _v) { }

        constexpr explicit InertialOdometry(const base::StateBase& state) :
            base::StateBase(state) { }
    };
}

#endif /* INCLUDE_INERTIALODOMETRY_HPP_ */
