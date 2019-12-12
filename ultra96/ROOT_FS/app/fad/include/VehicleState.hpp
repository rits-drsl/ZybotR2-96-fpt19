/**
 *  VehicleState: 車体の状態を表すクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_VEHICLESTATE_HPP_
#define INCLUDE_VEHICLESTATE_HPP_

#include <iostream>
#include <cmath>

#include <base/StateBase.hpp>

namespace core {
    class VehicleState : public base::StateBase {
    public:
        VehicleState() = default;
        constexpr VehicleState(const double& _x,
                               const double& _y,
                               const double& _t = 0,
                               const double& _v = 0) :
            base::StateBase(_x, _y, _t, _v) { }

        constexpr VehicleState(const double& _x,
                               const double& _y,
                               const Theta&  _t,
                               const double& _v = 0) :
            base::StateBase(_x, _y, _t, _v) { }

        constexpr explicit VehicleState(const base::StateBase& state) :
            base::StateBase(state) { }
    };
}

#endif /* INCLUDE_VEHICLESTATE_HPP_ */
