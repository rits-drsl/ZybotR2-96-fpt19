/**
 *  ParticleState: fad::ParticleFilterで使用する
 *                 粒子の状態を表現するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_PARTICLESTATE_HPP_
#define INCLUDE_PARTICLESTATE_HPP_

#include <iostream>
#include <cmath>

#include <base/StateBase.hpp>

namespace core {
    class ParticleState : public base::StateBase {
    public:
        double w = 0;

        ParticleState() = default;
        constexpr ParticleState(const double& _x,
                                const double& _y,
                                const double& _t = 0,
                                const double& _v = 0,
                                const double& _w = 0) :
            base::StateBase(_x, _y, _t, _v),
            w(_w) { }

        constexpr ParticleState(const double& _x,
                                const double& _y,
                                const Theta&  _t,
                                const double& _v = 0,
                                const double& _w = 0) :
            base::StateBase(_x, _y, _t, _v),
            w(_w) { }

        constexpr explicit ParticleState(const double& _w) :
            base::StateBase(),
            w(_w) { }

        constexpr ParticleState(const base::StateBase& state,
                                const double& _w = 0) :
            base::StateBase(state),
            w(_w) { }
    };
}

#endif /* INCLUDE_PARTICLESTATE_HPP_ */
