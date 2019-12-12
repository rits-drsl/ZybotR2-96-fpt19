/**
 *  TLState: 信号の状態を表すクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_TLSTATE_HPP_
#define INCLUDE_TLSTATE_HPP_

#include <iostream>
#include <cmath>

#include <base/StateBase.hpp>

namespace core {
    class TLState : public base::StateBase {
    public:
        TLState() = default;
        constexpr TLState(const double& _x,
                          const double& _y,
                          const double& _t = 0,
                          const double& _v = 0) :
            base::StateBase(_x, _y, _t, _v) { }

        constexpr TLState(const double& _x,
                          const double& _y,
                          const Theta&  _t,
                          const double& _v = 0) :
            base::StateBase(_x, _y, _t, _v) { }

        constexpr explicit TLState(const base::StateBase& state) :
            base::StateBase(state) { }
    };
}

#endif /* INCLUDE_TLSTATE_HPP_ */
