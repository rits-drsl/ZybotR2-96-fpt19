/**
 *  PatternMatcherDelta: パターンマッチング結果を表現するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_PATTERNMATCHERDELTA_HPP_
#define INCLUDE_PATTERNMATCHERDELTA_HPP_

#include <iostream>
#include <cmath>

#include <base/StateBase.hpp>

namespace core {
    class PatternMatcherDelta : public base::StateBase {
    public:
        PatternMatcherDelta() = default;
        constexpr PatternMatcherDelta(const double& _x,
                                      const double& _y,
                                      const double& _t = 0,
                                      const double& _v = 0) :
            base::StateBase(_x, _y, _t, _v) { }

        constexpr PatternMatcherDelta(const double& _x,
                                      const double& _y,
                                      const Theta&  _t = Theta(0.0),
                                      const double& _v = 0) :
            base::StateBase(_x, _y, _t, _v) { }

        constexpr explicit PatternMatcherDelta(const base::StateBase& state) :
            base::StateBase(state) { }
    };
}

#endif /* INCLUDE_PATTERNMATCHERDELTA_HPP_ */
