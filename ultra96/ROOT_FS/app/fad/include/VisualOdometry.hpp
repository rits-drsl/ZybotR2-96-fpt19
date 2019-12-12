/**
 *  VisualOdometry: 視覚オドメトリを表現するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_VISUALODOMETRY_HPP_
#define INCLUDE_VISUALODOMETRY_HPP_

#include <iostream>
#include <cmath>

#include <base/StateBase.hpp>

namespace core {
    class VisualOdometry : public base::StateBase {
    public:
        VisualOdometry() = default;
        constexpr VisualOdometry(const double& _x,
                                 const double& _y,
                                 const double& _t = 0,
                                 const double& _v = 0) :
            base::StateBase(_x, _y, _t, _v) { }

        constexpr VisualOdometry(const double& _x,
                                 const double& _y,
                                 const Theta&  _t,
                                 const double& _v = 0) :
            base::StateBase(_x, _y, _t, _v) { }

        constexpr explicit VisualOdometry(const base::StateBase& state) :
            base::StateBase(state) { }
    };
}

#endif /* INCLUDE_VISUALODOMETRY_HPP_ */
