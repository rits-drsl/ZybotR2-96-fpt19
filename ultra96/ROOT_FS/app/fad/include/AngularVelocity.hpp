/**
 *  AngularVelocity : 左右の車輪の角速度を表現するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_ANGULARVELOCITY_HPP_
#define INCLUDE_ANGULARVELOCITY_HPP_

namespace core {
    class AngularVelocity {
    public:
        double r{0};
        double l{0};

        AngularVelocity() = default;
        AngularVelocity(const double& _r, const double& _l) :
            r(_r), l(_l) { }
        ~AngularVelocity() { }
    };
}

#endif /* INCLUDE_ANGULARVELOCITY_HPP_ */
