/**
 *  IMUQuat: クオータニオンを表すクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_IMUQUAT_HPP_
#define INCLUDE_IMUQUAT_HPP_

namespace core {
    class IMUQuat {
    public:
        double w{0};
        double x{0};
        double y{0};
        double z{0};
        constexpr IMUQuat() = default;
        constexpr IMUQuat(const double _w,
                          const double _x,
                          const double _y,
                          const double _z) :
            w(_w), x(_x), y(_y), z(_z) { }
    };
}

#endif /* INCLUDE_IMUQUAT_HPP_ */
