/**
 *  Util: ユーティリティクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_UTIL_HPP_
#define INCLUDE_UTIL_HPP_

#include <iostream>
#include <cmath>

namespace core {
    class Util {
        public:
        template<typename T>
        static T clamp(T v, T begin, T end) {
            return (v < begin) ? begin : (end < v) ? end : v;
        }
    };
}

#endif /* INCLUDE_UTIL_HPP_ */
