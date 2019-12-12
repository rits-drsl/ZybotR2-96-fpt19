/**
 *  ConstraintType : 路面の種類を表す列挙体
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_ROADTYPE_HPP_
#define INCLUDE_ROADTYPE_HPP_

namespace core {
    enum class RoadType {
        NONE = 0b00000000,
        LINE = 0b00000001,
        EDGE = 0b00000010,
    };
}

#endif /* INCLUDE_ROADTYPE_HPP_ */
