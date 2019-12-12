/**
e *  ConstraintType : 制約を表す列挙体
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_CONSTRAINTTYPE_HPP_
#define INCLUDE_CONSTRAINTTYPE_HPP_

namespace core {
    enum class ConstraintType {
        NONE      = 0b00000000,
        ENTERABLE = 0b00000001,
        OPPOSITE  = 0b00000010,
        NOENTRY   = 0b00000100
    };
}

#endif /* INCLUDE_CONSTRAINTTYPE_HPP_ */
