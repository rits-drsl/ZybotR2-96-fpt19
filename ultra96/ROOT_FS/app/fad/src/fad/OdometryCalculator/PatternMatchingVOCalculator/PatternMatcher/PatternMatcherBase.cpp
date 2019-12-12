/**
 *  PatternMatcherBase: 路面画像と地図画像のマッチングを行うクラスの基底クラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "PatternMatcherBase.h"

namespace fad {
    namespace base {
        PatternMatcherBase::PatternMatcherBase() : result_() { }
        PatternMatcherBase::~PatternMatcherBase() { }
        const core::PatternMatcherDelta& PatternMatcherBase::getResult() const {
            return result_;
        }
    }
}
