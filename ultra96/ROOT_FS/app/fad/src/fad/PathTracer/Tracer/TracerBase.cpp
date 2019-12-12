/**
 *  TracerBase: ある状態からある状態まで車体を移動させるための制御出力を
 *              導出するクラスの基底クラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Yuta Ishida.
 *  Authors:
 *      Yuya Kudo   <ri0049ee@ed.ritsumei.ac.jp>
 *      Yuta Ishida <ri0066fs@ed.ritsumei.ac.jp>
 *
 */

#include "TracerBase.h"

namespace fad {
    namespace base {
        TracerBase::TracerBase() { }
        TracerBase::~TracerBase() { }

        void TracerBase::setCurrentState(const core::VehicleState& current_state) {
            current_state_ = current_state;
        }

        void TracerBase::setTargetState(const core::VehicleState& target_state) {
            target_state_ = target_state;
        }

        const core::AngularVelocity& TracerBase::getAngularVelocity() const {
            return av_;
        }
    }
}
