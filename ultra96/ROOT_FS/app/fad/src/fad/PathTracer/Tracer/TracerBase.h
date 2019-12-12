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

#ifndef FAD_SRC_FAD_PATHTRACER_TRACER_TRACERBASE_TRACERBASE_H_
#define FAD_SRC_FAD_PATHTRACER_TRACER_TRACERBASE_TRACERBASE_H_

#include <VehicleState.hpp>
#include <AngularVelocity.hpp>

namespace fad {
    namespace base {
        class TracerBase {
        public:
            TracerBase();
            virtual ~TracerBase();

            void setCurrentState(const core::VehicleState& current_state);
            void setTargetState(const core::VehicleState& target_state);
            const core::AngularVelocity& getAngularVelocity() const;

            virtual void calc() = 0;

        protected:
            core::VehicleState current_state_;
            core::VehicleState target_state_;
            core::AngularVelocity av_;
        };
    }
}

#endif /* FAD_SRC_FAD_PATHTRACER_TRACER_TRACERBASE_TRACERBASE_H_ */
