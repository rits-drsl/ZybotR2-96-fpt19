/**
 *  PurePursuit: PurePursuitによる経路追従を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Yuta Ishida.
 *  Authors:
 *      Yuya Kudo   <ri0049ee@ed.ritsumei.ac.jp>
 *      Yuta Ishida <ri0066fs@ed.ritsumei.ac.jp>
 *
 */

#include "PurePursuit.h"

namespace fad {
    PurePursuit::PurePursuit() {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        core::YAMLHelper::readStruct(root_path + MOTOR_PARAM_YAML_PATH, m_param_, "Motor");
        core::YAMLHelper::readStruct(root_path + PUREPURSUIT_PARAM_YAML_PATH, param_, "Basis");

        PIDParam pid_param;
        core::YAMLHelper::readStruct(root_path + PUREPURSUIT_PARAM_YAML_PATH, pid_param, "PID");
        pid_ctrl_ = std::make_unique<control::PID>(pid_param.P_gain, pid_param.I_gain, pid_param.D_gain, 0.0);
    }

    PurePursuit::~PurePursuit() {
    }

    void PurePursuit::calc() {
        const auto norm = current_state_.distanceFrom(target_state_);

        // PID制御器を用いて目標速度を導出
        pid_ctrl_->setTargetValue((norm <= param_.norm_thr) ? 0.0 : target_state_.v);
        pid_ctrl_->updete(current_state_.v);
        const auto velocity = pid_ctrl_->getOutValue();

        // 目標状態が存在する方向と現状態の方向との差を求める
        const auto alpha  = core::Theta(std::atan2(target_state_.y - current_state_.y, target_state_.x - current_state_.x));
        const auto t_diff = alpha - current_state_.t;

        // 旋回半径・旋回角速度を求める
        const auto R = norm / (2.0 * std::sin(t_diff.get()));
        const auto w = 2.0 * velocity * std::sin(t_diff.get()) / norm;

        // 角速度を求める
        av_.r = (R + m_param_.tire_tread / 2) * w;
        av_.l = (R - m_param_.tire_tread / 2) * w;
    }
}
