/**
 *  SelfLocalizedPurePursuit: PurePursuitによる経路追従を、状態を推定しながら行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada
 *  Copyright (C) 2019 Yuta Ishida
 *.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *      Ishida Yuta    <ri0066fs@ed.ritsumei.ac.jp>
 *
 */

#include "SelfLocalizedPurePursuit.h"

namespace fad {
    SelfLocalizedPurePursuit::SelfLocalizedPurePursuit() {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        core::YAMLHelper::readStruct(root_path + MOTOR_PARAM_YAML_PATH, m_param_, "Motor");
        core::YAMLHelper::readStruct(root_path + SELFLOCALIZEDPUREPURSUIT_PARAM_YAML_PATH, param_, "Basis");

        PIDParam pid_param;
        core::YAMLHelper::readStruct(root_path + SELFLOCALIZEDPUREPURSUIT_PARAM_YAML_PATH, pid_param, "PID");
        pid_ctrl_ = std::make_unique<control::PID>(pid_param.P_gain, pid_param.I_gain, pid_param.D_gain, 0.0);

        external_current_state_ = {0,0,0,0};
        internal_current_state_ = {0,0,0,0};
        prev_time_ = std::chrono::system_clock::now();
    }

    SelfLocalizedPurePursuit::~SelfLocalizedPurePursuit() {
    }

    void SelfLocalizedPurePursuit::calc() {
        // 現在の状態が与えられたら現在の状態を更新する。
        if(external_current_state_ != current_state_) {
            external_current_state_ = current_state_;
            internal_current_state_ = current_state_;
            prev_time_              = std::chrono::system_clock::now();
        }
        SelfLocalizedPurePursuit::updateInternalState();


        const auto norm = internal_current_state_.distanceFrom(target_state_);

        // PID制御器を用いて目標速度を導出
        pid_ctrl_->setTargetValue((norm <= param_.norm_thr) ? 0.0 : target_state_.v);
        pid_ctrl_->updete(internal_current_state_.v);
        const auto velocity = pid_ctrl_->getOutValue();

        // 目標状態が存在する方向と現状態の方向との差を求める
        const auto alpha  = core::Theta(std::atan2(target_state_.y - internal_current_state_.y, target_state_.x - internal_current_state_.x));
        const auto t_diff = alpha - internal_current_state_.t;

        // 旋回半径・旋回角速度を求める
        const auto R = norm / (2.0 * std::sin(t_diff.get()));
        const auto w_ = 2.0 * velocity * std::sin(t_diff.get()) / norm;

        // 角速度を求める
        av_.r = (R + m_param_.tire_tread / 2) * w_;
        av_.l = (R - m_param_.tire_tread / 2) * w_;
    }

    void SelfLocalizedPurePursuit::updateInternalState() {
        const auto current_time = std::chrono::system_clock::now();
        const auto timestep     = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - prev_time_).count() / 1e9;
        prev_time_ = current_time;

        internal_current_state_.x += internal_current_state_.v * std::cos(internal_current_state_.t.get()) * timestep;
        internal_current_state_.y += internal_current_state_.v * std::sin(internal_current_state_.t.get()) * timestep;
        internal_current_state_.t += core::Theta(std::tan(w_) * timestep);
        internal_current_state_.v = (av_.r + av_.l) * (m_param_.tire_tread / core::PI / 2) / 2;
    }
}
