/**
 *  PathTracer: 与えられたパスを追従するための角速度を導出するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Yuta Ishida.
 *  Authors:
 *      Yuya Kudo   <ri0049ee@ed.ritsumei.ac.jp>
 *      Yuta Ishida <ri0066fs@ed.ritsumei.ac.jp>
 *
 */

#include "PathTracer.h"

namespace fad {
    PathTracer::PathTracer() :
        path_(std::vector<core::VehicleState>()),
        path_ref_index_(0) {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        core::YAMLHelper::readStruct(root_path + TRACER_PARAM_YAML_PATH, param_, "Basis");

        if(param_.trace_method == "PP") {
            tracer_ = std::make_unique<PurePursuit>();
        }
        else if(param_.trace_method == "SLPP") {
            tracer_ = std::make_unique<SelfLocalizedPurePursuit>();
        }
        else if(param_.trace_method == "LQR") {
            tracer_ = std::make_unique<LQR>();
        }
        else {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Tracer type is invalid.");
        }
    }

    PathTracer::~PathTracer() {
    }

    void PathTracer::setPath(const std::vector<core::VehicleState>& path) {
        path_           = path;
        path_ref_index_ = 0;
    }

    void PathTracer::setCurrentState(const core::VehicleState& current_state) {
        current_state_ = current_state;
        tracer_->setCurrentState(current_state_);
    }

    const core::VehicleState& PathTracer::getRefState() const {
        return ref_state_;
    }

    core::AngularVelocity PathTracer::calcAngularVelocity() {
        if(path_.empty()) {
            ref_state_   = current_state_;
            ref_state_.v = 0.0;
        }
        else {
            // 現状態と目標状態との距離が一定以下になったら、二分探索の上限とする状態を次の状態に移行する
            const auto ratio = core::Util::clamp((current_state_.v - param_.lower_velocity) / (param_.upper_velocity - param_.lower_velocity), 0.0, 1.0);
            const auto dist_to_ref = param_.distance_current_to_ref_lower + (param_.distance_current_to_ref_upper - param_.distance_current_to_ref_lower) * ratio;
            while(path_ref_index_ != path_.size() - 1 && current_state_.distanceFrom(path_[path_ref_index_]) < dist_to_ref) {
                path_ref_index_++;
            }
            if(path_ref_index_ == path_.size() - 1 && current_state_.distanceFrom(path_[path_ref_index_]) < dist_to_ref) {
                path_ = std::vector<core::VehicleState>();
            }
            else if(path_ref_index_ == 0) {
                ref_state_ = path_[path_ref_index_];
            }
            else {
                // 現状態から一定の距離に存在する経路上の状態を二分探索で取得する
                auto r_target_state = path_[path_ref_index_];
                auto l_target_state = path_[path_ref_index_ - 1];
                while(1e-5 < r_target_state.distanceFrom(l_target_state)) {
                    const auto med_target_state = core::VehicleState((r_target_state + l_target_state) / 2);
                    if(current_state_.distanceFrom(med_target_state) <= dist_to_ref) {
                        l_target_state = med_target_state;
                    }
                    else {
                        r_target_state = med_target_state;
                    }
                }
                ref_state_ = r_target_state;

                const auto r_from_ref = path_[path_ref_index_].distanceFrom(ref_state_);
                const auto l_from_ref = path_[path_ref_index_ - 1].distanceFrom(ref_state_);
                ref_state_.t = core::Theta(std::atan2(ref_state_.y - path_[path_ref_index_ - 1].y, ref_state_.x - path_[path_ref_index_ - 1].x));

                ref_state_.v = (r_from_ref * path_[path_ref_index_].v + l_from_ref * path_[path_ref_index_ -1 ].v) / (r_from_ref + l_from_ref);
            }
            const auto dir = core::Theta(std::atan2(ref_state_.y - current_state_.y, ref_state_.x - current_state_.x));
            if(90 <= current_state_.t.disparityWith(dir).getDegree()) {
                ref_state_.v *= -1;
            }
        }

        tracer_->setTargetState(ref_state_);
        tracer_->calc();
        return !path_.empty() ? tracer_->getAngularVelocity() : core::AngularVelocity();
    }
}
