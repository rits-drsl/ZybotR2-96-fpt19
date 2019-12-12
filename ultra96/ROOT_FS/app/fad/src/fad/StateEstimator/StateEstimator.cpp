/**
 *  StateEstimator: 各種オドメトリを用いて自己位置推定を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "StateEstimator.h"

namespace fad {
    StateEstimator::StateEstimator() {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        core::YAMLHelper::readStruct(root_path + STATE_ESTIMATOR_PARAM_YAML_PATH, pf_param_, "PF");

        pf_ = std::make_unique<ParticleFilter>();
    }

    StateEstimator::~StateEstimator() {
    }

    void StateEstimator::init(const core::VehicleState& init_state) {
        pf_->init(init_state);
        trajectory_.clear();
    }

    const core::VehicleState& StateEstimator::getCurrentState() const {
        return pf_->getState();
    }

    const std::vector<core::ParticleState>& StateEstimator::getEstimateStates() const {
        return pf_->getParticles();
    }

    uint32_t StateEstimator::getUpdateCnt() const {
        return trajectory_.size();
    }

    const std::vector<core::VehicleState>& StateEstimator::getTrajectory() const {
        return trajectory_;
    }

    void StateEstimator::update(const core::WheelOdometry&    wo,
                                const core::InertialOdometry& io,
                                const core::VisualOdometry&   vo) {
        // 自己位置を更新する
        if(vo.isZero()) {
            pf_->estimate(wo, core::VisualOdometry(io.x, io.y, io.t), io,
                          pf_param_.wo_ps_sigma_when_vo_is_invalid,
                          pf_param_.wo_t_sigma_when_vo_is_invalid);
        }
        else {
            pf_->estimate(wo, vo, io,
                          pf_param_.wo_ps_sigma_when_vo_is_valid,
                          pf_param_.wo_t_sigma_when_vo_is_valid);
        }

        // 軌跡を保存する
        trajectory_.push_back(pf_->getState());
    }
}

