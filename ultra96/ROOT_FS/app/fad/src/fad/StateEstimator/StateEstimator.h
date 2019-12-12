/**
 *  StateEstimator: 路面画像・車輪オドメトリ・慣性オドメトリを用いて
 *                  自己位置推定を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_STATEESTIMATOR_STATEESTIMATOR_H_
#define FAD_SRC_FAD_STATEESTIMATOR_STATEESTIMATOR_H_

#include <memory>

#include <YAMLHelper.hpp>
#include <VehicleState.hpp>
#include <WheelOdometry.hpp>
#include <VisualOdometry.hpp>
#include <InertialOdometry.hpp>

#include "ParticleFilter/ParticleFilter.h"

namespace fad {
    class StateEstimator {
    public:
        StateEstimator();
        ~StateEstimator();

        StateEstimator(const StateEstimator& obj) = delete;
        StateEstimator &operator=(const StateEstimator& obj) = delete;

        void init(const core::VehicleState& init_state);

        const core::VehicleState& getCurrentState() const;

        const std::vector<core::ParticleState>& getEstimateStates() const;

        uint32_t getUpdateCnt() const;

        const std::vector<core::VehicleState>& getTrajectory() const;

        void update(const core::WheelOdometry&    wo,
                    const core::InertialOdometry& io,
                    const core::VisualOdometry&   vo = core::VisualOdometry());

    private:
        const std::string STATE_ESTIMATOR_PARAM_YAML_PATH = "/data/StateEstimator/param.yaml";

        class PFParam : public core::YAMLHelper::ParamBase {
        public:
            double wo_ps_sigma_when_vo_is_valid;
            double wo_t_sigma_when_vo_is_valid;
            double wo_ps_sigma_when_vo_is_invalid;
            double wo_t_sigma_when_vo_is_invalid;

            void read(const cv::FileNode& node) override {
                wo_ps_sigma_when_vo_is_valid   = (double)node["wo_ps_sigma_when_vo_is_valid"];
                wo_t_sigma_when_vo_is_valid    = (double)node["wo_t_sigma_when_vo_is_valid"];
                wo_ps_sigma_when_vo_is_invalid = (double)node["wo_ps_sigma_when_vo_is_invalid"];
                wo_t_sigma_when_vo_is_invalid  = (double)node["wo_t_sigma_when_vo_is_invalid"];
            }
        };

        PFParam pf_param_;

        std::unique_ptr<ParticleFilter> pf_;
        std::vector<core::VehicleState> trajectory_;
    };
}

#endif /* FAD_SRC_FAD_STATEESTIMATOR_STATEESTIMATOR_H_ */

