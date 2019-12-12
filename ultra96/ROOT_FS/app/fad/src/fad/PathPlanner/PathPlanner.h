/**
 *  PathPlanner: 与えられた地図上でパスを生成するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_PATHPLANNER_PATHPLANNER_H_
#define FAD_SRC_FAD_PATHPLANNER_PATHPLANNER_H_

#include <iostream>
#include <vector>
#include <random>
#include <cstdint>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <EnvironmentMap.hpp>
#include <ConstraintType.hpp>
#include <YAMLHelper.hpp>
#include <VehicleState.hpp>

#include <planner.h>

namespace fad {
    namespace pln = planner;

    class PathPlanner {
    public:
        PathPlanner();
        ~PathPlanner();

        PathPlanner(const PathPlanner& obj) = delete;
        PathPlanner &operator=(const PathPlanner& obj) = delete;

        void applyConstraint(const core::EnvironmentMap<core::ConstraintType>& constraint,
                             cv::Rect2d region = cv::Rect2d(-1, -1, -1, -1));

        const std::vector<core::VehicleState>& getResult() const;

        bool solve(const core::VehicleState& current_state,
                   const std::vector<core::VehicleState>& target_states);

    private:
        const std::string PARAM_YAML_PATH       = "/data/PathPlanner/param.yaml";
        const std::string MOTOR_PARAM_YAML_PATH = "/data/Common/motor.yaml";

        class ParamInformedRRTStar : public core::YAMLHelper::ParamBase {
        public:
            uint32_t dim;
            uint32_t max_sampling_num;
            double   goal_sampling_rate;
            double   expand_dist;
            double   R;
            double   goal_region_radius;
            double   terminate_ratio;

            void read(const cv::FileNode& node) override {
                dim                = (int)node["dim"];
                max_sampling_num   = (int)node["max_sampling_num"];
                goal_sampling_rate = (double)node["goal_sampling_rate"];
                expand_dist        = (double)node["expand_dist"];
                R                  = (double)node["R"];
                goal_region_radius = (double)node["goal_region_radius"];
                terminate_ratio    = (double)node["terminate_ratio"];
            }
        };

        std::unique_ptr<pln::InformedRRTStar>      planner_;
        std::shared_ptr<pln::base::ConstraintBase> constraint_;
        std::unique_ptr<pln::EuclideanSpace>       space_;

        std::vector<core::VehicleState> result_;

        ParamInformedRRTStar param_;
    };
}

#endif /* FAD_SRC_FAD_PATHPLANNER_PATHPLANNER_H_ */
