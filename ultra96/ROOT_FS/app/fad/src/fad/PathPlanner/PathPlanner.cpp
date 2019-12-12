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

#include "PathPlanner.h"

namespace fad {
    PathPlanner::PathPlanner() {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root>");
        }

        // パラメータを読み込む
        core::YAMLHelper::readStruct(std::getenv("FAD_ROOT") + PARAM_YAML_PATH, param_, "param");

        // 各インスタンスを初期化
        space_   = std::make_unique<pln::EuclideanSpace>(param_.dim);
        planner_ = std::make_unique<pln::InformedRRTStar>(param_.dim,
                                                          param_.max_sampling_num,
                                                          param_.goal_sampling_rate,
                                                          param_.expand_dist,
                                                          param_.R,
                                                          param_.goal_region_radius);

        // 車体の大きさを設定
        double tire_tread;
        core::YAMLHelper::read(std::getenv("FAD_ROOT") + MOTOR_PARAM_YAML_PATH, tire_tread, "Motor", "tire_tread");
        planner_->setBodySize(tire_tread / 2);
    }

    PathPlanner::~PathPlanner() {
    }

    void PathPlanner::applyConstraint(const core::EnvironmentMap<core::ConstraintType>& constraint,
                                      cv::Rect2d region) {
        if(region == cv::Rect2d(-1, -1, -1, -1)) {
            region = cv::Rect2d(0, 0, constraint.getRealWidth(), constraint.getRealHeight());
        }

        // 制約を定義
        const auto region_rect_pt = cv::Rect(constraint.getPixPoint(region.tl().x, region.tl().y),
                                             constraint.getPixPoint(region.br().x, region.br().y));
        std::vector<pln::ConstraintType> constraint_map;
        constraint_map.reserve(region_rect_pt.size().area());

        for(int yi = 0; yi < constraint.map.rows; yi++) {
            for(int xi = 0; xi < constraint.map.cols; xi++) {
                const auto val = (constraint.map.at<uint8_t>(yi, xi) & static_cast<uint8_t>(core::ConstraintType::ENTERABLE)) ?
                    pln::ConstraintType::ENTAERABLE : pln::ConstraintType::NOENTRY;
                constraint_map.emplace_back(val);
            }
        }

        std::vector<uint32_t> each_dim_size{ (uint32_t)constraint.map.cols, (uint32_t)constraint.map.rows };
        std::vector<pln::Bound> bounds{ pln::Bound(region.tl().x, region.br().x), pln::Bound(region.tl().y, region.br().y) };
        space_->setBound(bounds);

        // プランナーに反映
        constraint_ = std::make_shared<pln::SemanticSegmentConstraint>(*space_, constraint_map, each_dim_size);
        planner_->setProblemDefinition(constraint_);
    }

    const std::vector<core::VehicleState>& PathPlanner::getResult() const {
        return result_;
    }

    bool PathPlanner::solve(const core::VehicleState&              current_state,
                            const std::vector<core::VehicleState>& target_states) {
        auto validate_state = [&](const core::VehicleState& state) -> bool {
                                  return (space_->getBound(1).low <= state.x &&
                                          state.x < space_->getBound(1).high &&
                                          space_->getBound(2).low <= state.y &&
                                          state.y < space_->getBound(2).high) ? true : false;
                              };
        if(!validate_state(current_state)) {
            throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                        "Point value is invalid");
        }

        std::vector<core::VehicleState> result;
        auto start_state = current_state;
        auto success_planning = false;
        for(const auto& target_state : target_states) {
            if(!validate_state(target_state)) {
                throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                            "Point value is invalid");
            }

            pln::State start(start_state.x, start_state.y);
            pln::State goal(target_state.x, target_state.y);
            if(start == goal) {
                result.emplace_back(goal.vals[0], goal.vals[1], 0, target_state.v);
                continue;
            }

            // 終了条件を設定する
            planner_->setTerminateSearchCost(target_state.distanceFrom(start_state) * param_.terminate_ratio);

            // 解が見つかれば結果を格納する
            bool status = planner_->solve(start, goal);
            if(status) {
                const auto& result_nodes = planner_->getResult();
                for(const auto& result_node : result_nodes) {
                    result.emplace_back(result_node.vals[0], result_node.vals[1], 0, target_state.v);
                }
                success_planning = true;
                start_state = target_state;
            }
        }

        if(!success_planning) {
            return false;
        }
        else {
            for(size_t ri = 0; ri < result.size() - 1; ri++) {
                result[ri].t = core::Theta(std::atan2(result[ri + 1].y - result[ri].y, result[ri + 1].x - result[ri].x));
            }
            if(1 < result.size()) result[result.size() - 1].t = result[result.size() - 2].t;
            result_ = std::move(result);
            return true;
        }
    }
}
