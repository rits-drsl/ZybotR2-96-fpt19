/**
 *  ConstraintGenerator: 現状態を定義し、制約の生成を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "ConstraintGenerator.h"

namespace fad {
    ConstraintGenerator::ConstraintGenerator() :
        tl_detection_results_index_(0),
        time_of_stop_by_tl_(std::chrono::system_clock::now()),
        time_of_dept_by_tl_(std::chrono::system_clock::now())
    {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }

        const auto root_path = std::getenv("FAD_ROOT");
        core::YAMLHelper::readStruct(root_path + PARAM_YAML_PATH, param_, "Basis");
        core::YAMLHelper::readStruct(root_path + PARAM_YAML_PATH, tl_param_, "TrafficLight");
        core::YAMLHelper::readStruct(root_path + PARAM_YAML_PATH, obstacle_param_, "Obstacle");

        double tire_tread;
        core::YAMLHelper::read(root_path + MOTOR_PARAM_YAML_PATH, tire_tread, "Motor", "tire_tread");
        vehicle_size_ = tire_tread / 2;

        pedestrian_detector_ = std::make_unique<PedestrianDetector>();
        tl_detector_ = std::make_unique<TrafficLightDetector>();
        tl_detection_results_queue_ = std::vector<TrafficLightDetector::TLDetectInfo>(tl_param_.detection_results_queue_depth);
    }

    ConstraintGenerator::~ConstraintGenerator() {
    }

    void ConstraintGenerator::init() {
        state_.clear();
        route_info_ = core::RouteInfo();

        tl_current_status_ = core::TrafficLightStatus();
        tl_prev_status_    = core::TrafficLightStatus();
        tl_detection_results_queue_ = std::vector<TrafficLightDetector::TLDetectInfo>(tl_param_.detection_results_queue_depth);
        tl_detection_results_index_ = 0;
        time_of_stop_by_tl_ = std::chrono::system_clock::now();
        time_of_dept_by_tl_ = std::chrono::system_clock::now();

        obstacles_.clear();

        pedestrian_ = core::PedestrianStatus();
    }

    const std::set<ConstraintGenerator::Type>& ConstraintGenerator::getState() const {
        return state_;
    }

    const core::RouteInfo& ConstraintGenerator::getRouteInfo() const {
        return route_info_;
    }

    const core::TrafficLightStatus& ConstraintGenerator::getTLDetectionResult() const {
        return tl_current_status_;
    }

    const core::PedestrianStatus& ConstraintGenerator::getPedestrianDetectionResult() const {
        return pedestrian_;
    }

    bool ConstraintGenerator::detectTrafficLights(const cv::Mat& color_img,
                                                  const double&  distance_to_tl) {
        if(!param_.enable_traffic_light_detection) return false;
        const auto& q_depth = tl_param_.detection_results_queue_depth;
        tl_detection_results_index_ = (tl_detection_results_index_ == q_depth - 1) ? 0 : tl_detection_results_index_ + 1;
        auto& result_dst = tl_detection_results_queue_[tl_detection_results_index_];

        if(!tl_detector_->detect(color_img, distance_to_tl)) {
            result_dst = TrafficLightDetector::TLDetectInfo();
        }
        else {
            result_dst = tl_detector_->getResult();
        }
        return result_dst.type == TrafficLightDetector::TLDetectInfo::Type::NONE ? false : true;
    }

    bool ConstraintGenerator::detectPedestrians(const cv::Mat&            color_img,
                                                const core::VehicleState& current_state) {
        if(!param_.enable_pedestrian_detection) return false;
        pedestrian_ = core::PedestrianStatus();
        if(!pedestrian_detector_->detect(color_img)) {
            pedestrian_ = pedestrian_detector_->getResult();
        }
        return pedestrian_.rect.empty() ? false : true;
    }

    bool ConstraintGenerator::detectObstacles(const core::EnvironmentMap<uint8_t>& img,
                                              const core::VehicleState&            current_state) {
        if(!param_.enable_object_detection) return false;
        // 変換行列を生成
        const auto rotate       = Eigen::Rotation2Df((current_state.t + core::Theta(core::PI / 2)).get());
        const auto translation1 = Eigen::Translation<float, 2>(-img.map.cols / 2, -img.map.rows / 2 - (int)img.getCorrespondPixNum(img.offset.x));
        const auto translation2 = Eigen::Translation<float, 2>(img.getCorrespondPixNum(current_state.x), img.getCorrespondPixNum(current_state.y));
        const auto trans_mat    = translation2 * rotate * translation1;

        // 既存のすべての障害物から一定距離以上離れているかを確認する
        auto validate_new_obstacle = [&](const cv::Point2d& pt) -> bool {
                                         for(const auto& obstacle : obstacles_) {
                                             if(cv::norm(obstacle.pt - pt) <= obstacle_param_.occupation_radius) {
                                                 return false;
                                             }
                                         }
                                         return true;
                                     };

        cv::Mat closing_img;
        // Opening・Closing
        cv::morphologyEx(img.map, closing_img, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1,-1), obstacle_param_.nof_opening);
        cv::morphologyEx(closing_img, closing_img, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), obstacle_param_.nof_closing);

        // ラベリングを行い、面積が一定範囲内であれば障害物とする
        const auto prev_obstacles_size = obstacles_.size();
        const auto label_info_map = improc::LabelingExecutor::execute(closing_img);
        for(const auto& label_info : label_info_map) {
            const auto area = label_info.second.area * std::pow(img.ratio, 2);
            if(obstacle_param_.area_lower <= area && area <= obstacle_param_.area_upper) {
                const Eigen::Vector2f projection_pt = trans_mat * Eigen::Vector2f(label_info.second.center.x, label_info.second.end.y) * img.ratio;
                const auto pt = cv::Point2d(projection_pt.x(), projection_pt.y());
                if(validate_new_obstacle(pt)) {
                    const auto max_val = std::max(label_info.second.end.x - label_info.second.begin.x, label_info.second.end.y - label_info.second.begin.y);
                    obstacles_.emplace_back(pt, max_val * img.ratio);
                }
            }
        }

        return obstacles_.size() != prev_obstacles_size ? true : false;
    }

    void ConstraintGenerator::generate(const core::RouteInfo& route_info) {
        route_info_ = route_info;
        const auto current_time = std::chrono::system_clock::now();
        const auto region_offset_pix = route_info_.constraint.getPixPoint(route_info_.region.x, route_info_.region.y);

        // 赤信号を検出している場合、車体を停止させる
        tl_prev_status_    = tl_current_status_;
        tl_current_status_ = judgeTLStatus(route_info);
        if(tl_current_status_.type == core::TrafficLightStatus::Type::RED) {
            route_info_.target_states = std::vector<core::VehicleState>(1, route_info_.current_state);
            route_info_.target_states.front().v = 0;
            if((tl_prev_status_.type == core::TrafficLightStatus::Type::GREEN || tl_prev_status_.type == core::TrafficLightStatus::Type::NO_DETECT)) {
                time_of_stop_by_tl_ = current_time;
            }
        }
        else if((tl_prev_status_.type == core::TrafficLightStatus::Type::RED) &&
                (tl_current_status_.type == core::TrafficLightStatus::Type::GREEN || tl_current_status_.type == core::TrafficLightStatus::Type::NO_DETECT)) {
            time_of_dept_by_tl_ = current_time;
        }

        // 停止線に差し掛かった際、目標速度を変更する
        if(route_info_.stop_line) {
            for(auto& v : route_info_.target_states) {
                v.v = std::max(v.v * tl_param_.velocity_ratio_when_stop_line, param_.target_velocity_lower);
            }
        }

        auto validate_pt = [&](const cv::Point& pt) -> bool {
                               const auto& w = route_info_.constraint.map.cols;
                               const auto& h = route_info_.constraint.map.rows;
                               return (0 <= pt.x && pt.x < w && 0 <= pt.y && pt.y < h) ? true : false;
                           };

        // 障害物付近の制約を更新する
        auto exist_obs_in_region = false;
        for(const auto& obstacle : obstacles_) {
            // 障害物として追加されてから一定時間が過ぎていれば有効としない
            if(obstacle_param_.valid_time_s <= std::chrono::duration_cast<std::chrono::microseconds>(current_time - obstacle.time).count() / 1e6) {
                continue;
            }
            // 進入可能領域外の障害物は有効としない
            const auto obstacle_pix_pt = route_info_.constraint.getPixPoint(obstacle.pt - route_info_.region.tl());
            if(route_info_.constraint.map.at<uint8_t>(obstacle_pix_pt.y, obstacle_pix_pt.x) != (uint8_t)core::ConstraintType::ENTERABLE) {
                continue;
            }
            // 制約の更新
            const auto obstacle_size_pix = route_info_.constraint.getCorrespondPixNum(obstacle.size);
            const auto entarable_radius_pix = route_info_.constraint.getCorrespondPixNum(obstacle_param_.entarable_radius);
            for(int yi = 0; yi < route_info_.constraint.map.rows; yi++) {
                for(int xi = 0; xi < route_info_.constraint.map.cols; xi++) {
                    const auto norm = cv::norm(obstacle_pix_pt - cv::Point(xi, yi));
                    if(norm <= obstacle_size_pix + entarable_radius_pix) {
                        exist_obs_in_region = true;
                        if(!(route_info_.constraint.map.at<uint8_t>(yi, xi) & (uint8_t)core::ConstraintType::NOENTRY)) {
                            route_info_.constraint.map.at<uint8_t>(yi, xi) |= (uint8_t)core::ConstraintType::ENTERABLE;
                        }
                        if(norm < obstacle_size_pix) {
                            if(route_info_.constraint.map.at<uint8_t>(yi, xi) ==  (uint8_t)core::ConstraintType::ENTERABLE) {
                                route_info_.constraint.map.at<uint8_t>(yi, xi) &= ~(uint8_t)core::ConstraintType::ENTERABLE;
                            }
                        }
                    }
                }
            }
        }

        // 障害物が存在する場合、目標速度を変更する
        if(exist_obs_in_region) {
            for(auto& v : route_info_.target_states) {
                v.v = std::max(v.v * obstacle_param_.velocity_ratio_when_detect_obs, param_.target_velocity_lower);
            }
        }

        // 現状態が進入禁止領域内にある場合、一時的に付近を進入可能領域にする
        const auto current_status_pix = route_info_.constraint.getPixPoint(route_info_.current_state.x, route_info_.current_state.y) - region_offset_pix;
        auto check_whether_state_is_enterable = [&](const core::VehicleState& state) -> bool {
                                                    const auto region_tl = core::VehicleState(route_info_.region.x, route_info_.region.y);
                                                    const auto around_states = (state - region_tl).getEightStatesOnCircumference(vehicle_size_);
                                                    for(const auto& around_state : around_states) {
                                                        const auto pts = route_info_.constraint.getPixPoint(around_state.x, around_state.y);
                                                        if(!(route_info_.constraint.map.at<uint8_t>(pts.y, pts.x) & (uint8_t)core::ConstraintType::ENTERABLE)) {
                                                            return false;
                                                        }
                                                    }
                                                    return true;
                                                };
        if(validate_pt(current_status_pix) && !check_whether_state_is_enterable(route_info_.current_state)) {
            for(int yi = 0; yi < route_info_.constraint.map.rows; yi++) {
                for(int xi = 0; xi < route_info_.constraint.map.cols; xi++) {
                    const auto norm = cv::norm(current_status_pix - cv::Point(xi, yi));
                    if(norm <= route_info_.constraint.getCorrespondPixNum(param_.escape_radius)) {
                        route_info_.constraint.map.at<uint8_t>(yi, xi) |= (uint8_t)core::ConstraintType::ENTERABLE;
                    }
                }
            }
        }
    }

    core::TrafficLightStatus ConstraintGenerator::judgeTLStatus(const core::RouteInfo& route_info) const {
        auto calc_elapsed_time = [&](auto& t) -> double {
                                     const auto current_time = std::chrono::system_clock::now();
                                     return std::chrono::duration_cast<std::chrono::microseconds>(current_time - t).count() / 1e6;
                                 };
        // 車体が停止線付近に位置しない or
        // 赤信号検出により停止している時間が一定以上 or
        // 前回停止してから経過した時間が一定以下の場合
        if(!route_info.stop_line ||
           (tl_current_status_.type == core::TrafficLightStatus::Type::RED && tl_param_.limit_time_of_red_light <= calc_elapsed_time(time_of_stop_by_tl_)) ||
           calc_elapsed_time(time_of_dept_by_tl_) <= tl_param_.limit_time_of_rediscovery) {
            return core::TrafficLightStatus();
        }

        // 結果を取得する
        const auto& nof_thr  = tl_param_.nof_detection_for_continuous;
        const auto& dist_thr = tl_param_.distance_thr_for_continuous;
        auto prev_result = tl_detection_results_queue_[tl_detection_results_index_];
        std::vector<std::pair<TrafficLightDetector::TLDetectInfo::Type, size_t>> results(1, std::make_pair(prev_result.type, 0));
        for(size_t i = 0; i < tl_param_.detection_results_queue_depth; i++) {
            int index = (tl_detection_results_index_ - i);
            if(index < 0) {
                index += tl_param_.detection_results_queue_depth;
            }
            const auto& result = tl_detection_results_queue_[index];
            if(cv::norm(result.rect.tl() - prev_result.rect.tl()) <= dist_thr) {
                if(result.type == prev_result.type) {
                    results.back().second++;
                }
                else if(results.back().second < nof_thr) {
                    results.back() = std::make_pair(result.type, 1);
                }
                else {
                    results.emplace_back(result.type, 1);
                }
            }
            prev_result = result;
        }
        if(results.back().second < nof_thr) {
            results.pop_back();
        }

        // 認識結果の時系列から場合分けを行う
        // NOTE: 赤信号の検出しか行わないため簡略化
        if(results.size() == 0) {
            return core::TrafficLightStatus();
        }
        else {
            const auto& front_type = results[0].first;
            if(front_type == TrafficLightDetector::TLDetectInfo::Type::RED) {
                return core::TrafficLightStatus(core::TrafficLightStatus::Type::RED,
                                                tl_detection_results_queue_[tl_detection_results_index_].rect,
                                                std::to_string(tl_detection_results_queue_[tl_detection_results_index_].val));
            }
            else {
                return core::TrafficLightStatus();
            }
        }
    }
}
