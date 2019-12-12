/**
 *  RoutePlanner: 事前地図・経路情報・制約のベースを読み込み、
 *                車体周辺の情報の生成を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "RoutePlanner.h"

namespace fad {

    RoutePlanner::RoutePlanner()
        : current_route_index_(0) {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");
        core::YAMLHelper::readStruct(root_path + PARAM_YAML_PATH, param_, "Basis");

        auto digit = [](const uint32_t& num) -> uint32_t {
                         return std::to_string(num).size();
                     };
        auto zfill = [](const uint32_t& num, const uint32_t& digit) -> std::string {
                         std::string ret = std::to_string(num);
                         if(ret.size() < digit) ret = std::string(digit - ret.size(), '0') + ret;
                         return ret;
                     };

        //--- 環境情報を読み込む
        // 環境情報の分解能(環境地図1pixあたりの実空間における大きさ(メートル))を読み込む
        double ratio;
        core::YAMLHelper::read(root_path + ENVIRONMENT_YAML_PATH, ratio, "ratio");

        // 環境情報を持つファイルのパスを読み込む
        EnvironmentFilePathParam env_file_paths;
        core::YAMLHelper::readStruct(root_path + ENVIRONMENT_YAML_PATH, env_file_paths, "file_path_information");

        // 環境地図を読み込む
        auto line = cv::imread(root_path + env_file_paths.line_map, CV_8UC1);
        auto edge = cv::imread(root_path + env_file_paths.edge_map, CV_8UC1);
        cv::Mat world_map = cv::Mat::zeros(line.size(), CV_8UC1);
        for(int yi = 0; yi < line.rows; yi++) {
            for(int xi = 0; xi < line.cols; xi++) {
                world_map.data[xi + line.cols * yi] |= (line.data[xi+line.cols*yi] != 0) ?
                    static_cast<uint8_t>(core::RoadType::LINE) : static_cast<uint8_t>(core::RoadType::NONE);
                world_map.data[xi + line.cols * yi] |= (edge.data[xi+line.cols*yi] != 0) ?
                    static_cast<uint8_t>(core::RoadType::EDGE) : static_cast<uint8_t>(core::RoadType::NONE);
            }
        }

        world_.map   = world_map;
        world_.ratio = ratio;

        // パスの情報を読み込む
        int path_num;
        core::YAMLHelper::read(root_path + env_file_paths.paths, path_num, "num");
        const auto path_num_digit = digit(path_num);
        std::map<std::string, double> velocity_map;
        for(int path_i = 0; path_i < path_num; path_i++) {
            // 対象のパスを読み込む
            int point_num;
            std::string path_tag = "path" + zfill(path_i, path_num_digit);
            core::YAMLHelper::read(root_path + env_file_paths.paths, point_num, path_tag, "num");
            const auto point_num_digit = digit(point_num);

            decltype(PathInfo::points) points;
            for(int point_i = 0; point_i < point_num; point_i++) {
                ParamPoint point;
                std::string point_tag = "point" + zfill(point_i, point_num_digit);
                core::YAMLHelper::readStruct(root_path + env_file_paths.paths, point, path_tag, point_tag);

                double v;
                core::YAMLHelper::read(root_path + env_file_paths.paths, v, "velocity", point.velocity);
                if(v != 0.0) {
                    velocity_map[point.velocity] = v;
                }

                point.x *= ratio;
                point.y *= ratio;
                points.emplace_back(std::move(point));
            }

            // 対象のパスが持つ制約情報を読み込む
            std::string constraint_file_path;
            core::YAMLHelper::read(root_path + env_file_paths.paths, constraint_file_path, path_tag, "constraint_file");
            const auto constraint_src = cv::imread(root_path + constraint_file_path);
            if(constraint_src.size() != world_.map.size()) {
                throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                       "Constraint map size is invalid.");
            }

            core::EnvironmentMap<core::ConstraintType> constraint;
            constraint.map   = cv::Mat(constraint_src.size(), CV_8UC1);
            constraint.ratio = ratio;
            for(int yi = 0; yi < constraint_src.rows; yi++) {
                for(int xi = 0; xi < constraint_src.cols; xi++) {
                    auto constraint_pix = core::ConstraintType::NONE;
                    for(const auto& constraint_pair : CONSTRAINT_MAP) {
                        if(constraint_src.at<cv::Vec3b>(yi,xi) == constraint_pair.first) {
                            constraint_pix = constraint_pair.second;
                            break;
                        }
                    }
                    constraint.map.at<uint8_t>(yi, xi) = static_cast<uint8_t>(constraint_pix);
                }
            }
            route_base_.paths.emplace_back(std::move(points), std::move(constraint));
        }

        // パス間の接続情報を読み込む
        int conection_num;
        core::YAMLHelper::read(root_path + env_file_paths.path_connection, conection_num, "num");
        const auto conection_num_digit = digit(conection_num);
        for(int conn_i = 0; conn_i < conection_num; conn_i++) {
            ParamConn conn;
            std::string conn_tag = "conn" + zfill(conn_i, conection_num_digit);
            core::YAMLHelper::readStruct(root_path + env_file_paths.path_connection, conn, conn_tag);
            route_base_.path_conn.emplace_back(std::move(conn));
        }

        //  実際に走行する経路を読み込む
        int route_num;
        core::YAMLHelper::read(root_path + env_file_paths.route, route_num, "num");
        const auto route_num_digit = digit(route_num);
        for(int route_i = 0; route_i < route_num; route_i++) {
            ParamRoute ref_point;
            std::string ref_point_tag = "ref_point" + zfill(route_i, route_num_digit);
            core::YAMLHelper::readStruct(root_path + env_file_paths.route, ref_point, ref_point_tag);

            // パス間を跨ぐとき、接続情報が登録されているかを確認する
            if(1 <= route_i && ref_point.number != route_base_.route.back().number) {
                bool conn_is_ok = false;
                for(const auto& conn : route_base_.path_conn) {
                    if(conn.src_path_index  == route_base_.route.back().index &&
                       conn.src_path_number == route_base_.route.back().number &&
                       conn.dst_path_index  == ref_point.index &&
                       conn.dst_path_number == ref_point.number) {
                        conn_is_ok = true;
                        break;
                    }
                }
                if(!conn_is_ok) {
                    throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                           "Route information is invalid");
                }
            }
            route_base_.route.emplace_back(std::move(ref_point));
        }

        // 信号の情報を読み込む
        int tl_num;
        core::YAMLHelper::read(root_path + env_file_paths.traffic_light, tl_num, "num");
        for(int tl_i = 0; tl_i < tl_num; tl_i++) {
            ParamTL tl_state;
            core::YAMLHelper::readStruct(root_path + env_file_paths.traffic_light, tl_state, "tl" + std::to_string(tl_i));
            traffic_light_states_.emplace_back(tl_state.x * ratio, tl_state.y * ratio, (tl_state.theta / 180.0) * core::PI);
        }

        // 経路を生成
        for(const auto& node : route_base_.route) {
            const auto& x = route_base_.paths[node.number].points[node.index].x;
            const auto& y = route_base_.paths[node.number].points[node.index].y;
            const auto& velocity = route_base_.paths[node.number].points[node.index].velocity;
            route_.emplace_back(x, y, 0, velocity_map[velocity]);
        }
        for(size_t ri = 0; ri < route_.size() - 1; ri++) {
            route_[ri].t = core::Theta(atan2(route_[ri + 1].y - route_[ri].y, route_[ri + 1].x - route_[ri].x));
        }
        route_[route_.size() - 1].t = route_[route_.size() - 2].t;
        estimated_current_state_ = route_[current_route_index_];
    }

    RoutePlanner::~RoutePlanner() {
    }

    void RoutePlanner::setCurrentState(const core::VehicleState& state) {
        if(!(0 <= state.x && state.x <= world_.getRealWidth() && 0 <= state.y && state.y <= world_.getRealHeight())) {
            throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                        "State is out of range : state.x -> " +
                                        std::to_string(state.x) + ", state.y -> " + std::to_string(state.y));
        }

        const auto& target_state      = route_[current_route_index_];
        const auto& next_target_state = route_[std::min((int)current_route_index_ + 1, (int)route_.size() - 1)];
        if(next_target_state.distanceFrom(state) < target_state.distanceFrom(state)) {
            current_route_index_++;
        }
        estimated_current_state_ = state;
    }

    void RoutePlanner::init() noexcept {
        current_route_index_ = 0;
    }

    const core::EnvironmentMap<core::RoadType>& RoutePlanner::getWorldMap() const {
        return world_;
    }

    const std::vector<core::VehicleState>& RoutePlanner::getRoute() const {
        return route_;
    }

    const core::VehicleState& RoutePlanner::getRefCurrentState() const {
        return route_[current_route_index_];
    }

    void RoutePlanner::increaseRefCurrentStateIndex() {
        if(current_route_index_ < route_.size() - 1) {
            current_route_index_++;
        }
    }

    core::RouteInfo RoutePlanner::generateRouteInfomation() const {
        const auto  target_route_index = calcTargetRouteIndex(current_route_index_, 0.0);
        const auto& current_number = route_base_.route[current_route_index_].number;
        const auto& current_index  = route_base_.route[current_route_index_].index;
        const auto& target_number  = route_base_.route[target_route_index].number;

        core::RouteInfo info;
        info.current_state  = estimated_current_state_;
        info.stop_line      = route_base_.paths[current_number].points[current_index].is_stop_line;
        info.can_line_trace = route_base_.paths[current_number].points[current_index].line_trace_mode;
        info.constraint     = route_base_.paths[target_number].constraint.clone();

        // 目標状態の系列を生成
        std::vector<core::VehicleState> target_states;
        for(auto i = current_route_index_; i <= target_route_index; i++) {
            if(route_base_.route[i].is_via_point || i == target_route_index) {
                target_states.push_back(route_[i]);
                target_states.back().v = route_[current_route_index_].v;
            }
        }
        info.target_states = target_states;

        // パスプランニングを行う領域を定義
        const auto med_state = (estimated_current_state_ + info.target_states.back()) / 2;
        const auto helf_one_side_of_the_square = med_state.distanceFrom(estimated_current_state_) * param_.ratio_for_path_planner_region;
        const auto tl_x = std::max(0.0, med_state.x - helf_one_side_of_the_square);
        const auto tl_y = std::max(0.0, med_state.y - helf_one_side_of_the_square);
        const auto br_x = std::min(info.constraint.getRealWidth(), med_state.x + helf_one_side_of_the_square);
        const auto br_y = std::min(info.constraint.getRealHeight(), med_state.y + helf_one_side_of_the_square);

        info.region = cv::Rect2d(cv::Point2d(tl_x, tl_y), cv::Point2d(br_x, br_y));
        const auto region_rect_pt = cv::Rect(info.constraint.getPixPoint(info.region.tl().x, info.region.tl().y),
                                             info.constraint.getPixPoint(info.region.br().x, info.region.br().y));
        info.constraint.map = cv::Mat(info.constraint.map, region_rect_pt);

        // パスの接続時には直前の制約も参照する
        auto prev_number = target_number;
        for(size_t i = 0; i < param_.nof_checking_index_in_gen_constraint; i++) {
            if(target_route_index <= i) {
                break;
            }
            const auto& target_number = route_base_.route[target_route_index - (i + 1)].number;
            if(prev_number != target_number) {
                const auto& prev_constraint = cv::Mat(route_base_.paths[route_base_.route[target_route_index - (i + 1)].number].constraint.map, region_rect_pt);
                cv::bitwise_or(info.constraint.map, prev_constraint, info.constraint.map);
            }
            prev_number = target_number;
        }

        return info;
    }

    bool RoutePlanner::reachedGoal() const {
        return (calcTargetRouteIndex(current_route_index_, 0.0) == route_base_.route.size() - 1) ? true : false;
    }

    bool RoutePlanner::searchStateOfTL(core::TLState& state) const {
        auto min_dist = std::numeric_limits<double>::max();
        for(const auto& traffic_light : traffic_light_states_) {
            const auto c2tl = core::Theta(std::atan2(traffic_light.y - estimated_current_state_.y, traffic_light.x - estimated_current_state_.x));
            if((estimated_current_state_.t + core::Theta(core::PI)).disparityWith(traffic_light.t).getDegree() <= param_.search_tl_deg_thr &&
               estimated_current_state_.t.disparityWith(c2tl).getDegree() <= param_.search_tl_deg_thr) {
                const auto dist_to_tl = traffic_light.distanceFrom(estimated_current_state_);
                if(dist_to_tl <= param_.search_tl_dist_thr && dist_to_tl < min_dist) {
                    min_dist = traffic_light.distanceFrom(estimated_current_state_);
                    state    = traffic_light;
                }
            }
        }
        return (min_dist == std::numeric_limits<double>::max()) ? false : true;
    }

    uint32_t RoutePlanner::calcTargetRouteIndex(const size_t& route_index, double total_dist) const {
        // 現状態から理想経路上で距離Rだけ離れた状態を目標状態とする
        if(param_.R < total_dist || route_.size() == route_index) {
            return route_index - 1;
        }
        total_dist += route_[route_index].distanceFrom(route_[route_index + 1]);
        return calcTargetRouteIndex(route_index + 1, total_dist);
    };
}
