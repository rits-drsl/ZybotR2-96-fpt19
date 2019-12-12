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

#ifndef FAD_SRC_FAD_ROUTEPLANNER_ROUTEPLANNER_H_
#define FAD_SRC_FAD_ROUTEPLANNER_ROUTEPLANNER_H_

#include <iostream>
#include <vector>
#include <random>
#include <memory>
#include <cstdint>
#include <cstdlib>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <EnvironmentMap.hpp>
#include <ConstraintType.hpp>
#include <RoadType.hpp>
#include <Util.hpp>
#include <YAMLHelper.hpp>
#include <VehicleState.hpp>
#include <TLState.hpp>
#include <RouteInfo.hpp>

namespace fad {
    class RoutePlanner {
    public:
        RoutePlanner();
        ~RoutePlanner();

        RoutePlanner(const RoutePlanner& obj) = delete;
        RoutePlanner &operator=(const RoutePlanner& obj) = delete;

        /**
         *  @brief 現在位置を登録する
         *  @param (state) 現在位置
         */
        void setCurrentState(const core::VehicleState& state);

        /**
         *  @brief 走行状態を初期化する
         */
        void init() noexcept;

        /**
         *  @brief 地図を取得する
         */
        const core::EnvironmentMap<core::RoadType>& getWorldMap() const;

        /**
         *  @brief 経路を取得する
         */
        const std::vector<core::VehicleState>& getRoute() const;

        /**
         *  @brief 理想経路上における現在状態を取得する
         */
        const core::VehicleState& getRefCurrentState() const;

        /**
         *  @brief 理想経路上において、現状態として参照している状態を強制的にインクリメントする
         *  @note RoutePlanner::generateRouteInfomation() から得られる目標状態が
         *        障害物の出現等によって進入不能になった際に使用します
         */
        void increaseRefCurrentStateIndex();

        /**
         *  @brief 経路情報を生成・取得する
         */
        core::RouteInfo generateRouteInfomation() const;

        /**
         *  @brief 現状態から観測可能な信号を調べる
         *  @return 観測可能な信号が存在しなければFalseを返す
         */
        bool searchStateOfTL(core::TLState& state) const;

        /**
         *  @brief 設定された経路の終端に辿り着いたかどうかを調べる
         *  @return 経路の終端(ゴール)から距離R以内に到達していればtrueを返す
         */
        bool reachedGoal() const;
    private:
        const std::string PARAM_YAML_PATH       = "/data/RoutePlanner/param.yaml";
        const std::string ENVIRONMENT_YAML_PATH = "/data/RoutePlanner/environment.yaml";

        const std::vector<std::pair<cv::Vec3b, core::ConstraintType>> CONSTRAINT_MAP = {
            {cv::Vec3b{0,255,0}, core::ConstraintType::ENTERABLE},
            {cv::Vec3b{255,0,0}, core::ConstraintType::OPPOSITE},
            {cv::Vec3b{0,0,255}, core::ConstraintType::NOENTRY}};

        class RoutePlannerParam : public core::YAMLHelper::ParamBase {
        public:
            double R;
            double ratio_for_path_planner_region;
            uint32_t nof_checking_index_in_gen_constraint;
            double search_tl_dist_thr;
            double search_tl_deg_thr;

            void read(const cv::FileNode& node) override {
                R                                    = (double)node["R"];
                ratio_for_path_planner_region        = (double)node["ratio_for_path_planner_region"];
                nof_checking_index_in_gen_constraint = (int)node["nof_checking_index_in_gen_constraint"];
                search_tl_dist_thr                   = (double)node["search_tl_dist_thr"];
                search_tl_deg_thr                    = (double)node["search_tl_deg_thr"];
            }
        };

        class EnvironmentFilePathParam : public core::YAMLHelper::ParamBase {
        public:
            std::string edge_map;
            std::string line_map;
            std::string paths;
            std::string path_connection;
            std::string route;
            std::string traffic_light;

            void read(const cv::FileNode& node) override {
                edge_map        = (std::string)node["edge_map"];
                line_map        = (std::string)node["line_map"];
                paths           = (std::string)node["paths"];
                path_connection = (std::string)node["path_connection"];
                route           = (std::string)node["route"];
                traffic_light   = (std::string)node["traffic_light"];
            }
        };

        class ParamPoint : public core::YAMLHelper::ParamBase {
        public:
            double      x;
            double      y;
            bool        is_stop_line;
            bool        line_trace_mode;
            std::string velocity;

            void read(const cv::FileNode& node) override {
                x               = (double)node["x"];
                y               = (double)node["y"];
                is_stop_line    = (int)node["is_stop_line"];
                line_trace_mode = (int)node["line_trace_mode"];
                velocity        = (std::string)node["velocity"];
            }
        };

        class ParamTL : public core::YAMLHelper::ParamBase {
        public:
            double x;
            double y;
            double theta;

            void read(const cv::FileNode& node) override {
                x     = (double)node["x"];
                y     = (double)node["y"];
                theta = (double)node["theta"];
            }
        };

        class ParamConn : public core::YAMLHelper::ParamBase {
        public:
            uint32_t dst_path_index;
            uint32_t dst_path_number;
            uint32_t src_path_index;
            uint32_t src_path_number;

            void read(const cv::FileNode& node) override {
                src_path_index  = (int)node["src_path_index"];
                src_path_number = (int)node["src_path_number"];
                dst_path_index  = (int)node["dst_path_index"];
                dst_path_number = (int)node["dst_path_number"];
            }
        };

        class ParamRoute : public core::YAMLHelper::ParamBase {
        public:
            uint32_t index;
            uint32_t number;
            bool     is_via_point;

            void read(const cv::FileNode& node) override {
                index        = (int)node["index"];
                number       = (int)node["number"];
                is_via_point = (int)node["is_via_point"];
            }
        };

        struct PathInfo {
            std::vector<ParamPoint>                    points;     // パスを構成するノードの情報
            core::EnvironmentMap<core::ConstraintType> constraint; // 制約情報
            PathInfo(const std::vector<ParamPoint>& _points,
                     const core::EnvironmentMap<core::ConstraintType>& _constraint) :
                points(_points), constraint(_constraint) { }
        };

        struct RouteBase {
            std::vector<PathInfo>   paths;     // パスの情報
            std::vector<ParamConn>  path_conn; // パス同士の接続情報
            std::vector<ParamRoute> route;     // 実際に走行する経路
        };

        RouteBase route_base_;
        RoutePlannerParam param_;

        core::EnvironmentMap<core::RoadType> world_; // 地図
        std::vector<core::VehicleState>      route_; // 経路
        std::vector<core::TLState>           traffic_light_states_; // 信号
        uint32_t                             current_route_index_; // 参照する目標状態のindex
        core::VehicleState                   estimated_current_state_; // 外部から与えられた推定状態

        uint32_t calcTargetRouteIndex(const size_t& route_index, double total_dist) const;
    };
}

#endif /* FAD_SRC_FAD_ROUTEPLANNER_ROUTEPLANNER_H__ */
