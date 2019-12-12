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

#ifndef FAD_SRC_FAD_CONSTRAINTGENERATOR_CONSTRAINTGENERATOR_H_
#define FAD_SRC_FAD_CONSTRAINTGENERATOR_CONSTRAINTGENERATOR_H_

#include <set>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <EnvironmentMap.hpp>
#include <ConstraintType.hpp>
#include <VehicleState.hpp>
#include <YAMLHelper.hpp>
#include <RouteInfo.hpp>
#include <TrafficLightStatus.hpp>
#include <PedestrianStatus.hpp>

#include <improc.h>

#include "TrafficLightDetector/TrafficLightDetector.h"
#include "PedestrianDetector/PedestrianDetector.h"

namespace fad {
    class ConstraintGenerator {
    public:
        enum class Type {
            TRAFFIC_LIGHT_IS_RED,
            PEDESTRIAN_DETECTED,
            OBSTACLE_DETECTED,
        };

        ConstraintGenerator();
        ~ConstraintGenerator();

        ConstraintGenerator(const ConstraintGenerator& obj) = delete;
        ConstraintGenerator &operator=(const ConstraintGenerator& ConstraintGenerator) = delete;

        /**
         *  @brief 保持している認識結果・制約を初期化する
         */
        void init();

        /**
         *  @brief 外界の状態を表すフラグの集合を取得する
         */
        const std::set<Type>& getState() const;

        /**
         *  @brief 生成した制約を取得する
         */
        const core::RouteInfo& getRouteInfo() const;

        /**
         *  @brief 信号認識の結果を取得する
         */
        const core::TrafficLightStatus& getTLDetectionResult() const;

        /**
         *  @brief 歩行者認識の結果を取得する
         */
        const core::PedestrianStatus& getPedestrianDetectionResult() const;

        /**
         *  @brief 信号認識を実行する
         *  @param (color_img)      Webカメラから取得した前方画像
         *  @param (distance_to_tl) 現状態から信号までの距離
         *  @return 認識された場合、trueを返す
         */
        bool detectTrafficLights(const cv::Mat& color_img,
                                 const double&  distance_to_tl = -1);

        /**
         *  @brief 歩行者認識を実行する
         *  @param (color_img) Webカメラから取得した前方画像
         *  @return 認識された場合、trueを返す
         */
        bool detectPedestrians(const cv::Mat&            color_img,
                               const core::VehicleState& current_state);

        /**
         *  @brief 障害物認識を実行する
         *  @param (bird_eye_sat_img) 路面の俯瞰画像を彩度二値化した画像
         *  @return 認識された場合、trueを返す
         */
        bool detectObstacles(const core::EnvironmentMap<uint8_t>& bird_eye_sat_img,
                             const core::VehicleState&            current_state);

        /**
         *  @brief 制約に外界の状態を付与する
         *  @param (&route_info) 制約情報(上書きする)
         */
        void generate(const core::RouteInfo& route_info);

    private:
        const std::string PARAM_YAML_PATH       = "/data/ConstraintGenerator/param.yaml";
        const std::string MOTOR_PARAM_YAML_PATH = "/data/Common/motor.yaml";


        class ConstraintGeneratorParam : public core::YAMLHelper::ParamBase {
        public:
            double escape_radius;
            double target_velocity_lower;
            bool   enable_object_detection;
            bool   enable_traffic_light_detection;
            bool   enable_pedestrian_detection;

            void read(const cv::FileNode& node) override {
                escape_radius                  = (double)node["escape_radius"];
                target_velocity_lower          = (double)node["target_velocity_lower"];
                enable_object_detection        = (int)node["enable_object_detection"];
                enable_traffic_light_detection = (int)node["enable_traffic_light_detection"];
                enable_pedestrian_detection    = (int)node["enable_pedestrian_detection"];
            }
        };

        class TrafficLightParam : public core::YAMLHelper::ParamBase {
        public:
            double   limit_time_of_red_light;
            double   limit_time_of_rediscovery;
            uint32_t detection_results_queue_depth;
            uint32_t nof_detection_for_continuous;
            double   distance_thr_for_continuous;
            double   velocity_ratio_when_stop_line;

            void read(const cv::FileNode& node) override {
                limit_time_of_red_light       = (double)node["limit_time_of_red_light"];
                limit_time_of_rediscovery     = (double)node["limit_time_of_rediscovery"];
                detection_results_queue_depth = (int)node["detection_results_queue_depth"];
                nof_detection_for_continuous  = (int)node["nof_detection_for_continuous"];
                distance_thr_for_continuous   = (double)node["distance_thr_for_continuous"];
                velocity_ratio_when_stop_line = (double)node["velocity_ratio_when_stop_line"];
            }
        };

        class ObstacleParam : public core::YAMLHelper::ParamBase {
        public:
            uint32_t nof_closing;
            uint32_t nof_opening;
            double   area_lower;
            double   area_upper;
            double   entarable_radius;
            double   valid_time_s;
            double   occupation_radius;
            double   velocity_ratio_when_detect_obs;

            void read(const cv::FileNode& node) override {
                nof_closing       = (int)node["nof_closing"];
                nof_opening       = (int)node["nof_opening"];
                area_lower        = (double)node["area_lower"];
                area_upper        = (double)node["area_upper"];
                entarable_radius  = (double)node["entarable_radius"];
                valid_time_s      = (double)node["valid_time_s"];
                occupation_radius = (double)node["occupation_radius"];
                velocity_ratio_when_detect_obs = (double)node["velocity_ratio_when_detect_obs"];
            }
        };

        struct RealSpaceObject {
            using scs = std::chrono::system_clock;
            cv::Point2d     pt;   // 全体座標系におけるオブジェクトの位置
            double          size; // 大きさ
            scs::time_point time; // 検出時間
            RealSpaceObject(const cv::Point2d&     _pt   = cv::Point2d(),
                            const double&          _size = 0.0,
                            const scs::time_point& _time = scs::now()) :
                pt(_pt), size(_size), time(_time) { }
        };

        std::unique_ptr<TrafficLightDetector> tl_detector_;
        std::unique_ptr<PedestrianDetector>   pedestrian_detector_;

        ConstraintGeneratorParam param_;
        TrafficLightParam        tl_param_;
        ObstacleParam            obstacle_param_;

        // 車体の大きさ
        double vehicle_size_;

        // 生成した制約
        core::RouteInfo route_info_;

        // 信号認識の結果の系列を格納するリングバッファ
        std::vector<TrafficLightDetector::TLDetectInfo> tl_detection_results_queue_;
        size_t tl_detection_results_index_;

        // 信号認識の結果から判断された信号の状態
        core::TrafficLightStatus tl_current_status_, tl_prev_status_;

        // 信号によって停止した時間・発進した時間
        std::chrono::system_clock::time_point time_of_stop_by_tl_, time_of_dept_by_tl_;

        // 歩行者の検出結果
        core::PedestrianStatus pedestrian_;

        // 全体座標系におけるオブジェクトの位置
        std::vector<RealSpaceObject> obstacles_;

        // 外界の状態を表すフラグの集合
        std::set<Type> state_;

        /**
         *  @brief 信号認識の結果から信号の状態を判断する
         */
        core::TrafficLightStatus judgeTLStatus(const core::RouteInfo& route_info) const;
    };
}

#endif /* FAD_SRC_FAD_CONSTRAINTGENERATOR_CONSTRAINTGENERATOR_H_ */
