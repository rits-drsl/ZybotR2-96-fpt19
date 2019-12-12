/**
 *  PathTracer: 与えられたパスを追従する角速度を導出するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Yuta Ishida.
 *  Authors:
 *      Yuya Kudo   <ri0049ee@ed.ritsumei.ac.jp>
 *      Yuta Ishida <ri0066fs@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_PATHTRACER_PATHTRACER_H_
#define FAD_SRC_FAD_PATHTRACER_PATHTRACER_H_

#include <chrono>

#include <VehicleState.hpp>
#include <YAMLHelper.hpp>
#include <Util.hpp>

#include <control.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Tracer/PurePursuit/PurePursuit.h"
#include "Tracer/SelfLocalizedPurePursuit/SelfLocalizedPurePursuit.h"
#include "Tracer/LQR/LQR.h"

namespace fad {
    class PathTracer {
    public:
        PathTracer();
        ~PathTracer();

        PathTracer(const PathTracer& obj) = delete;
        PathTracer &operator=(const PathTracer& obj) = delete;

        /**
         *  @brief 経路を設定する
         *  @param (path) 全体座標系における座標で構成されたパス
         */
        void setPath(const std::vector<core::VehicleState>& path);

        /**
         *  @brief 現在状態を設定する
         */
        void setCurrentState(const core::VehicleState& current_state);

        /**
         *  @brief 現在設定されている目標状態を取得する
         */
        const core::VehicleState& getRefState() const;

        /**
         *  @brief 出力すべき角速度を導出する
         */
        core::AngularVelocity calcAngularVelocity();

    private:
        const std::string TRACER_PARAM_YAML_PATH = "/data/PathTracer/param.yaml";
        const std::string MOTOR_PARAM_YAML_PATH  = "/data/Common/motor.yaml";

        class TracerParam : public core::YAMLHelper::ParamBase {
        public:
            double distance_current_to_ref_lower;
            double distance_current_to_ref_upper;
            double lower_velocity;
            double upper_velocity;
            std::string trace_method;

            void read(const cv::FileNode& node) override {
                distance_current_to_ref_lower = (double)node["distance_current_to_ref_lower"];
                distance_current_to_ref_upper = (double)node["distance_current_to_ref_upper"];
                lower_velocity                = (double)node["lower_velocity"];
                upper_velocity                = (double)node["upper_velocity"];
                trace_method                  = (std::string)node["trace_method"];
            }
        };

        TracerParam param_;

        std::vector<core::VehicleState> path_;
        uint32_t                        path_ref_index_;

        core::VehicleState current_state_, ref_state_;
        std::unique_ptr<base::TracerBase> tracer_;
    };
}

#endif /* FAD_SRC_FAD_PATHTRACER_PATHTRACER_H_ */
