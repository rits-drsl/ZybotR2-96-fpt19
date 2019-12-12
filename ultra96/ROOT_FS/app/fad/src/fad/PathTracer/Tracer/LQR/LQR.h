/**
 *  LQR: LQRを用いて与えられたパスを追従するための角速度を導出するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Copyright (M) 2019 Yuta Ishida
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *      Yuta Ishida    <ri0066fs@ed.ritsumei.ac.jp>
 */

#ifndef FAD_SRC_FAD_PATHTRACER_TRACER_LQR_LQR_H_
#define FAD_SRC_FAD_PATHTRACER_TRACER_LQR_LQR_H_

#include <chrono>
#include <cmath>
#include <cstdlib>

#include <VehicleState.hpp>
#include <YAMLHelper.hpp>
#include <Util.hpp>

#include <control.h>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>


#include "../TracerBase.h"

namespace fad {
    class LQR : public
    base::TracerBase {
    public:
        LQR();
        ~LQR();

        void calc() override;

    private:
        const std::string LQR_PARAM_YAML_PATH   = "/data/PathTracer/Tracer/LQR/param.yaml";
        const std::string MOTOR_PARAM_YAML_PATH = "/data/Common/motor.yaml";

        class LQRParam : public core::YAMLHelper::ParamBase{
        public:
            double riccati_threshold;
            int    riccati_roop_threshold;
            double angle_Restriction_threshold;
            double velocity_Restriction_threshold;

            void read(const cv::FileNode& node) override {
                riccati_threshold              = (double)node["riccati_threshold"];
                riccati_roop_threshold         = (double)node["riccati_roop_threshold"];
                angle_Restriction_threshold    = (double)node["angle_Restriction_threshold"];
                velocity_Restriction_threshold = (double)node["velocity_Restriction_threshold"];
            }
        };

        class MotorParam : public core::YAMLHelper::ParamBase {
        public:
            double tire_tread;
            double one_rotation_dist;

            void read(const cv::FileNode& node) override {
                tire_tread = (double)node["tire_tread"];
                one_rotation_dist = (double)node["one_rotation_dist"];
            }
        };

        LQRParam lqr_param_;
        MotorParam motor_param_;

        double timestep_;

        double wheel_diameter;

        core::VehicleState external_target_state_, external_current_state_, internal_current_state_;
        double delta_velocity_;
        double current_distance_;
        double prev_distance_;

        core::Theta current_angle_distance_;
        core::Theta prev_angle_difrence_;

        Eigen::MatrixXd Q_, R_, K_, U_;

        std::chrono::system_clock::time_point prev_time_;

        int update_state;

        /**
         * 処理の間隔を計算。
         * 目標座標と現在座標の差分を取る。
         * 目標と現在点の距離と目標と現在角度の角度差を計算する。
         */
        void calcTargetDifferent();

        /**
         * 最適な入力を得るために必要な変数Uをリカッチ方程式で計算する。
         */
        void calcRiccatiSolve();

        /**
         * 算出した入力速度、角度を左右輪の角速度に変換する。
         */
        void convertAngleVelocity();

        /**
         * 現在stateを更新する。
         */
        void updateInternalState();

        /**
         * 入力の上限そ設定する。
         */
        void inputRestriction();
    };
};
#endif /* FAD_SRC_FAD_PATHTRACER_TRACER_LQR_LQR_H_ */
