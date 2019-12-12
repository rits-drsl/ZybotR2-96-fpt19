/**
 *  PurePursuit: PurePursuitによる経路追従を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Yuta Ishida.
 *  Authors:
 *      Yuya Kudo   <ri0049ee@ed.ritsumei.ac.jp>
 *      Yuta Ishida <ri0066fs@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_PATHTRACER_TRACER_PUREPURSUIT_PUREPURSUIT_H_
#define FAD_SRC_FAD_PATHTRACER_TRACER_PUREPURSUIT_PUREPURSUIT_H_

#include <memory>

#include <control.h>

#include <YAMLHelper.hpp>

#include "../TracerBase.h"

namespace fad {
    class PurePursuit : public base::TracerBase {
    public:
        PurePursuit();
        ~PurePursuit();

        void calc() override;

    private:
        const std::string PUREPURSUIT_PARAM_YAML_PATH = "/data/PathTracer/Tracer/PurePursuit/param.yaml";
        const std::string MOTOR_PARAM_YAML_PATH       = "/data/Common/motor.yaml";

        class PurePursuitParam : public core::YAMLHelper::ParamBase {
        public:
            double norm_thr;

            void read(const cv::FileNode& node) override {
                norm_thr = (double)node["norm_thr"];
            }
        };

        class PIDParam : public core::YAMLHelper::ParamBase {
        public:
            double P_gain;
            double I_gain;
            double D_gain;

            void read(const cv::FileNode& node) override {
                P_gain = (double)node["P_gain"];
                I_gain = (double)node["I_gain"];
                D_gain = (double)node["D_gain"];
            }
        };

        class MotorParam : public core::YAMLHelper::ParamBase {
        public:
            double one_rotation_pulse;
            double one_rotation_dist;
            double tire_tread;

            void read(const cv::FileNode& node) override {
                one_rotation_pulse = (double)node["one_rotation_pulse"];
                one_rotation_dist  = (double)node["one_rotation_dist"];
                tire_tread         = (double)node["tire_tread"];
            }
        };

        PurePursuitParam param_;
        MotorParam       m_param_;
        std::unique_ptr<control::PID> pid_ctrl_;
    };
}

#endif /* FAD_SRC_FAD_PATHTRACER_TRACER_PUREPURSUIT_PUREPURSUIT_H_ */
