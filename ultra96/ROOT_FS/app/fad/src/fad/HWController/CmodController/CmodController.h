/**
 *  CmodController: Cmodの制御を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_HWCONTROLLER_HWCONTROLLER_H_
#define FAD_SRC_FAD_HWCONTROLLER_HWCONTROLLER_H_

#include <WheelOdometry.hpp>
#include <YAMLHelper.hpp>

#include <zynqpl.h>

namespace fad {
    class CmodController {
    public:
        enum class BTN {
            CMOD1,
            CMOD2
        };

        struct MotorCnt {
            int right;
            int left;
        };

        CmodController();
        ~CmodController();

        CmodController(const CmodController& obj) = delete;
        CmodController &operator=(const CmodController& obj) = delete;

        /**
         *  @brief モータの角速度を設定する
         *  @param (l) 左のモータの角速度
         *  @param (r) 右のモータの角速度
         */
        void setWheelAngularVelocity(const double& r, const double& l);

        /**
         *  @brief 左右のモータに付属しているロータリーエンコーダのカウンタ値を取得する
         */
        MotorCnt getCntOfRotaryEncoder();

        /**
         *  @brief BTNの状態を取得する
         */
        bool BTNIsPushed(const BTN& id);

    private:
        const std::string HW_PARAM_YAML_PATH    = "/data/HWController/param.yaml";
        const std::string MOTOR_PARAM_YAML_PATH = "/data/Common/motor.yaml";

        class DevNameParam : public core::YAMLHelper::ParamBase {
        public:
            std::string gpio1;
            std::string gpio2;

            void read(const cv::FileNode& node) override {
                gpio1     = (std::string)node["gpio1"];
                gpio2     = (std::string)node["gpio2"];
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

        std::unique_ptr<zynqpl::Cmod> cmod_;

        bool                do_calc_w_odm_;
        double              w_odm_update_period_us_;
        core::WheelOdometry w_odm_;

        /**
         *  @brief Cmodと同期しているレジスタを初期化する
         */
        void initCmod();
    };
}

#endif  /* FAD_SRC_FAD_HWCONTROLLER_HWCONTROLLER_H_ */
