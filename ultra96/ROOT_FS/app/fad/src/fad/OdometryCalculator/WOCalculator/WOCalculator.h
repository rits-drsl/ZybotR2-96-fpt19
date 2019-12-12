/**
 *  WOCalculator: WheelOdometryを導出するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_ODOMETRYCALCULATOR_WOCALCULATOR_WOCALCULATOR_H_
#define FAD_SRC_FAD_ODOMETRYCALCULATOR_WOCALCULATOR_WOCALCULATOR_H_

#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <YAMLHelper.hpp>
#include <VehicleState.hpp>
#include <WheelOdometry.hpp>

namespace fad {
    class WOCalculator {
    public:
        WOCalculator();
        ~WOCalculator();

        WOCalculator(const WOCalculator& obj) = delete;
        WOCalculator &operator=(const WOCalculator& obj) = delete;

        WOCalculator(WOCalculator&&) = default;

        void init(const int& r_cnt,
                  const int& l_cnt);

        core::WheelOdometry get(const core::Theta& current_theta,
                                const bool reset = true);

        const core::WheelOdometry& getTotal(const core::Theta& current_theta) const;

        void update(const int& r_cnt,
                    const int& l_cnt);

    private:
        const std::string WO_PARAM_YAML_PATH    = "/data/OdometryCalculator/WOCalculator/param.yaml";
        const std::string MOTOR_PARAM_YAML_PATH = "/data/Common/motor.yaml";

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

        core::WheelOdometry wo_;
        core::WheelOdometry total_wo_;

        MotorParam m_param_;
        uint32_t   LFQueueDepth_;
        double     rot_dist_par_pulse_;

        std::vector<double> r_dist_diff_queue_, l_dist_diff_queue_;
        uint32_t            ref_index_cnt_;
        double              prev_r_dist_;
        double              prev_l_dist_;
    };
}

#endif /* FAD_SRC_FAD_ODOMETRYCALCULATOR_WOCALCULATOR_WOCALCULATOR_H_ */

