/**
 *  IOCalculator: Inertial Odometryを導出するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "IOCalculator.h"

namespace fad {
    IOCalculator::IOCalculator() {
    }

    IOCalculator::~IOCalculator() {
    }

    void IOCalculator::init(const core::IMUQuat& current_quat) {
        prev_yaw_= calcYaw(current_quat);
    }

    core::InertialOdometry IOCalculator::get(const core::Theta& current_theta,
                                             const bool& reset) {
        auto ret_io = io_;
        if(reset) {
            io_ = core::InertialOdometry();
        }

        // 全体座標系へ変換
        const auto norm_w_odm = ret_io.norm();
        const auto t          = core::Theta(std::atan2(ret_io.y, ret_io.x));
        ret_io.x = norm_w_odm * std::cos((current_theta + t).get());
        ret_io.y = norm_w_odm * std::sin((current_theta + t).get());

        return ret_io;
    }

    void IOCalculator::update(const core::IMUQuat& current_quat,
                              const double&  absolute_scale) {
        const auto yaw       = calcYaw(current_quat);
        const auto delta_yaw = core::Theta(yaw - prev_yaw_) * -1;
        io_.x += absolute_scale * std::cos(delta_yaw.get());
        io_.y += absolute_scale * std::sin(delta_yaw.get());
        io_.t += delta_yaw;
        prev_yaw_ = yaw;
    }

    double IOCalculator::calcYaw(const core::IMUQuat& quat) const {
        // クォータニオンを回転行列に変換し、回転行列からオイラー角を取得する
        const auto R = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z).toRotationMatrix();
        Eigen::Vector3d euler_angles = R.eulerAngles(2, 1, 0);
        return euler_angles(2);
    }
}
