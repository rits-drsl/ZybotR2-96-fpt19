/**
 *  OptorIMUController: OptorのIMUからデータを取得するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "OptorIMUController.h"

namespace fad {
    OptorIMUController::OptorIMUController() {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        if(std::getenv("ROBOT_NAME") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export ROBOT_NAME=<name of your robot> ");
        }
        const auto robot_name = std::getenv("ROBOT_NAME");

        std::string setting_file;
        core::YAMLHelper::read(root_path + HW_PARAM_YAML_PATH, setting_file, "Optor", "setting_file", robot_name);
        visensor_load_settings((root_path + setting_file).c_str());

        if(visensor_Start_IMU() < 0) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Could not start optor imu");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        update();
        auto quat = getQuat();
        if(quat.x == 0 && quat.w == 0 && quat.w == 0 && quat.w == 0) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                     "Failed to initialize Opter IMU");
        }
    }

    OptorIMUController::~OptorIMUController() {
        visensor_Close_IMU();
    }

    OptorIMUController::IMU::Gyr OptorIMUController::getGyr() const {
        return IMU::Gyr{ imudata_.rx, imudata_.ry, imudata_.rz };
    }

    OptorIMUController::IMU::Acc OptorIMUController::getAcc() const {
        return IMU::Acc{ imudata_.ax, imudata_.ay, imudata_.az };
    }

    core::IMUQuat OptorIMUController::getQuat() const {
        return core::IMUQuat(imudata_.qx, imudata_.qy, imudata_.qz, imudata_.qw);
    }

    double OptorIMUController::getUpdateTime() const {
        return imudata_.timestamp;
    }

    void OptorIMUController::update() {
        if(visensor_imu_have_fresh_data()) {
            visensor_get_imudata_latest(&imudata_);
        }
    }
}

