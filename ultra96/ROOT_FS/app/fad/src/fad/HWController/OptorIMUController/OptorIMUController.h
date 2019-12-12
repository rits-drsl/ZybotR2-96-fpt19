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

#ifndef FAD_SRC_FAD_HWCONTROLLER_OPTORIMUCONTROLLER_OPTORIMUCONTROLLER_H_
#define FAD_SRC_FAD_HWCONTROLLER_OPTORIMUCONTROLLER_OPTORIMUCONTROLLER_H_

#include <thread>

#include <YAMLHelper.hpp>
#include <IMUQuat.hpp>

#include <improc.h>
#include <optorusb.h>
#include <optorimu.h>
#include <optorcam.h>

namespace fad {
    class OptorIMUController {
    public:
        struct IMU {
            struct Gyr {
                double x;
                double y;
                double z;
            };
            struct Acc {
                double x;
                double y;
                double z;
            };
        };

        OptorIMUController();
        ~OptorIMUController();

        OptorIMUController(const OptorIMUController& obj) = delete;
        OptorIMUController &operator=(const OptorIMUController& obj) = delete;

        IMU::Gyr getGyr() const;
        IMU::Acc getAcc() const;
        core::IMUQuat getQuat() const;
        double getUpdateTime() const;
        void update();

    private:
        const std::string HW_PARAM_YAML_PATH = "/data/HWController/param.yaml";

        visensor_imudata imudata_;
    };
}

#endif  /* FAD_SRC_FAD_HWCONTROLLER_OPTORIMUCONTROLLER_OPTORIMUCONTROLLER_H_ */
