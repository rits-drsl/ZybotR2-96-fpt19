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

#ifndef FAD_SRC_FAD_ODOMETRYCALCULATOR_IOCALCULATOR_IOCALCULATOR_H_
#define FAD_SRC_FAD_ODOMETRYCALCULATOR_IOCALCULATOR_IOCALCULATOR_H_

#include <numeric>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <opencv2/core/eigen.hpp>

#include <YAMLHelper.hpp>
#include <InertialOdometry.hpp>
#include <IMUQuat.hpp>

namespace fad {
    class IOCalculator {
    public:
        explicit IOCalculator();
        ~IOCalculator();

        IOCalculator(const IOCalculator& obj) = delete;
        IOCalculator &operator=(const IOCalculator& obj) = delete;

        IOCalculator(IOCalculator&&) = default;

        void init(const core::IMUQuat& current_quat);

        core::InertialOdometry get(const core::Theta& current_theta,
                                   const bool& reset = true);

        void update(const core::IMUQuat& current_quat,
                    const double&        absolute_scale);

        double calcYaw(const core::IMUQuat& quat) const;
    private:
        core::InertialOdometry io_;
        double prev_yaw_;
    };
}

#endif /* FAD_SRC_FAD_ODOMETRYCALCULATOR_IOCALCULATOR_IOCALCULATOR_H_ */
