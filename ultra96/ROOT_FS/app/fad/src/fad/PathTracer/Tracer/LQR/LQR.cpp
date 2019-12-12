/**
 *  LQR: LQRを用いて与えられたパスを追従するための角速度を導出するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada
 *  Copyright (C) 2019 Yuta Ishida
 *.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *      Ishida Yuta    <ri0066fs@ed.ritsumei.ac.jp>
 *
 */

#include "LQR.h"

namespace fad {
    LQR::LQR() :
        Q_(Eigen::MatrixXd::Zero(5,5)),
        R_(Eigen::MatrixXd::Zero(2,2)),
        K_(Eigen::MatrixXd::Zero(5,5)),
        U_(Eigen::MatrixXd::Zero(2,1))
    {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        cv::Mat lqr_param_Q;
        cv::Mat lqr_param_R;

        core::YAMLHelper::readStruct(root_path + LQR_PARAM_YAML_PATH, lqr_param_, "param");
        core::YAMLHelper::readStruct(root_path + MOTOR_PARAM_YAML_PATH, motor_param_, "Motor");
        core::YAMLHelper::read(root_path + LQR_PARAM_YAML_PATH, lqr_param_Q, "Q");
        core::YAMLHelper::read(root_path + LQR_PARAM_YAML_PATH, lqr_param_R, "R");

        cv::cv2eigen(lqr_param_Q, Q_);
        cv::cv2eigen(lqr_param_R, R_);

        wheel_diameter = motor_param_.one_rotation_dist / core::PI / 2;
        update_state = 0;
    }

    LQR::~LQR() { };

    void LQR::calc() {
        // TracerBase::setCurrentState・TracerBase::setCurrentStateによって
        // 外部から現状態・目標状態が設定された際に内部状態の初期化を行う
        if(external_target_state_ != target_state_) {
            external_target_state_ = target_state_;
        }
        if(external_current_state_ != current_state_) {
            external_current_state_ = current_state_;
            internal_current_state_ = current_state_;
            current_distance_       = 0;
            current_angle_distance_.isZero();
            prev_distance_          = 0;
            prev_angle_difrence_.isZero();
            prev_time_              = std::chrono::system_clock::now();
        }
        LQR::updateInternalState();

        LQR::calcTargetDifferent();

        LQR::calcRiccatiSolve();

        LQR::inputRestriction();

        LQR::updateInternalState();

        LQR::convertAngleVelocity();
    };

    // private
    void LQR::calcTargetDifferent(){
        delta_velocity_         = internal_current_state_.v - target_state_.v;
        current_distance_       = internal_current_state_.distanceFrom(target_state_);
        current_angle_distance_ = internal_current_state_.t - target_state_.t;
        if(current_angle_distance_.get() < 0) {
            current_distance_ *= -1;
        }
    };

    void LQR::calcRiccatiSolve() {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(5,5);
        A(0,0) = 1.0;
        A(0,1) = timestep_;
        A(1,2) = internal_current_state_.v ;
        A(2,2) = 1.0;
        A(2,3) = timestep_;
        A(4,4) = 1.0;

        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(5,2);
        B(3.0) = 1;
        B(4,1) = timestep_;

        Eigen::MatrixXd X(5,1);
        X(0,0) = current_distance_;
        X(1,0) = (current_distance_ - prev_distance_) / timestep_;
        X(2,0) = current_angle_distance_.get();
        X(3,0) = (current_angle_distance_ - prev_angle_difrence_).get() / timestep_;
        X(4,0) = delta_velocity_;

        Eigen::MatrixXd Z;

        auto A_trans = A.transpose();
        auto B_trans = B.transpose();
        auto E_5_5 = Eigen::MatrixXd::Identity(5,5);


        int n = (int)A.rows();
        // Hamilton Matrix
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> COD_Atrans(A_trans);
        auto A_trans_inver_Q = COD_Atrans.solve(Q_);
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> COD_A(A);
        auto A_inver_E = COD_A.solve(E_5_5);
        Eigen::MatrixXd Ham(2 * n, 2 * n);

        Ham << A + B * R_.inverse() * B.transpose() * A_trans_inver_Q, -B * R_.inverse() * B.transpose() * A_inver_E.transpose(), -A_trans_inver_Q, A_inver_E;

        // EigenVec, Value
        Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);
        if (Eigs.info() != Eigen::Success) abort();

        // eigenvector storage
        Eigen::MatrixXcd eigvec(2*n, n);
        int j = 0;

        // store those with negative real number
        for(int i = 0; i < 2*n; ++i){
            if(Eigs.eigenvalues()[i].real() < 0){
                eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2*n, 1);
                ++j;
            }
        }

        Eigen::MatrixXcd U(n, n);
        Eigen::MatrixXcd V(n, n);

        U = eigvec.block(0,0,n,n);
        V = eigvec.block(n,0,n,n);

        Z = (V * U.inverse()).real();

        Eigen::PartialPivLU<Eigen::MatrixXd> lu(R_ + (B_trans * Z * B));
        U_ = -lu.solve(B_trans * Z * A) * X;
    };

    void LQR::convertAngleVelocity() {
        av_.r = ((internal_current_state_.v) + (motor_param_.tire_tread * U_(0,0))) / wheel_diameter;
        av_.l = ((internal_current_state_.v) - (motor_param_.tire_tread * U_(0,0))) / wheel_diameter;
    };

    void LQR::updateInternalState() {
        auto current_time = std::chrono::system_clock::now();
        timestep_  = std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - prev_time_).count() / 1e9;
        // if(update_state == 0) {
        prev_time_ = current_time;
        // update_state = 1;
        // }
        // else {
            // update_state = 0;
        // }

        prev_distance_       = current_distance_;
        prev_angle_difrence_ = current_angle_distance_;

        internal_current_state_.x += internal_current_state_.v * std::cos(internal_current_state_.t.get()) * timestep_;
        internal_current_state_.y += internal_current_state_.v * std::sin(internal_current_state_.t.get()) * timestep_;
        internal_current_state_.t += core::Theta(std::tan(U_(0,0)) * timestep_);
        internal_current_state_.v += U_(1,0) * timestep_;
    };

    void LQR::inputRestriction() {
        auto abs_angle_thr = core::PI * lqr_param_.angle_Restriction_threshold / 180;
        U_(0,0) = core::Util::clamp(U_(0,0), -abs_angle_thr, abs_angle_thr);

        auto abs_velocity_thr = lqr_param_.velocity_Restriction_threshold;
        U_(1,0) = core::Util::clamp(U_(1,0), -abs_velocity_thr, abs_velocity_thr);

        if((internal_current_state_.v >= 0.000 && internal_current_state_.v <= 0.020 && U_(1,0) > 0.0) ||
           (internal_current_state_.v <= 0.000 && internal_current_state_.v >= -0.020 && U_(1,0) < 0.0)) {
            internal_current_state_.v *= 4.0;
        }
    }
}
