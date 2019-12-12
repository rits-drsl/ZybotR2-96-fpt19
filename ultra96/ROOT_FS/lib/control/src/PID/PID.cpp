/**
 *  PID: PID制御を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 *
 */

#include <PID/PID.h>

namespace control {
    PID::PID(const Gain& gain,
             const double& target_val) :
        gain_(gain),
        target_val_(target_val),
        out_val_(0),
        prev_error_(0),
        i_error_(0),
        prev_error_is_valid_(false) {
    }

    PID::PID(const double& P_gain,
             const double& I_gain,
             const double& D_gain,
             const double& target_val) :
        gain_({P_gain, I_gain, D_gain}),
        target_val_(target_val),
        out_val_(0),
        prev_error_(0),
        i_error_(0),
        prev_error_is_valid_(false) {
    }

    PID::~PID() {
    }

    void PID::setTargetValue(const double& val) {
        target_val_ = val;
    }

    void PID::setGain(const Gain& gain) {
        gain_ = gain;
    }

    void PID::setGain(const double& P_gain,
                      const double& I_gain,
                      const double& D_gain) {
        gain_ = {P_gain, I_gain, D_gain};
    }

    double PID::getOutValue() const {
        return out_val_;
    }

    void PID::init() {
        out_val_             = 0;
        prev_error_          = 0;
        i_error_             = 0;
        prev_error_is_valid_ = false;
    }

    void PID::updete(const double& val) {
        auto error  = target_val_ - val;
        i_error_   += (error + prev_error_) / 2;

        auto p_term = gain_.P * error;
        auto i_term = gain_.I * i_error_;
        auto d_term = prev_error_is_valid_ ? gain_.D * (error - prev_error_) : 0;

        out_val_ += p_term + i_term + d_term;

        prev_error_          = error;
        prev_error_is_valid_ = true;
    }
}
