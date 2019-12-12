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

#include "CmodController.h"

namespace fad {
    CmodController::CmodController() {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        DevNameParam devname_param;
        core::YAMLHelper::readStruct(root_path + HW_PARAM_YAML_PATH, devname_param, "devname");

        cmod_ = std::make_unique<zynqpl::Cmod>(devname_param.gpio1,
                                               devname_param.gpio2);

        // レジスタを初期化
        initCmod();

        // PIDゲインを設定する
        PIDParam pid_param;
        core::YAMLHelper::readStruct(root_path + HW_PARAM_YAML_PATH, pid_param, "Motor", "PID");
        cmod_->setPIDParameter(pid_param.P_gain, zynqpl::motor::ID::RIGHT, zynqpl::motor::Gain::PROPORTION);
        cmod_->setPIDParameter(pid_param.P_gain, zynqpl::motor::ID::LEFT,  zynqpl::motor::Gain::PROPORTION);
        cmod_->setPIDParameter(pid_param.I_gain, zynqpl::motor::ID::RIGHT, zynqpl::motor::Gain::INTEGRATION);
        cmod_->setPIDParameter(pid_param.I_gain, zynqpl::motor::ID::LEFT,  zynqpl::motor::Gain::INTEGRATION);
        cmod_->setPIDParameter(pid_param.D_gain, zynqpl::motor::ID::RIGHT, zynqpl::motor::Gain::DERIVATIVE);
        cmod_->setPIDParameter(pid_param.D_gain, zynqpl::motor::ID::LEFT,  zynqpl::motor::Gain::DERIVATIVE);
    }

    CmodController::~CmodController() {
        setWheelAngularVelocity(0.0, 0.0);
    }

    void CmodController::initCmod() {
        cmod_->initRegs();
    }

    void CmodController::setWheelAngularVelocity(const double& r, const double& l) {
        cmod_->setAngulerVelocity(r, zynqpl::motor::ID::RIGHT);
        cmod_->setAngulerVelocity(l, zynqpl::motor::ID::LEFT);
    }

    CmodController::MotorCnt CmodController::getCntOfRotaryEncoder() {
        return MotorCnt{cmod_->getRotCount(zynqpl::motor::ID::RIGHT), cmod_->getRotCount(zynqpl::motor::ID::LEFT)};
    }

    bool CmodController::BTNIsPushed(const BTN& id) {
        bool is_pushed;

        switch(id) {
            case BTN::CMOD1: {
                is_pushed = cmod_->getCmodBTN1Status();
                break;
            }
            case BTN::CMOD2: {
                is_pushed = cmod_->getCmodBTN2Status();
                break;
            }
            default: {
                is_pushed = false;
            }
        }

        return is_pushed;
    }
}

