/**
 *  Cmod: Cmod側と同期しているレジスタにアクセスするクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#include <cmod/cmod.h>

namespace zynqpl {
    Cmod::Cmod(const std::string& gpio1_devname,
               const std::string& gpio2_devname) :
        reg_accessor_(std::make_unique<RegAccessor>(gpio1_devname, gpio2_devname)) {
    }

    Cmod::~Cmod() {
    }

    void Cmod::setAngulerVelocity(const double& anguler_velocity, const motor::ID& id) const {
        uint32_t data = 0;

        // 8ビット左シフトして書き込み
        data |= static_cast<int>(anguler_velocity * 256);
        reg_accessor_->write(motor::ADDR_ROTATION_VELOCITY[static_cast<int>(id)], data);
    }

    void Cmod::setPIDParameter(const double& gain, const motor::ID& id, const motor::Gain& type) const {
        uint32_t data = 0;

        // 8ビット左シフトして書き込み
        data |= static_cast<int>(gain * 256);
        reg_accessor_->write(motor::ADDR_PWM_GAIN[static_cast<int>(type)][static_cast<int>(id)], data);
    }

    int Cmod::getRotCount(const motor::ID& id) const {
        return static_cast<int>(reg_accessor_->read(motor::ADDR_ROT_CNT[static_cast<int>(id)]));
    }

    bool Cmod::getCmodBTN1Status() const {
        return reg_accessor_->read(motor::ADDR_CMOD_BTN) & motor::CMOD_BTN1_MASK;
    }

    bool Cmod::getCmodBTN2Status() const {
        return reg_accessor_->read(motor::ADDR_CMOD_BTN) & motor::CMOD_BTN2_MASK;
    }

    void Cmod::initRegs() const {
        for(int i = 0; i < 256; i++) {
            reg_accessor_->write(i, 0x00);
        }
    }
}

