/**
 *  PSIICController: PS IIC制御クラス
 *                   for 16bit memory address
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#include <pcam/ps/PSIIC.h>

namespace zynqpl {
    PSIIC::PSIIC(const std::string& iic_devname, uint8_t slave_addr) {
        // deviceのopen
        iic_fd_ = open(("/dev/" + iic_devname).c_str(), O_RDWR);
        if(!iic_fd_) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not open device of iic : " + std::string(strerror(errno)));
        }

        // IIC SLAVE ADDRの設定
        if(ioctl(iic_fd_, I2C_SLAVE, (slave_addr >> 1)) < 0) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not set iic slave addr : " + std::string(strerror(errno)));
        }
    }

    PSIIC::~PSIIC() {
        close(iic_fd_);
    }

    uint8_t PSIIC::iicRead(uint16_t addr) const {
        uint8_t write_buf[2] = {(uint8_t)((addr & 0xFF00) >> 8), (uint8_t)(addr & 0x00FF)};
        uint8_t read_buf[1];

        if(!write(iic_fd_, write_buf, 2)) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not write addr to iic device : " + std::string(strerror(errno)));
        }
        if(!read(iic_fd_, read_buf, 1)) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not read data from iic device : " + std::string(strerror(errno)));
        }
        return read_buf[0];
    }

    void PSIIC::iicWrite(uint16_t addr, uint8_t data) const {
        uint8_t write_buf[3] = {(uint8_t)((addr & 0xFF00) >> 8), (uint8_t)(addr & 0x00FF), data};
        if(!write(iic_fd_, write_buf, 3)) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not write addr to iic device : " + std::string(strerror(errno)));
        }
    }
}
