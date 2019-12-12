/**
 *  RegAccessor: レジスタ同期システムのレジスタにアクセスするクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#include <cmod/reg/RegAccessor.h>

namespace zynqpl {
    RegAccessor::RegAccessor(const std::string& gpio1_devname,
                             const std::string& gpio2_devname) {
        // deviceのopen
        gpio1_fd_ = open(("/dev/" + gpio1_devname).c_str(), O_RDWR | O_SYNC);
        if(!gpio1_fd_) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "Could not open device : " + std::string(strerror(errno)));
        }

        gpio2_fd_ = open(("/dev/" + gpio2_devname).c_str(), O_RDWR | O_SYNC);
        if(!gpio2_fd_) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "Could not open device : " + std::string(strerror(errno)));
        }

        // baseaddrの取得
        gpio1_baseaddr_ = mmap(NULL, XGPIO_ALLOC_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, gpio1_fd_, 0);
        if(gpio1_baseaddr_ == MAP_FAILED) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "Could not get baseaddr : " + std::string(strerror(errno)));
        }

        gpio2_baseaddr_ = mmap(NULL, XGPIO_ALLOC_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, gpio2_fd_, 0);
        if(gpio2_baseaddr_ == MAP_FAILED) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "Could not get baseaddr : " + std::string(strerror(errno)));
        }
    }

    RegAccessor::~RegAccessor() {
        close(gpio1_fd_);
        close(gpio2_fd_);
        munmap(gpio1_baseaddr_, XGPIO_ALLOC_SIZE);
        munmap(gpio2_baseaddr_, XGPIO_ALLOC_SIZE);
    }

    uint32_t RegAccessor::read(const uint8_t& addr) const {
        REG(gpio2_baseaddr_, XGPIO_DATA_OFFSET) = (GPIO2_O_CTRL_START |
                                                   (addr & GPIO2_O_CTRL_ADDR_MASK));

        auto cnt = 0;
        while(!(REG(gpio2_baseaddr_, XGPIO_DATA2_OFFSET) & GPIO2_I_CTRL_DONE)) {
            if(1e6 < cnt) {
                throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                         "Accessing UART synchronous registar was timeout");
            }
            usleep(1);
            cnt++;
        }

        REG(gpio2_baseaddr_, XGPIO_DATA_OFFSET) = 0;

        return REG(gpio1_baseaddr_, XGPIO_DATA2_OFFSET);
    }

    void RegAccessor::write(const uint8_t& addr, const uint32_t& data) const {
        REG(gpio1_baseaddr_, XGPIO_DATA_OFFSET) = data;
        REG(gpio2_baseaddr_, XGPIO_DATA_OFFSET) = (GPIO2_O_CTRL_START |
                                                   GPIO2_O_CTRL_WRITE_MODE |
                                                   (addr & GPIO2_O_CTRL_ADDR_MASK));

        auto cnt = 0;
        while(!(REG(gpio2_baseaddr_, XGPIO_DATA2_OFFSET) & GPIO2_I_CTRL_DONE)) {
            if(1e6 < cnt) {
                throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                         "Accessing UART synchronous registar was timeout");
            }
            usleep(1);
            cnt++;
        }

        REG(gpio1_baseaddr_, XGPIO_DATA_OFFSET) = 0;
        REG(gpio2_baseaddr_, XGPIO_DATA_OFFSET) = 0;
    }
}
