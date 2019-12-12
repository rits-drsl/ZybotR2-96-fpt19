/**
 *  RegAccessor: レジスタ同期システムのレジスタにアクセスするクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#ifndef ZYNQPL_INCLUDE_CMOD_REG_REGACCESSOR_H_
#define ZYNQPL_INCLUDE_CMOD_REG_REGACCESSOR_H_

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <stdexcept>
#include <cstring>
#include <cstdint>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#define REG(base, offset) *((volatile uint32_t*)((uintptr_t)(base) + (offset)))

namespace zynqpl {

    /**
     *  RegAccessor
     *  レジスタ同期システムのレジスタにアクセスするクラス
     */
    class RegAccessor {
    public:

        /**
         *  RegAccessor(Constructor)
         *  @gpio1_devname: /dev/下にあるdataの入出力を行うAXI GPIOのデバイスファイル名
         *  @gpio2_devname: /dev/下にある制御信号の入出力を行うAXI GPIOのデバイスファイル名
         */
        RegAccessor(const std::string& gpio1_devname,
                    const std::string& gpio2_devname);
        ~RegAccessor();

        RegAccessor(const RegAccessor& obj) = delete;
        RegAccessor &operator =(const RegAccessor& obj) = delete;

        /**
         *  レジスタの値を読み込む
         *  @addr: 読み込むレジスタのアドレス
         */
        uint32_t read(const uint8_t& addr) const;

        /**
         *  レジスタに値を書き込む
         *  @addr: 書き込むレジスタのアドレス
         *  @data: レジスタに書き込む値
         */
        void write(const uint8_t& addr, const uint32_t& data) const;

    private:
        int gpio1_fd_, gpio2_fd_;
        void* gpio1_baseaddr_;
        void* gpio2_baseaddr_;

        const uint32_t XGPIO_ALLOC_SIZE   = 0x1000;
        const uint32_t XGPIO_DATA_OFFSET  = 0x0;    // Data register for 1st channel
        const uint32_t XGPIO_DATA2_OFFSET = 0x8;    // Data register for 2st channel

        const uint32_t GPIO2_O_CTRL_ADDR_MASK  =   0b11111111;
        const uint32_t GPIO2_O_CTRL_START      =  0b100000000;
        const uint32_t GPIO2_O_CTRL_WRITE_MODE = 0b1000000000;
        const uint32_t GPIO2_I_CTRL_DONE       =          0b1;
    };
}

#endif /* ZYNQPL_INCLUDE_CMOD_REG_REGACCESSOR_H_ */

