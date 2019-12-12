/**
 *  PSIIC: PS IIC制御クラス
 *         for 16bit memory address
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#ifndef ZYNQPL_INCLUDE_PCAM_PS_PSIIC_H_
#define ZYNQPL_INCLUDE_PCAM_PS_PSIIC_H_

#include <iostream>
#include <utility>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

namespace zynqpl {
    class PSIIC {
    public:
        PSIIC(const std::string& iic_devname, uint8_t slave_addr);
        ~PSIIC();
        uint8_t iicRead(uint16_t addr) const;
        void iicWrite(uint16_t addr, uint8_t data) const;
    private:
        int iic_fd_;
    };
}

#endif /* ZYNQPL_INCLUDE_PCAM_PS_PSIIC_H_ */
