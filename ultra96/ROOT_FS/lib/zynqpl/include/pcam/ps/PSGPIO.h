/**
 *  PSGPIO: PS GPIO制御クラス
 *          CAM_EN_PIN : PcamのVcc
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#ifndef ZYNQPL_INCLUDE_PCAM_PS_PSGPIO_H_
#define ZYNQPL_INCLUDE_PCAM_PS_PSGPIO_H_

#include <iostream>
#include <utility>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

namespace zynqpl {
class PSGPIO {
    public:
        PSGPIO();
        ~PSGPIO();

        // CMOSイメージセンサへの給電を開始する
        void turnOnPowerToCam() const;
        // CMOSイメージセンサへの給電を停止する
        void turnOffPowerToCam() const;
    private:
        const uint32_t CAM_EN_PIN = 375;
    };
}

#endif /* ZYNQPL_INCLUDE_PCAM_PS_PSGPIO_H_ */
