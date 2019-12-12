/**
 *  Preimproc: 前処理を行うモジュールの制御を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#ifndef ZYNQPL_INCLUDE_PREIMPROC_PREIMPROC_H_
#define ZYNQPL_INCLUDE_PREIMPROC_PREIMPROC_H_

#include <iostream>
#include <utility>
#include <stdexcept>
#include <cstdint>
#include <cstring>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#define REG(base, offset) *((volatile uint32_t*)((uintptr_t)(base) + (offset)))

namespace zynqpl {
    class PreImProc {
    public:
        PreImProc(std::string preimproc_devname);
        ~PreImProc();

        // 二値化しきい値を設定する
        void SetGrayBinThreshold(uint8_t thr) const;

        // 彩度しきい値を設定する
        void SetSatBinThreshold(uint8_t thr) const;

        // ヒステリシスしきい値を設定する
        void SetHistThreshold(uint8_t hthr, uint8_t lthr) const;

    private:
        int   preimproc_fd_;
        void* preimproc_baseaddr_;

        const uint32_t PREIMPTOC_ALLOC_SIZE = 0x10000;

        const uint32_t XPREIMPROC_PARAM_ADDR_SAT_BIN_THR_DATA  = 0x10;
        const uint32_t XPREIMPROC_PARAM_BITS_SAT_BIN_THR_DATA  = 8;
        const uint32_t XPREIMPROC_PARAM_ADDR_GRAY_BIN_THR_DATA = 0x18;
        const uint32_t XPREIMPROC_PARAM_BITS_GRAY_BIN_THR_DATA = 8;
        const uint32_t XPREIMPROC_PARAM_ADDR_HIST_HTHR_DATA    = 0x20;
        const uint32_t XPREIMPROC_PARAM_BITS_HIST_HTHR_DATA    = 8;
        const uint32_t XPREIMPROC_PARAM_ADDR_HIST_LTHR_DATA    = 0x28;
        const uint32_t XPREIMPROC_PARAM_BITS_HIST_LTHR_DATA    = 8;
    };
}

#endif /* ZYNQPL_INCLUDE_PREIMPROC_PREIMPROC_H_ */
