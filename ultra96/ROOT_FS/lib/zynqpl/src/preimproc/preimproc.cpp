/**
 *  Preimproc: 前処理を行うモジュールの制御を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#include <preimproc/preimproc.h>

namespace zynqpl {
    PreImProc::PreImProc(std::string preimproc_devname) {
        // deviceのopen
        preimproc_fd_ = open(("/dev/" + preimproc_devname).c_str(), O_RDWR | O_SYNC);
        if(!preimproc_fd_) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not open device of preimproc : " + std::string(strerror(errno)));
        }

        // Baseaddrの取得
        preimproc_baseaddr_ = mmap(NULL, PREIMPTOC_ALLOC_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, preimproc_fd_, 0);
        if(preimproc_baseaddr_ == MAP_FAILED) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not get baseaddr : " + std::string(strerror(errno)));
        }
    }

    PreImProc::~PreImProc() {
        close(preimproc_fd_);
        munmap(preimproc_baseaddr_, PREIMPTOC_ALLOC_SIZE);
    }

    void PreImProc::SetGrayBinThreshold(uint8_t thr) const {
        REG(preimproc_baseaddr_, XPREIMPROC_PARAM_ADDR_GRAY_BIN_THR_DATA) = (uint32_t)(thr);
    }

    void PreImProc::SetSatBinThreshold(uint8_t thr) const {
        REG(preimproc_baseaddr_, XPREIMPROC_PARAM_ADDR_SAT_BIN_THR_DATA) = (uint32_t)(thr);
    }

    void PreImProc::SetHistThreshold(uint8_t hthr, uint8_t lthr) const {
        REG(preimproc_baseaddr_, XPREIMPROC_PARAM_ADDR_HIST_HTHR_DATA) = (uint32_t)(hthr);
        REG(preimproc_baseaddr_, XPREIMPROC_PARAM_ADDR_HIST_LTHR_DATA) = (uint32_t)(lthr);
    }
}
