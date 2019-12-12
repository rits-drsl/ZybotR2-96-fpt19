/**
 *  PSGPIOController: PS GPIO制御クラス
 *                    CAM_EN_PIN : PcamのVcc
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#include <pcam/ps/PSGPIO.h>

namespace zynqpl {
    PSGPIO::PSGPIO() {
        int fd;
        char fname[128];

        // ポートを有効化する
        fd = open("/sys/class/gpio/export", O_WRONLY);
        if(!fd) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not open file of gpio SysFS : " + std::string(strerror(errno)));
        }
        if(!write(fd, std::to_string(CAM_EN_PIN).c_str(), 3)) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not enable CAM GPIO : " + std::string(strerror(errno)));
        }
        close(fd);

        // ポートのdirectionを設定する
        sprintf(fname, "/sys/class/gpio/gpio%d/direction", CAM_EN_PIN);
        fd = open(fname, O_WRONLY);
        if(!fd) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not open file of CAM GPIO direction : " + std::string(strerror(errno)));
        }
        if(!write(fd, "out", 3)) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not set CAM GPIO direction : " + std::string(strerror(errno)));
        }
        close(fd);
    }

    PSGPIO::~PSGPIO() {
        // ポートを無効化する
        int fd = open("/sys/class/gpio/unexport", O_WRONLY);
        close(fd);
    }

    void PSGPIO::turnOnPowerToCam() const {
        char fname[128];

        sprintf(fname, "/sys/class/gpio/gpio%d/value", CAM_EN_PIN);
        int cam_en_fd = open(fname, O_WRONLY);

        if(!write(cam_en_fd, "1", 1)) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not write \"1\" to CAM GPIO : " + std::string(strerror(errno)));
        }

        close(cam_en_fd);
    }

    void PSGPIO::turnOffPowerToCam() const {
        char fname[128];

        sprintf(fname, "/sys/class/gpio/gpio%d/value", CAM_EN_PIN);
        int cam_en_fd = open(fname, O_WRONLY);

        if(!write(cam_en_fd, "0", 1)) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "]" +
                                     "could not write \"0\" to CAM GPIO : " + std::string(strerror(errno)));
        }

        close(cam_en_fd);
    }
}
