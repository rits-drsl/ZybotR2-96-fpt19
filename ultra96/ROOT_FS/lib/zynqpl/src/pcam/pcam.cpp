/**
 *  Pcam: Pcamの初期化・Pcamからの画像取得を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *
 *  Authors:
 *      Yuya Kudo <ri0049ee@ed.ritsumei.ac.jp>
 */

#include <pcam/pcam.h>

namespace zynqpl {
    Pcam::Pcam(const std::string& video_devname,
               const std::string& iic_devname,
               OV5640_cfg::mode_t mode,
               OV5640_cfg::awb_t  awb,
               uint32_t           pixelformat) :
        width_(OV5640_cfg::resolutions[mode].width),
        height_(OV5640_cfg::resolutions[mode].height) {
        psgpio_ = std::make_unique<PSGPIO>();
        psiic_  = std::make_unique<PSIIC>(iic_devname, OV5640_cfg::OV5640_SLAVE_ADDR);

        reset();
        init();
        applyMode(mode);
        applyAwb(awb);

        video_ = std::make_unique<VideoController>(video_devname,
                                                   OV5640_cfg::resolutions[mode].width,
                                                   OV5640_cfg::resolutions[mode].height,
                                                   pixelformat);
    }

    Pcam::~Pcam() {
        shutdown();
    }

    void Pcam::fetchFrame(uint8_t* frame) const {
        auto v4l2_buf = video_->grub();
        memcpy(frame, v4l2_buf, video_->buf_size);
        video_->release();
    }

    uint32_t Pcam::getImageWidth() const {
        return width_;
    }

    uint32_t Pcam::getImageHeight() const {
        return height_;
    }

    void Pcam::init() const {
        uint8_t id_h, id_l;
        readReg(OV5640_cfg::REG_ID_H, id_h);
        readReg(OV5640_cfg::REG_ID_H, id_h);
        readReg(OV5640_cfg::REG_ID_L, id_l);

        if (id_h != OV5640_cfg::DEV_ID_H_ || id_l != OV5640_cfg::DEV_ID_L_) {
            char msg[100];
            snprintf(msg, sizeof(msg), "Got %02x %02x. Expected %02x %02x\r\n",
                     id_h, id_l, OV5640_cfg::DEV_ID_H_, OV5640_cfg::DEV_ID_L_);
            throw std::runtime_error(std::string(msg));
        }

        writeReg(0x3103, 0x11);
        writeReg(0x3008, 0x82);
        usleep(10000);

        for(size_t i = 0; i < sizeof(OV5640_cfg::cfg_init_) / sizeof(OV5640_cfg::cfg_init_[0]); ++i) {
            writeReg(OV5640_cfg::cfg_init_[i].addr, OV5640_cfg::cfg_init_[i].data);
        }
    }

    void Pcam::shutdown() const {
        psgpio_->turnOffPowerToCam();
        usleep(10000);
    }

    void Pcam::reset() const {
        psgpio_->turnOffPowerToCam();
        usleep(10000);
        psgpio_->turnOnPowerToCam();
        usleep(10000);
    }

    void Pcam::applyMode(OV5640_cfg::mode_t mode) const {
        if(mode >= OV5640_cfg::mode_t::MODE_END) {
            throw std::runtime_error("OV5640 MODE setting is invalid");
        }
        writeReg(0x3008, 0x42);
        const auto cfg_mode = &OV5640_cfg::modes[mode];
        writeConfig(cfg_mode->cfg, cfg_mode->cfg_size);
        writeReg(0x3008, 0x02);
    }

    void Pcam::applyAwb(OV5640_cfg::awb_t awb) const {
        if(awb >= OV5640_cfg::awb_t::AWB_END) {
            throw std::runtime_error("OV5640 AWB setting is invalid");
        }
        writeReg(0x3008, 0x42);
        auto cfg_mode = &OV5640_cfg::awbs[awb];
        writeConfig(cfg_mode->cfg, cfg_mode->cfg_size);
        writeReg(0x3008, 0x02);
    }

    void Pcam::readReg(uint16_t reg_addr, uint8_t& buf) const {
        buf = psiic_->iicRead(reg_addr);
        usleep(10000);
    }

    void Pcam::writeReg(uint16_t reg_addr, uint8_t const reg_data) const {
        auto cnt = 10;
        while(true) {
            psiic_->iicWrite(reg_addr, reg_data);
            usleep(10000);

            // soft reset
            if(reg_addr == 0x3008) break;

            uint8_t buf;
            readReg(reg_addr, buf);
            if(buf == reg_data) {
                std::cout << "[PCam init : Status OK] ";
            }
            else {
                std::cout << "[PCam init : Status NG] ";
            }

            printf("addr : 0x%04X, write : 0x%02X, read : 0x%02X\n", reg_addr, (int)reg_data, (int)buf);
            if(buf == reg_data) break;

            cnt--;
            if(cnt == 0) {
                throw std::runtime_error("process that write to reg by using iic is failure");
            }
        }
    }

    void Pcam::writeConfig(OV5640_cfg::config_word_t const* cfg, size_t cfg_size) const {
        for(size_t i = 0; i < cfg_size; ++i) {
            writeReg(cfg[i].addr, cfg[i].data);
        }
    }
}
