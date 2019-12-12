/**
 *  PcamController: Pcamの制御を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_HWCONTROLLER_PCAMCONTROLLER_PCAMCONTROLLER_H_
#define FAD_SRC_FAD_HWCONTROLLER_PCAMCONTROLLER_PCAMCONTROLLER_H_

#include <numeric>

#include <EnvironmentMap.hpp>
#include <YAMLHelper.hpp>

#include <zynqpl.h>

#include "PcamImageCorrector/PcamImageCorrector.h"

namespace fad {
    class PcamController {
    public:
        enum class ImageType {
            GRAY, // グレイスケール画像
            SAT,  // 彩度二値化画像
            EDGE, // エッジ画像
            BIN   // 二値化画像
        };

        PcamController();
        ~PcamController();

        PcamController(const PcamController& obj) = delete;
        PcamController &operator=(const PcamController& obj) = delete;

        /**
         *  @brief 車体前方の路面の俯瞰画像の参照を取得する
         */
        const core::EnvironmentMap<uint8_t>& getRoadSurfaceImage(const ImageType& type);

        /**
         *  @brief 車体前方の路面の俯瞰画像を更新する
         *  @param (ratio) 取得する俯瞰画像の1pixあたりの実空間上での大きさ(メートル)
         */
        void updateRoadSurfaceImage(const double& ratio);

    private:
        const std::string HW_PARAM_YAML_PATH = "/data/HWController/param.yaml";

        class DevNameParam : public core::YAMLHelper::ParamBase {
        public:
            std::string pcam;
            std::string iic;
            std::string preimproc;

            void read(const cv::FileNode& node) override {
                pcam      = (std::string)node["pcam"];
                iic       = (std::string)node["iic"];
                preimproc = (std::string)node["preimproc"];
            }
        };

        class PreImProcParam : public core::YAMLHelper::ParamBase {
        public:
            uint8_t bin_thr;
            uint8_t sat_thr;
            uint8_t hysteresis_hthr;
            uint8_t hysteresis_lthr;

            void read(const cv::FileNode& node) override {
                bin_thr         = (int)node["bin_thr"];
                sat_thr         = (int)node["sat_thr"];
                hysteresis_hthr = (int)node["hysteresis_hthr"];
                hysteresis_lthr = (int)node["hysteresis_lthr"];
            }
        };

        std::unique_ptr<zynqpl::Pcam>       pcam_;
        std::unique_ptr<zynqpl::PreImProc>  hw_preimproc_;
        std::unique_ptr<PcamImageCorrector> pcam_corrector_;

        core::EnvironmentMap<uint8_t> bird_eye_img_buf_;
        std::map<ImageType, core::EnvironmentMap<uint8_t>> bird_eye_img_;
    };
}

#endif  /* FAD_SRC_FAD_HWCONTROLLER_PCAMCONTROLLER_PCAMCONTROLLER_H_ */
