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

#include "PcamController.h"

namespace fad {
    PcamController::PcamController() {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        if(std::getenv("ROBOT_NAME") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export ROBOT_NAME=<name of your robot> ");
        }
        const auto robot_name = std::getenv("ROBOT_NAME");

        DevNameParam devname_param;
        core::YAMLHelper::readStruct(root_path + HW_PARAM_YAML_PATH, devname_param, "devname");

        pcam_         = std::make_unique<zynqpl::Pcam>(devname_param.pcam, devname_param.iic);
        hw_preimproc_ = std::make_unique<zynqpl::PreImProc>(devname_param.preimproc);

        // 路面画像処理用クラス・バッファを初期化
        double      pcam_ratio;
        cv::Mat     homography_mat;
        double      distance_im_center_from_vehicle;
        std::string calibration_parameter_file;
        core::YAMLHelper::read(root_path + HW_PARAM_YAML_PATH, pcam_ratio, "Pcam", "ratio", robot_name);
        core::YAMLHelper::read(root_path + HW_PARAM_YAML_PATH, homography_mat, "Pcam", "H", robot_name);
        core::YAMLHelper::read(root_path + HW_PARAM_YAML_PATH, distance_im_center_from_vehicle, "Pcam", "distance_im_center_from_vehicle", robot_name);
        core::YAMLHelper::read(root_path + HW_PARAM_YAML_PATH, calibration_parameter_file, "Pcam", "calibration_parameter_file", robot_name);

        const auto pcam_size = cv::Size(pcam_->getImageWidth(), pcam_->getImageHeight());
        pcam_corrector_ = std::make_unique<PcamImageCorrector>(pcam_size, root_path + calibration_parameter_file, homography_mat.ptr<double>());

        bird_eye_img_buf_.map    = cv::Mat::zeros(pcam_size, CV_8UC1);
        bird_eye_img_buf_.ratio  = pcam_ratio;
        bird_eye_img_buf_.offset = cv::Point2d(distance_im_center_from_vehicle, 0.0);

        bird_eye_img_.emplace(ImageType::GRAY, bird_eye_img_buf_.clone());
        bird_eye_img_.emplace(ImageType::SAT,  bird_eye_img_buf_.clone());
        bird_eye_img_.emplace(ImageType::EDGE, bird_eye_img_buf_.clone());
        bird_eye_img_.emplace(ImageType::BIN,  bird_eye_img_buf_.clone());

        // 前処理のパラメータを設定
        PreImProcParam preimproc_param;
        core::YAMLHelper::readStruct(root_path + HW_PARAM_YAML_PATH, preimproc_param, "PreImProc");
        hw_preimproc_->SetGrayBinThreshold(preimproc_param.bin_thr);
        hw_preimproc_->SetSatBinThreshold(preimproc_param.sat_thr);
        hw_preimproc_->SetHistThreshold(preimproc_param.hysteresis_hthr,
                                        preimproc_param.hysteresis_lthr);
    }

    PcamController::~PcamController() {
    }

    const core::EnvironmentMap<uint8_t>& PcamController::getRoadSurfaceImage(const ImageType& type) {
        return bird_eye_img_[type];
    }

    void PcamController::updateRoadSurfaceImage(const double& ratio) {
        // Pcamから画像を取得する
        pcam_->fetchFrame(bird_eye_img_buf_.map.data);

        // 歪曲収差補正・射影変換を行い、俯瞰画像に変換する
        pcam_corrector_->execute(bird_eye_img_buf_.map,
                                 bird_eye_img_[ImageType::GRAY].map,
                                 bird_eye_img_[ImageType::SAT].map,
                                 bird_eye_img_[ImageType::EDGE].map,
                                 bird_eye_img_[ImageType::BIN].map,
                                 bird_eye_img_buf_.ratio / ratio);

        for(auto& img : bird_eye_img_) {
            img.second.ratio = ratio;
        }
    }
}

