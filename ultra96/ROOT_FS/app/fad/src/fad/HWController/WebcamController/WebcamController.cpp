/**
 *  WebcamController: Webカメラから画像を取得するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "WebcamController.h"

namespace fad {
    WebcamController::WebcamController() {
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

        // webカメラ関連のクラス・バッファを初期化
        WebCamParam webcam_param;
        core::YAMLHelper::readStruct(root_path + HW_PARAM_YAML_PATH, webcam_param, "WebCam");
        auto webcam_size = cv::Size(webcam_param.width, webcam_param.height);

        webcam_cap_ = cv::VideoCapture('0' - devname_param.webcam.back()); // TODO: もう少し丁寧に判定する
        if(!webcam_cap_.isOpened()) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                     "Web camera device can not open");
        }

        webcam_cap_.set(CV_CAP_PROP_FRAME_WIDTH,  webcam_size.width);
        webcam_cap_.set(CV_CAP_PROP_FRAME_HEIGHT, webcam_size.height);

        std::string calibration_parameter_file;
        core::YAMLHelper::read(root_path + HW_PARAM_YAML_PATH, calibration_parameter_file, "WebCam", "calibration_parameter_file", robot_name);
        webcam_corrector_ = std::make_unique<improc::ImageCorrector>(webcam_size, root_path + calibration_parameter_file);
        webcam_img_buf_   = cv::Mat(webcam_size, CV_8UC3);
        webcam_img_       = cv::Mat(webcam_size, CV_8UC3);
    }

    WebcamController::~WebcamController() {
    }

    const cv::Mat& WebcamController::getWebCamImage() {
        return webcam_img_;
    }

    void WebcamController::updateWebCamImage() {
        webcam_cap_.read(webcam_img_buf_);
        webcam_corrector_->execute(webcam_img_buf_, webcam_img_);
    }
}

