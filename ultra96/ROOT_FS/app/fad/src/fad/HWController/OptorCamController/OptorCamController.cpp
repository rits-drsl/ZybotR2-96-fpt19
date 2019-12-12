/**
 *  OptorCamController: Optorのカメラから画像データを取得するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "OptorCamController.h"

namespace fad {
    OptorCamController::OptorCamController() {
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

        std::string setting_file;
        core::YAMLHelper::read(root_path + HW_PARAM_YAML_PATH, setting_file, "Optor", "setting_file", robot_name);
        visensor_load_settings((root_path + setting_file).c_str());

        if(visensor_Start_Cameras() < 0) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Could not start optor cam");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        const auto optor_img_size = cv::Size(visensor_img_width(), visensor_img_height());

        std::string r_calibration_parameter_file, l_calibration_parameter_file;
        core::YAMLHelper::read(root_path + HW_PARAM_YAML_PATH, r_calibration_parameter_file, "Optor", "calibration_parameter_file", robot_name, "r");
        core::YAMLHelper::read(root_path + HW_PARAM_YAML_PATH, l_calibration_parameter_file, "Optor", "calibration_parameter_file", robot_name, "l");
        right_img_corrector_ = std::make_unique<improc::ImageCorrector>(optor_img_size, root_path + r_calibration_parameter_file);
        left_img_corrector_  = std::make_unique<improc::ImageCorrector>(optor_img_size, root_path + l_calibration_parameter_file);

        right_img_     = cv::Mat::zeros(optor_img_size, CV_8UC1);
        left_img_      = cv::Mat::zeros(optor_img_size, CV_8UC1);
        right_img_buf_ = cv::Mat::zeros(optor_img_size, CV_8UC1);
        left_img_buf_  = cv::Mat::zeros(optor_img_size, CV_8UC1);
    }

    OptorCamController::~OptorCamController() {
        visensor_Close_Cameras();
    }

    const cv::Mat& OptorCamController::getRightImage() const {
        return right_img_;
    }

    const cv::Mat& OptorCamController::getLeftImage() const {
        return left_img_;
    }

    double OptorCamController::getRightImageUpdateTime() const {
        return right_img_get_time_;
    }

    double OptorCamController::getLeftImageUpdateTime() const {
        return left_img_get_time_;
    }

    void OptorCamController::updateOptorImage() {
        if(!visensor_is_leftcam_open() || !visensor_is_rightcam_open()) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Do not open optor cam");
        }

        bool left_img_update  = false;
        bool right_img_update = false;
        if(visensor_is_left_img_new()) {
            visensor_get_left_latest_img(left_img_buf_.data, &left_img_get_time_);
            left_img_update = true;
        }

        if(visensor_is_right_img_new()) {
            visensor_get_right_latest_img(right_img_buf_.data, &right_img_get_time_);
            right_img_update = true;
        }

        if(left_img_update) {
            left_img_corrector_->execute(left_img_buf_, left_img_);
        }

        if(right_img_update) {
            right_img_corrector_->execute(right_img_buf_, right_img_);
        }
    }
}

