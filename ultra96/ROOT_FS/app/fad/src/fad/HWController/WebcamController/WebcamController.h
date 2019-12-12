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

#ifndef FAD_SRC_FAD_HWCONTROLLER_WEBCAMCONTROLLER_WEBCAMCONTROLLER_H_
#define FAD_SRC_FAD_HWCONTROLLER_WEBCAMCONTROLLER_WEBCAMCONTROLLER_H_

#include <YAMLHelper.hpp>

#include <improc.h>

namespace fad {
    class WebcamController {
    public:
        WebcamController();
        ~WebcamController();

        WebcamController(const WebcamController& obj) = delete;
        WebcamController &operator=(const WebcamController& obj) = delete;

        /**
         *  @brief webカメラの画像の参照を取得する
         */
        const cv::Mat& getWebCamImage();

        /**
         *  @brief webカメラの画像を更新する
         */
        void updateWebCamImage();

    private:
        const std::string HW_PARAM_YAML_PATH = "/data/HWController/param.yaml";

        class DevNameParam : public core::YAMLHelper::ParamBase {
        public:
            std::string webcam;

            void read(const cv::FileNode& node) override {
                webcam = (std::string)node["webcam"];
            }
        };

        class WebCamParam : public core::YAMLHelper::ParamBase {
        public:
            uint32_t width;
            uint32_t height;

            void read(const cv::FileNode& node) override {
                width  = (int)node["width"];
                height = (int)node["height"];
            }
        };

        std::unique_ptr<improc::ImageCorrector> webcam_corrector_;

        cv::VideoCapture webcam_cap_;
        cv::Mat          webcam_img_buf_;
        cv::Mat          webcam_img_;
    };
}

#endif  /* FAD_SRC_FAD_HWCONTROLLER_WEBCAMCONTROLLER_WEBCAMCONTROLLER_H_ */
