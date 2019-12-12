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

#ifndef FAD_SRC_FAD_HWCONTROLLER_OPTORCAMCONTROLLER_OPTORCAMCONTROLLER_H_
#define FAD_SRC_FAD_HWCONTROLLER_OPTORCAMCONTROLLER_OPTORCAMCONTROLLER_H_

#include <thread>

#include <YAMLHelper.hpp>

#include <improc.h>
#include <optorusb.h>
#include <optorimu.h>
#include <optorcam.h>

namespace fad {
    class OptorCamController {
    public:
        OptorCamController();
        ~OptorCamController();

        OptorCamController(const OptorCamController& obj) = delete;
        OptorCamController &operator=(const OptorCamController& obj) = delete;

        const cv::Mat& getRightImage() const;
        const cv::Mat& getLeftImage() const;
        double getRightImageUpdateTime() const;
        double getLeftImageUpdateTime() const;
        void updateOptorImage();

    private:
        const std::string HW_PARAM_YAML_PATH = "/data/HWController/param.yaml";

        std::unique_ptr<improc::ImageCorrector> right_img_corrector_;
        std::unique_ptr<improc::ImageCorrector> left_img_corrector_;

        cv::Mat right_img_, right_img_buf_;
        cv::Mat left_img_, left_img_buf_;

        double right_img_get_time_, left_img_get_time_;
    };
}

#endif  /* FAD_SRC_FAD_HWCONTROLLER_OPTORCAMCONTROLLER_OPTORCAMCONTROLLER_H_ */
