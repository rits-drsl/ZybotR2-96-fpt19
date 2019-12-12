/**
 *  PedestrianDetector: 歩行者の認識を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "PedestrianDetector.h"

namespace fad {
    PedestrianDetector::PedestrianDetector() {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");
        core::YAMLHelper::readStruct(root_path + PARAM_YAML_PATH, param_, "Pedestrian");

        // カスケード分類器のパラメータの読み込み
        pedestrian_classifier_.load(root_path + param_.pedestrian_haar_like_cascade_file_path);
    }

    PedestrianDetector::~PedestrianDetector() {
    }

    const core::PedestrianStatus& PedestrianDetector::getResult() const {
        return result_;
    }

    bool PedestrianDetector::detect(const cv::Mat& color_img) {
        const auto& top    = param_.y_of_top_of_detection_area;
        const auto& bottom = param_.y_of_bottom_of_detection_area;
        const auto& left   = param_.x_of_left_of_detection_area;
        const auto& right  = param_.x_of_right_of_detection_area;

        result_ = core::PedestrianStatus();

        // 画像の領域を絞る
        cv::Mat corrected_img(color_img, cv::Rect(left, top, right - left, bottom - top));

        // Haar-Like特徴量を用いたカスケード分類器で歩行者を認識する
        std::vector<cv::Rect> cand_of_pedestrian_regions;
        pedestrian_classifier_.detectMultiScale(corrected_img, cand_of_pedestrian_regions, param_.haar_like_scale_factor, param_.haar_like_min_neighbors);
        if(!cand_of_pedestrian_regions.empty()) {
            auto max_eval_val = 0.0;
            for(const auto& cand_of_pedestrian_region : cand_of_pedestrian_regions) {
                // 領域に対して、HSV変換・色域二値化・Closing・ラベリングを実行する
                auto cand_mat = cv::Mat(corrected_img, cand_of_pedestrian_region).clone();
                cv::GaussianBlur(cand_mat, cand_mat, cv::Size(param_.gaussian_kernel_size, param_.gaussian_kernel_size), 0);
                cv::cvtColor(cand_mat, cand_mat, cv::COLOR_BGR2HSV);
                cv::inRange(cand_mat, cv::Scalar(param_.hsv_trans_min_h, param_.hsv_trans_min_s, param_.hsv_trans_min_v),
                            cv::Scalar(param_.hsv_trans_max_h, param_.hsv_trans_max_s, param_.hsv_trans_max_v), cand_mat);
                cv::morphologyEx(cand_mat, cand_mat, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), param_.nof_closing);

                cv::Mat label_map;
                const auto label_info_map = improc::LabelingExecutor::execute(cand_mat, label_map);
                for(const auto& label_info : label_info_map) {
                    const auto& tl = label_info.second.begin;
                    const auto& br = label_info.second.end;
                    const auto aspect_ratio = (br.x - tl.x) / (double)(br.y - tl.y);
                    const auto ratio_of_labeled_region = label_info.second.area / (double)cand_of_pedestrian_region.area();
                    const auto calc_eval_val = [](double val, const double& lower, const double& upper) -> double {
                                                   val = core::Util::clamp(val, lower, upper);
                                                   return std::abs(val - lower) * std::abs(val - upper);
                                               };
                    const auto aspect_eval_val = calc_eval_val(aspect_ratio, param_.aspect_ratio_lower, param_.aspect_ratio_upper);
                    const auto region_eval_val = calc_eval_val(ratio_of_labeled_region, param_.ratio_of_labeled_region_lower, param_.ratio_of_labeled_region_upper);
                    const auto eval_val = aspect_eval_val * region_eval_val;
                    if(max_eval_val < eval_val) {
                        max_eval_val = eval_val;
                        result_.rect = cand_of_pedestrian_region;
                        cv::inRange(label_map, label_info.first, label_info.first, result_.region);
                        result_.comment = std::to_string(eval_val);
                    }
                }
            }
        }
        return result_.rect.empty() ? false : true;
    }
}
