/**
 *  TrafficLightDetector: 信号機の認識を行うクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "TrafficLightDetector.h"

namespace fad {
    TrafficLightDetector::TrafficLightDetector() {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");
        core::YAMLHelper::readStruct(root_path + PARAM_YAML_PATH, tl_param_, "TrafficLight");

        // カスケード分類器のパラメータの読み込み
        red_tl_classifier_.load(root_path + tl_param_.red_tl_haar_like_cascade_file_path);

        // テンプレート画像の読み込み
        int nof_red_template;
        core::YAMLHelper::read(root_path + PARAM_YAML_PATH, nof_red_template, "Templates", "num");
        for(int templ_i = 0; templ_i < nof_red_template; templ_i++) {
            TemplateParam templ_param;
            core::YAMLHelper::readStruct(root_path + PARAM_YAML_PATH, templ_param, "Templates", "template" + std::to_string(templ_i));
            cv::Mat templ = cv::imread(root_path + templ_param.file_path);
            red_templates_.emplace_back(templ_param.distance, std::move(templ));
        }
    }

    TrafficLightDetector::~TrafficLightDetector() {
    }

    const TrafficLightDetector::TLDetectInfo& TrafficLightDetector::getResult() const {
        return result_;
    }

    bool TrafficLightDetector::detect(const cv::Mat& color_img,
                                      const double& distance_to_tl) {
        if(distance_to_tl == -1) return false;

        const auto& top    = tl_param_.y_of_top_of_detection_area;
        const auto& bottom = tl_param_.y_of_bottom_of_detection_area;
        const auto& left   = tl_param_.x_of_left_of_detection_area;
        const auto& right  = tl_param_.x_of_right_of_detection_area;

        // 画像の領域を絞る
        cv::Mat corrected_img(color_img, cv::Rect(left, top, right - left, bottom - top));

        // Haar-Like特徴量を用いたカスケード分類器で赤信号を認識する
        std::vector<cv::Rect> cand_of_red_tl_regions;
        red_tl_classifier_.detectMultiScale(corrected_img, cand_of_red_tl_regions, tl_param_.haar_like_scale_factor, tl_param_.haar_like_min_neighbors);
        if(cand_of_red_tl_regions.empty()) {
            result_ = TLDetectInfo();
        }
        else {
            // 最も距離が近いテンプレート画像を取得する
            // TODO: にぶたん
            auto min_dist = std::numeric_limits<double>::max();
            auto min_dist_index = -1;
            for(size_t i = 0; i < red_templates_.size(); i++) {
                const auto dist = std::abs(distance_to_tl - red_templates_[i].first);
                if(dist < min_dist) {
                    min_dist = dist;
                    min_dist_index = i;
                }
            }
            const auto& red_templ = red_templates_[min_dist_index].second;

            // 各領域に対してテンプレートマッチングを実行する
            auto total_max_val = 0.0;
            auto result_rect   = cv::Rect();
            for(auto& cand_of_red_tl_region : cand_of_red_tl_regions) {
                if(cand_of_red_tl_region.width < red_templ.cols * (1 - tl_param_.region_width_ratio_with_tm)   ||
                   red_templ.cols * (1 + tl_param_.region_width_ratio_with_tm) < cand_of_red_tl_region.width   ||
                   cand_of_red_tl_region.height < red_templ.rows * (1 - tl_param_.region_height_ratio_with_tm) ||
                   red_templ.rows * (1 + tl_param_.region_height_ratio_with_tm) < cand_of_red_tl_region.height) {
                    continue;
                }
                if(cand_of_red_tl_region.width  < red_templ.cols) {
                    const auto additional_pix = red_templ.cols - cand_of_red_tl_region.width;
                    cand_of_red_tl_region.width += additional_pix;
                    if(corrected_img.cols < cand_of_red_tl_region.br().x) {
                        cand_of_red_tl_region.x -= additional_pix;
                    }
                }
                if(cand_of_red_tl_region.height  < red_templ.rows) {
                    const auto additional_pix = red_templ.rows - cand_of_red_tl_region.height;
                    cand_of_red_tl_region.height += additional_pix;
                    if(corrected_img.rows < cand_of_red_tl_region.br().y) {
                        cand_of_red_tl_region.y -= additional_pix;
                    }
                }
                cv::Mat   result;
                cv::Point max_pt;
                double    max_val;
                cv::matchTemplate(red_templ, cv::Mat(corrected_img, cand_of_red_tl_region), result, tl_param_.opencv_tm_method);
                cv::minMaxLoc(result, NULL, &max_val, NULL, &max_pt);
                if(total_max_val < max_val) {
                    total_max_val = max_val;
                    result_rect = cv::Rect(cand_of_red_tl_region.x + left, cand_of_red_tl_region.y + top,
                                           cand_of_red_tl_region.width, cand_of_red_tl_region.height);
                }
            }
            if(total_max_val < tl_param_.tm_max_val_thr) {
                result_ = TLDetectInfo();
            }
            else {
                result_ = TLDetectInfo(result_rect, TLDetectInfo::Type::RED, total_max_val);
            }
        }
        return result_.type == TLDetectInfo::Type::NONE ? false : true;
    }
}
