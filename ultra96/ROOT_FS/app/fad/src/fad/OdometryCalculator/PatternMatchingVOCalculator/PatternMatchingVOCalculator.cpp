/**
 *  PatternMatchingVOCalculator: 路面の俯瞰画像からVisualOdometryを導出するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "PatternMatchingVOCalculator.h"

namespace fad {
    PatternMatchingVOCalculator::PatternMatchingVOCalculator() {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        core::YAMLHelper::readStruct(root_path + PARAM_YAML_PATH, param_, "param");

        if(param_.match_method == "AKAZE") {
            pattern_matcher_ = std::make_unique<AKAZEPatternMatcher>();
        }
        else if(param_.match_method == "TM") {
            pattern_matcher_ = std::make_unique<TMPatternMatcher>();
        }
        else {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Pattern matcher type is invalid.");
        }
    }

    PatternMatchingVOCalculator::~PatternMatchingVOCalculator() {
    }

    void PatternMatchingVOCalculator::init(const core::EnvironmentMap<core::RoadType>& ref_world) {
        ref_world_ = ref_world.clone();
    }

    core::VisualOdometry PatternMatchingVOCalculator::get(const bool reset) {
        const auto ret_vo = vo_;
        if(reset) vo_ = core::VisualOdometry();
        return ret_vo;
    }

    void PatternMatchingVOCalculator::update(const core::VehicleState&            current_state,
                                             const core::EnvironmentMap<uint8_t>& bird_eye_bin_img,
                                             const core::EnvironmentMap<uint8_t>& bird_eye_edge_img) {
        if(bird_eye_bin_img.map.size() != bird_eye_edge_img.map.size()) {
            throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                        "Input image size is different");
        }
        const auto ori_size = bird_eye_bin_img.map.size();

        // 二値画像とエッジ画像を合成する
        // NOTE: 二値画像に対してラベリングを行い、ある一定以上の面積の領域を除去する
        cv::Mat label_map;
        auto label_info_map = improc::LabelingExecutor::execute(bird_eye_bin_img.map, label_map);
        cv::Mat pattern_img = cv::Mat::zeros(cv::Size(ori_size.width * (1.0 - (param_.cutting_left_ratio_of_pattern_image + param_.cutting_right_ratio_of_pattern_image)),
                                                      ori_size.height * (1.0 - param_.cutting_above_ratio_of_pattern_image)), CV_8UC1);
        int ori_img_x_offset = ori_size.width * param_.cutting_left_ratio_of_pattern_image;
        int ori_img_y_offset = ori_size.height * param_.cutting_above_ratio_of_pattern_image;
        for(int yi = 0; yi < pattern_img.rows; yi++) {
            for(int xi = 0; xi < pattern_img.cols; xi++) {
                const auto ori_index = ori_img_x_offset + xi + ori_size.width * (yi + ori_img_y_offset);
                if(bird_eye_bin_img.map.data[ori_index] != 0 &&
                   label_info_map[label_map.data[ori_index]].area * std::pow(bird_eye_bin_img.ratio, 2) <= param_.line_area_thr) {
                    pattern_img.data[xi + pattern_img.cols * yi] = 0xFF;
                }

                if(bird_eye_edge_img.map.data[ori_index] != 0) {
                    pattern_img.data[xi + pattern_img.cols * yi] = 0x7F;
                }
            }
        }

        // 路面の俯瞰画像を正規化する
        const auto target_center = cv::Point(pattern_img.cols / 2, pattern_img.rows / 2);
        const auto target_angle  = current_state.t + core::Theta(core::PI / 2);
        const auto w_rot = (int)(std::round(pattern_img.rows * std::abs(std::sin(target_angle.get())) +
                                            pattern_img.cols * std::abs(std::cos(target_angle.get()))));
        const auto h_rot = (int)(std::round(pattern_img.rows * std::abs(std::cos(target_angle.get())) +
                                            pattern_img.cols * std::abs(std::sin(target_angle.get()))));
        const auto target_size = cv::Size(w_rot, h_rot);

        auto trans_mat = cv::getRotationMatrix2D(target_center, -target_angle.getDegree(), 1.0);
        trans_mat.at<double>(0, 2) += w_rot / 2 - pattern_img.cols / 2;
        trans_mat.at<double>(1, 2) += h_rot / 2 - pattern_img.rows / 2;
        cv::warpAffine(pattern_img, pattern_img, trans_mat, target_size, cv::INTER_CUBIC);

        // 参照地図の対応する領域を切り取る
        const auto offset_norm_pix = ref_world_.getCorrespondPixNum(cv::norm(bird_eye_bin_img.offset));
        const auto offset_vector   = cv::Point(offset_norm_pix * std::cos(current_state.t.get()), offset_norm_pix * std::sin(current_state.t.get()));
        const auto ref_center      = ref_world_.getPixPoint(current_state.x, current_state.y) + offset_vector;
        const auto half_ref_side_length_pt = cv::Point(ref_world_.getCorrespondPixNum(param_.cutting_size_of_ref_map) / 2,
                                                       ref_world_.getCorrespondPixNum(param_.cutting_size_of_ref_map) / 2);
        const auto ref_region_tl = ref_center - half_ref_side_length_pt;
        const auto ref_region_br = ref_center + half_ref_side_length_pt;
        const auto corrected_ref_region_tl = cv::Point(core::Util::clamp(ref_region_tl.x, 0, ref_world_.map.cols - 1),
                                                       core::Util::clamp(ref_region_tl.y, 0, ref_world_.map.rows - 1));
        const auto corrected_ref_region_br = cv::Point(core::Util::clamp(ref_region_br.x, 0, ref_world_.map.cols - 1),
                                                       core::Util::clamp(ref_region_br.y, 0, ref_world_.map.rows - 1));
        const auto ref_region = cv::Rect(corrected_ref_region_tl, corrected_ref_region_br);

        auto ref_img = ref_world_.map(ref_region).clone();
        for(int i = 0; i < ref_img.size().area(); i++) {
            ref_img.data[i] =
                (ref_img.data[i] == (uint8_t)core::RoadType::EDGE) ? 0x7F :
                (ref_img.data[i] == (uint8_t)core::RoadType::LINE) ? 0xFF : 0x00;
        }
        cv::resize(pattern_img, pattern_img, cv::Size(), param_.scale_ratio, param_.scale_ratio, cv::INTER_CUBIC);
        cv::resize(ref_img, ref_img, cv::Size(), param_.scale_ratio, param_.scale_ratio, cv::INTER_CUBIC);

        // パターンマッチングを実行
        const auto p_match_is_success = pattern_matcher_->solve(pattern_img,
                                                                ref_img,
                                                                cv::Point(ref_img.cols / 2, ref_img.rows / 2),
                                                                ref_world_.getCorrespondPixNum(param_.pattern_matcher_delta_threshord));

        // パターンマッチングの結果をVOとして保持
        if(p_match_is_success) {
            const auto& result = pattern_matcher_->getResult();
            vo_ = core::VisualOdometry(result.x * ref_world_.ratio * (1.0 / param_.scale_ratio),
                                       result.y * ref_world_.ratio * (1.0 / param_.scale_ratio), result.t);
        }
    }
}
