/**
 *  VOCalculator: VisualOdometryを導出するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "VOCalculator.h"

namespace fad {
    VOCalculator::VOCalculator(const cv::Mat&            init_frame,
                               const core::VehicleState& init_state) {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        // パラメータの読み込み
        core::YAMLHelper::readStruct(root_path + VO_PARAM_YAML_PATH, vo_param_, "Basis");
        core::YAMLHelper::readStruct(root_path + VO_PARAM_YAML_PATH, fast_param_, "Fast");
        core::YAMLHelper::readStruct(root_path + VO_PARAM_YAML_PATH, five_pt_param_, "FivePt");
        core::YAMLHelper::readStruct(root_path + VO_PARAM_YAML_PATH, optical_flow_param_, "OpticalFlow");

        // 初期化
        init(init_frame, init_state);
    }

    VOCalculator::~VOCalculator() {
    }

    void VOCalculator::init(const cv::Mat&            init_frame,
                            const core::VehicleState& init_state) {
        R_ = cv::Mat::eye(cv::Size(3, 3), CV_64F);

        cv::cvtColor(init_frame, prev_frame_, cv::COLOR_BGR2GRAY);
        detectFeaturePt(init_frame, prev_feature_pt_);
        prev_state_ = init_state;
        prev_theta_ = 0;

        vo_q_ = std::vector<core::VisualOdometry>(vo_param_.vo_queue_depth);
        vo_q_front_index_ = 0;
    }

    const core::VisualOdometry& VOCalculator::get() const {
        return vo_;
    }

    bool VOCalculator::run(const cv::Mat&            current_frame,
                           const core::VehicleState& current_state) {
        cv::cvtColor(current_frame, gray_frame_, cv::COLOR_BGR2GRAY);

        // Lucas-Kanade法を用いてオプティカルフローを導出
        std::vector<cv::Point2f> current_feature_pt;
        trackFeaturePt(prev_frame_,
                       gray_frame_,
                       prev_feature_pt_,
                       current_feature_pt);

        // Nisterの5点アルゴリズムとRANSACを用いて基礎行列を推定
        cv::Mat mask, R, t;
        const auto E = cv::findEssentialMat(current_feature_pt, prev_feature_pt_,
                                            five_pt_param_.focal,
                                            cv::Point2d(five_pt_param_.pp_x, five_pt_param_.pp_y),
                                            cv::RANSAC,
                                            five_pt_param_.prob,
                                            five_pt_param_.threshold,
                                            mask);

        // 基礎行列に対する特異値分解を行い、回転行列と並進ベクトルを導出
        cv::recoverPose(E, current_feature_pt, prev_feature_pt_, R, t,
                        five_pt_param_.focal,
                        cv::Point2d(five_pt_param_.pp_x, five_pt_param_.pp_y),
                        mask);

        // 追跡出来た特徴点の数が少ない場合、特徴点を再抽出する
        if(prev_feature_pt_.size() < vo_param_.feature_pt_min_num_feat) {
            detectFeaturePt(prev_frame_, prev_feature_pt_);
            trackFeaturePt(prev_frame_,
                           gray_frame_,
                           prev_feature_pt_,
                           current_feature_pt);
        }

        // 回転行列の更新
        R_ = R * R_;

        // 移動平均フィルタを用いて最終的なVOを導出
        cv::Mat odometry = prev_state_.distanceFrom(current_state) * (R_ * t);
        const auto theta = std::atan2(odometry.at<double>(0), odometry.at<double>(2));
        const auto dt    = theta - prev_theta_;

        vo_q_front_index_ = (vo_q_front_index_ + 1 == vo_param_.vo_queue_depth) ? 0 : vo_q_front_index_ + 1;
        vo_q_[vo_q_front_index_] = core::VisualOdometry(odometry.at<double>(2),
                                                        odometry.at<double>(0),
                                                        dt);

        vo_ = core::VisualOdometry();
        auto vo_t = 0.0;
        for(const auto& v : vo_q_) {
            vo_.x += v.x;
            vo_.y += v.y;
            vo_t  += v.t.get();
        }
        vo_.x /= vo_q_.size();
        vo_.y /= vo_q_.size();

        auto vo_norm = vo_.norm();
        if(vo_param_.vo_dtheta_valid_norm_threshold <= vo_norm) {
            vo_.t = core::Theta(vo_t / vo_q_.size());
        }

        // 全体座標系へ変換
        vo_.x = vo_norm * std::cos((current_state.t + vo_.t).get());
        vo_.y = vo_norm * std::sin((current_state.t + vo_.t).get());

        // 保持する情報を更新
        prev_state_      = current_state;
        prev_feature_pt_ = current_feature_pt;
        prev_frame_      = gray_frame_.clone();
        prev_theta_      = theta;

        return true;
    }

    void VOCalculator::trackFeaturePt(const cv::Mat&            prev_img,
                                      const cv::Mat&            next_img,
                                      std::vector<cv::Point2f>& prev_pt,
                                      std::vector<cv::Point2f>& next_pt) const {
        const auto termcrit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                               optical_flow_param_.tarm_criteria_max_count,
                                               optical_flow_param_.tarm_criteria_epsilon);

        std::vector<float>   err;
        std::vector<uint8_t> status;
        cv::calcOpticalFlowPyrLK(prev_img, next_img, prev_pt, next_pt, status, err,
                                 cv::Size(optical_flow_param_.win_size, optical_flow_param_.win_size),
                                 optical_flow_param_.max_level, termcrit, 0, optical_flow_param_.min_eig_threshold);

        // 追跡出来なかった特徴点を削除する
        auto l_index = 0;
        for(size_t i = 0; i < status.size(); i++) {
            if(status.at(i) == 0) continue;
            prev_pt.at(l_index) = prev_pt.at(i);
            next_pt.at(l_index) = next_pt.at(i);
            l_index++;
        }
        prev_pt.erase(prev_pt.begin() + l_index, prev_pt.end());
        next_pt.erase(next_pt.begin() + l_index, next_pt.end());
    }

    void VOCalculator::detectFeaturePt(const cv::Mat&            prev_img,
                                       std::vector<cv::Point2f>& prev_pt) const {
        std::vector<cv::KeyPoint> keypoints_1;
        cv::FAST(prev_img, keypoints_1, fast_param_.threshold, fast_param_.use_nms);
        cv::KeyPoint::convert(keypoints_1, prev_pt, std::vector<int>());
    }
}
