/**
 *  WOCalculator: WheelOdometryを導出するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#include "WOCalculator.h"

namespace fad {
    WOCalculator::WOCalculator() :
        ref_index_cnt_(0) {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");

        // モータ関連のパラメータを読み込む
        core::YAMLHelper::readStruct(root_path + MOTOR_PARAM_YAML_PATH, m_param_, "Motor");
        rot_dist_par_pulse_ = m_param_.one_rotation_dist / m_param_.one_rotation_pulse;

        // WheelOdometryで移動平均を取る際に使用するqueueの深さを読み込む
        int LFQueueDepth;
        core::YAMLHelper::read(root_path + WO_PARAM_YAML_PATH, LFQueueDepth, "LFQueueDepth");
        LFQueueDepth_ = LFQueueDepth;
    }

    WOCalculator::~WOCalculator() {
    }

    void WOCalculator::init(const int& r_cnt,
                            const int& l_cnt) {
        ref_index_cnt_     = 0;
        r_dist_diff_queue_ = std::vector<double>(LFQueueDepth_, 0);
        l_dist_diff_queue_ = std::vector<double>(LFQueueDepth_, 0);
        prev_r_dist_       = r_cnt * rot_dist_par_pulse_;
        prev_l_dist_       = l_cnt * rot_dist_par_pulse_;
        total_wo_          = core::WheelOdometry();
    }

    core::WheelOdometry WOCalculator::get(const core::Theta& current_theta,
                                          const bool reset) {
        auto ret_wo = wo_;
        if(reset) {
            wo_ = core::WheelOdometry();
        }

        // 全体座標系へ変換
        const auto norm_w_odm = ret_wo.norm();
        const auto t          = core::Theta(std::atan2(ret_wo.y, ret_wo.x));
        ret_wo.x = norm_w_odm * std::cos((current_theta + t).get());
        ret_wo.y = norm_w_odm * std::sin((current_theta + t).get());

        return ret_wo;
    }

    const core::WheelOdometry& WOCalculator::getTotal(const core::Theta& current_theta) const {
        return total_wo_;
    }

    void WOCalculator::update(const int& r_cnt,
                              const int& l_cnt) {
        // メートル単位に変換
        const auto r_dist = r_cnt * rot_dist_par_pulse_;
        const auto l_dist = l_cnt * rot_dist_par_pulse_;

        // 前状態との差分を算出し、状態を更新する
        const auto diff_r_dist = r_dist - prev_r_dist_;
        const auto diff_l_dist = l_dist - prev_l_dist_;

        r_dist_diff_queue_[ref_index_cnt_] = diff_r_dist;
        l_dist_diff_queue_[ref_index_cnt_] = diff_l_dist;
        ref_index_cnt_ = (ref_index_cnt_ < LFQueueDepth_ - 1) ? ref_index_cnt_ + 1 : 0;

        // 微小時間あたりの旋回角度・移動量を導出し、現在状態を更新する
        const auto acc_diff_r_dist = std::accumulate(r_dist_diff_queue_.begin(), r_dist_diff_queue_.end(), 0.0);
        const auto acc_diff_l_dist = std::accumulate(l_dist_diff_queue_.begin(), l_dist_diff_queue_.end(), 0.0);
        const auto delta_norm  = (acc_diff_r_dist + acc_diff_l_dist) / (2 * LFQueueDepth_);
        const auto delta_theta = core::Theta((acc_diff_r_dist - acc_diff_l_dist) / (m_param_.tire_tread * LFQueueDepth_));

        wo_.x += delta_norm * std::cos(delta_theta.get());
        wo_.y += delta_norm * std::sin(delta_theta.get());
        wo_.t += delta_theta;

        total_wo_ += wo_;

        // 保持する前情報を更新
        prev_r_dist_ = r_dist;
        prev_l_dist_ = l_dist;
    }
}
