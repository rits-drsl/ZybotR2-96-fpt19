/**
 *  ParticleFilter: パーティクルフィルタを用いて
 *                  現状態を推定するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef FAD_SRC_FAD_ROADSURFACESLAM_PARTICLEFILTER_PARTICLEFILTER_H_
#define FAD_SRC_FAD_ROADSURFACESLAM_PARTICLEFILTER_PARTICLEFILTER_H_

#include <vector>
#include <random>
#include <chrono>

#include <YAMLHelper.hpp>
#include <VehicleState.hpp>
#include <WheelOdometry.hpp>
#include <VisualOdometry.hpp>
#include <InertialOdometry.hpp>
#include <ParticleState.hpp>

namespace fad {
    class ParticleFilter {
    public:
        ParticleFilter();
        ~ParticleFilter();

        ParticleFilter(const ParticleFilter& obj) = delete;
        ParticleFilter &operator=(const ParticleFilter& obj) = delete;

        ParticleFilter(ParticleFilter&&) = default;

        /**
         *  @brief 保持しているパーティクルの状態を初期化する
         */
        void init(const core::VehicleState& init_state);

        /**
         *  @brief パーティクルフィルタを用いた次状態の推定を実行する
         *  @return 次状態の推定に成功した場合、Trueを返す
         */
        void estimate(const core::WheelOdometry&    wo,
                      const core::VisualOdometry&   vo,
                      const core::InertialOdometry& io,
                      const double&                 wo_ps_sigma = 1.0,
                      const double&                 wo_t_sigma  = 1.0);

        /**
         *  @brief 推定した状態の参照を返す
         */
        const core::VehicleState& getState() const;

        /**
         *  @brief パーティクルを返す
         */
        const std::vector<core::ParticleState>& getParticles() const;

        /**
         *  @brief パーティクルの重さの合計を返す
         */
        const double& getParticlesWeightSum() const;

    private:
        const std::string PARTICLE_FILTER_PARAM_YAML_PATH = "/data/StateEstimator/ParticleFilter/param.yaml";

        class PFParam : public core::YAMLHelper::ParamBase {
        public:
            uint32_t max_num_particle;
            double likelifood_position_sigma;
            double likelifood_theta_sigma;
            double reset_ratio_thr;
            double reset_position_sigma;
            double reset_theta_sigma;
            double vo_ratio_when_calc_disparity;
            double vo_ps_sigma;
            double vo_t_sigma;
            double io_ps_sigma;
            double io_t_sigma;

            void read(const cv::FileNode& node) override {
                max_num_particle             = (int)node["max_num_particle"];
                likelifood_position_sigma    = (double)node["likelifood_position_sigma"];
                likelifood_theta_sigma       = (double)node["likelifood_theta_sigma"];
                reset_ratio_thr              = (double)node["reset_ratio_thr"];
                reset_position_sigma         = (double)node["reset_position_sigma"];
                reset_theta_sigma            = (double)node["reset_theta_sigma"];
                vo_ratio_when_calc_disparity = (double)node["vo_ratio_when_calc_disparity"];
                vo_ps_sigma                  = (double)node["vo_ps_sigma"];
                vo_t_sigma                   = (double)node["vo_t_sigma"];
                io_ps_sigma                  = (double)node["io_ps_sigma"];
                io_t_sigma                   = (double)node["io_t_sigma"];
            }
        };

        PFParam pf_param_;

        double init_weight_;

        std::chrono::system_clock::time_point prev_time_;

        std::mt19937 rand_;

        core::VehicleState               state_;
        std::vector<core::ParticleState> particles_;

        std::uniform_real_distribution<double> dist_uni_real_position_;
        std::uniform_real_distribution<double> dist_uni_real_theta_;

        double reset_thr_;   // リセットのしきい値
        double weight_sum_;  // パーティクルの重みの総和

        /**
         *  @brief パーティクルの状態を更新する
         */
        void updateParticles(const core::WheelOdometry&    wo,
                             const core::VisualOdometry&   vo,
                             const core::InertialOdometry& io,
                             const double&                 wo_ps_sigma,
                             const double&                 wo_t_sigma);

        /**
         *  @brief リサンプリングを行う
         */
        void resampling();

        /**
         *  @brief 膨張リセットを行う
         */
        void resetExpansionary();
    };
}

#endif /* FAD_SRC_FAD_ROADSURFACESLAM_PARTICLEFILTER_PARTICLEFILTER_H_ */
