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

#include "ParticleFilter.h"

namespace fad {
    ParticleFilter::ParticleFilter() :
        rand_(std::mt19937(std::random_device()())) {
        if(std::getenv("FAD_ROOT") == nullptr) {
            throw std::logic_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                   "Please set environment value : $ export FAD_ROOT=<path of project root> ");
        }
        const auto root_path = std::getenv("FAD_ROOT");
        // パーティクルフィルタのパラメータを読み込む
        core::YAMLHelper::readStruct(root_path + PARTICLE_FILTER_PARAM_YAML_PATH, pf_param_, "param");

        init_weight_ = 1.0/ pf_param_.max_num_particle;
        reset_thr_   = pf_param_.max_num_particle * pf_param_.reset_ratio_thr;

        if(!(0.0 <= pf_param_.reset_ratio_thr && pf_param_.reset_ratio_thr <= 1.0)){
            throw std::invalid_argument("invalid reset_ratio_thr");
        }
        if(!(0.0 <= pf_param_.vo_ratio_when_calc_disparity && pf_param_.vo_ratio_when_calc_disparity <= 1.0)){
            throw std::invalid_argument("invalid vo_ratio_when_calc_disparity");
        }

        dist_uni_real_position_ = std::uniform_real_distribution<double>(-pf_param_.reset_position_sigma, pf_param_.reset_position_sigma);
        dist_uni_real_theta_    = std::uniform_real_distribution<double>(-pf_param_.reset_theta_sigma, pf_param_.reset_theta_sigma);
    }

    ParticleFilter::~ParticleFilter() {
    }

    void ParticleFilter::init(const core::VehicleState& init_state) {
        state_     = init_state;
        particles_ = std::vector<core::ParticleState>(pf_param_.max_num_particle,
                                                      core::ParticleState(init_state.x,
                                                                          init_state.y,
                                                                          init_state.t,
                                                                          init_state.v,
                                                                          init_weight_));
    }

    void ParticleFilter::estimate(const core::WheelOdometry&    wo,
                                  const core::VisualOdometry&   vo,
                                  const core::InertialOdometry& io,
                                  const double&                 wo_ps_sigma,
                                  const double&                 wo_t_sigma) {
        updateParticles(wo, vo, io, wo_ps_sigma, wo_t_sigma);

        // 全粒子の加重平均により次状態の代表を決定する
        // NOTE: Thetaの加重平均を取る際は境界を考慮する必要があります
        struct WeightedTheta {
            core::Theta t; double w;
            WeightedTheta(const core::Theta& _t, const double& _w) : t(_t), w(_w) { }
        };

        std::vector<WeightedTheta> wt_v;
        auto x_sum = 0.0;
        auto y_sum = 0.0;
        for(size_t i = 1; i < particles_.size(); i++){
            x_sum += particles_[i].x * particles_[i].w;
            y_sum += particles_[i].y * particles_[i].w;
            wt_v.emplace_back(particles_[i].t, particles_[i].w);
        }

        const auto calc_wt_ave = [&](const WeightedTheta& a, const WeightedTheta& b) -> core::Theta {
                                     if(std::abs(a.t.get() - b.t.get()) <= core::PI) {
                                         return core::Theta((a.t.get() * a.w + b.t.get() * b.w)  / (a.w + b.w));
                                     }
                                     else {
                                         const auto pi = core::Theta(core::PI);
                                         return core::Theta(((a.t + pi).get() * a.w + (b.t + pi).get() * b.w) / (a.w + b.w)) - pi;
                                     }
                                 };

        const auto calc_all_wt_ave = [&](auto&& f, std::vector<WeightedTheta>& wt_v, size_t wt_v_index) -> core::Theta {
                                         if(wt_v.size() <= 1) {
                                             throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                                                      "The number of particle is few");
                                         }
                                         else if(wt_v_index == 2) {
                                             return calc_wt_ave(wt_v[0], wt_v[1]);
                                         }
                                         else {
                                             if(wt_v_index % 2 == 1) wt_v_index--;
                                             for(size_t i = 0; i < wt_v_index; i += 2) {
                                                 wt_v[i / 2].t = calc_wt_ave(wt_v[i], wt_v[i + 1]);
                                                 wt_v[i / 2].w = wt_v[i].w + wt_v[i + 1].w;
                                             }
                                             return f(f, wt_v, wt_v_index / 2);
                                         }
                                     };

        auto new_state = core::VehicleState(x_sum / weight_sum_,
                                            y_sum / weight_sum_,
                                            calc_all_wt_ave(calc_all_wt_ave, wt_v, wt_v.size()));

        const auto current_time = std::chrono::system_clock::now();
        const auto interval     = std::chrono::duration_cast<std::chrono::microseconds>(current_time - prev_time_).count() / 1e6;
        new_state.v = new_state.distanceFrom(state_) / interval;

        const auto dir = core::Theta(std::atan2(new_state.y - state_.y, new_state.x - state_.x));
        if(90 <= new_state.t.disparityWith(dir).getDegree()) {
            new_state.v *= -1;
        }

        state_     = new_state;
        prev_time_ = current_time;

        if(weight_sum_ < reset_thr_) {
            resetExpansionary();
        }
        else {
            resampling();
        }
    }

    const core::VehicleState& ParticleFilter::getState() const {
        return state_;
    }

    const std::vector<core::ParticleState> & ParticleFilter::getParticles() const {
        return particles_;
    }

    const double& ParticleFilter::getParticlesWeightSum() const {
        return weight_sum_;
    }

    void ParticleFilter::resetExpansionary() {
        for(auto &particle : particles_) {
            particle.x += dist_uni_real_position_(rand_);
            particle.y += dist_uni_real_position_(rand_);
            particle.t += core::Theta(dist_uni_real_theta_(rand_));
            particle.w = 1.0 / particles_.size();
        }
    }

    void ParticleFilter::updateParticles(const core::WheelOdometry&  wo,
                                         const core::VisualOdometry& vo,
                                         const core::InertialOdometry& io,
                                         const double& wo_ps_sigma,
                                         const double& wo_t_sigma) {
        weight_sum_ = 0;
        auto calc_gauss = [](const double& m, const double& s, const double& v) -> double {
                              return (1 / (std::sqrt(2 * core::PI) * s)) * std::exp(- std::pow(v - m, 2) / (2 * std::pow(s, 2)));
                          };
        auto calc_trans_with_gauss = [&](const core::base::StateBase& state,
                                         const core::base::StateBase& odometry,
                                         const double&                ps_sigma = 1.0,
                                         const double&                t_sigma  = 1.0) -> core::ParticleState {
                                         auto dist_normal_position = std::normal_distribution<>(0, ps_sigma);
                                         auto dist_normal_theta    = std::normal_distribution<>(0, t_sigma);
                                         return core::ParticleState(state.x + odometry.x + dist_normal_position(rand_),
                                                                    state.y + odometry.y + dist_normal_position(rand_),
                                                                    state.t + odometry.t + core::Theta(dist_normal_theta(rand_)));
                                     };

        const auto obs_by_vo = calc_trans_with_gauss(state_, vo, pf_param_.vo_ps_sigma, pf_param_.vo_t_sigma);
        const auto obs_by_io = calc_trans_with_gauss(state_, io, pf_param_.io_ps_sigma, pf_param_.io_t_sigma);
        for(auto& particle : particles_) {
            // 状態遷移モデルに基づいて位置・向きを導出
            auto new_particle = calc_trans_with_gauss(particle, wo, wo_ps_sigma, wo_t_sigma);

            // 尤度を計算
            const auto vo_ps_disparity = new_particle.distanceFrom(obs_by_vo);
            const auto vo_t_disparity  = new_particle.t.disparityWith(obs_by_vo.t).raw;
            const auto io_ps_disparity = new_particle.distanceFrom(obs_by_io);
            const auto io_t_disparity  = new_particle.t.disparityWith(obs_by_io.t).raw;
            new_particle.w = particle.w *
                pf_param_.vo_ratio_when_calc_disparity * (
                (calc_gauss(0, pf_param_.likelifood_position_sigma, vo_ps_disparity) *
                 calc_gauss(0, pf_param_.likelifood_theta_sigma, vo_t_disparity)) +
                (1 - pf_param_.vo_ratio_when_calc_disparity) *
                (calc_gauss(0, pf_param_.likelifood_position_sigma, io_ps_disparity) *
                 calc_gauss(0, pf_param_.likelifood_theta_sigma, io_t_disparity)));

            // パーティクルを更新
            particle = new_particle;
            weight_sum_ += new_particle.w;
        }
        if(weight_sum_ == 0) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                     "All particles are dead");
        }
    }

    void ParticleFilter::resampling() {
        std::vector<core::ParticleState> prev;

        double sum_weight = 0.0;
        for(auto &p : particles_) {
            p.w += sum_weight;
            sum_weight = p.w;
            prev.push_back(p);
        }

        std::vector<int> choice(particles_.size());
        std::uniform_real_distribution<double> rnd(0.0, 1.0);
        auto accum = rnd(rand_) / particles_.size();
        auto step  = sum_weight / particles_.size();

        size_t j = 0;
        for(auto &c : choice) {
            while(prev[j].w <= accum && j < particles_.size() - 1) j++;
            c = j;
            accum += step;
        }

        for(size_t i = 0; i < particles_.size(); i++) {
            particles_[i]   = prev[choice[i]];
            particles_[i].w = 1.0 / particles_.size();
        }
    }
}
