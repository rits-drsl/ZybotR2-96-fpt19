/**
 *  ProcTimer: 処理時間計測クラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_PROCTIMER_HPP_
#define INCLUDE_PROCTIMER_HPP_

#include <iostream>
#include <vector>
#include <string>
#include <chrono>

namespace core {
    class ProcTimer {
    public:
        explicit ProcTimer(const uint32_t& max_hold_size) noexcept;
        void start() noexcept;
        void stop() noexcept;
        void clear() noexcept;
        double getElapsedTime() const noexcept;
        double getAverageElapsedTime() const noexcept;
        void dumpAverageElapsedTime(const std::string& proc_name = "Process") const noexcept;
        void dumpElapsedTime(const std::string& proc_name = "Process") const noexcept;
    private:
        std::chrono::system_clock::time_point t_start_;
        std::chrono::system_clock::time_point t_end_;
        std::vector<double>                   elapsed_times_;
        uint32_t                              max_hold_size_;
    };

    inline ProcTimer::ProcTimer(const uint32_t& max_hold_size) noexcept :
        t_start_(std::chrono::system_clock::now()),
        t_end_(std::chrono::system_clock::now()),
        max_hold_size_(max_hold_size == 0 ? 1 : max_hold_size) {
    }

    inline void ProcTimer::start() noexcept {
        t_start_ = std::chrono::system_clock::now();
        if(max_hold_size_ <= elapsed_times_.size()) {
            elapsed_times_.erase(elapsed_times_.begin());
        }
    }

    inline void ProcTimer::stop() noexcept {
        t_end_       = std::chrono::system_clock::now();
        elapsed_times_.push_back(std::chrono::duration_cast<std::chrono::microseconds>(t_end_ - t_start_).count());
    }

    inline void ProcTimer::clear() noexcept {
        elapsed_times_.clear();
    }

    inline double ProcTimer::getElapsedTime() const noexcept {
        if(elapsed_times_.size() == 0) {
            return 0;
        }
        else {
            return elapsed_times_.back() / 1e6f;
        }
    }

    inline double ProcTimer::getAverageElapsedTime() const noexcept {
        if(elapsed_times_.size() == 0) {
            return 0;
        }
        else {
            double ret = 0;
            for (const auto &elapsed_time : elapsed_times_) {
                ret += elapsed_time;
            }
            return ret / (double)elapsed_times_.size() / 1e6f;
        }
    }

    inline void ProcTimer::dumpElapsedTime(const std::string& proc_name) const noexcept {
        if(elapsed_times_.size() == 0) {
            return;
        }
        else {
            std::cout << "[Elapsed Time] "
                      << proc_name
                      << " : "
                      << elapsed_times_.back() / 1e6f
                      << "s"
                      << std::endl << std::endl;
        }
    }

    inline void ProcTimer::dumpAverageElapsedTime(const std::string& proc_name) const noexcept {
        if(elapsed_times_.size() == 0) {
            return;
        }
        else {
            double ret = 0;
            for(const auto& elapsed_time : elapsed_times_) {
                ret += elapsed_time;
            }

            std::cout << "[Elapsed Time] "
                      << proc_name
                      << " : "
                      << ret / (double)elapsed_times_.size() / 1e6f
                      << "s"
                      << std::endl << std::endl;
        }
    }
}

#endif /* INCLUDE_PROCTIMER_HPP_ */
