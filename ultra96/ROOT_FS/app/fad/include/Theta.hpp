/**
 *  Theta: 弧度法で角度を表現するクラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_THETA_HPP_
#define INCLUDE_THETA_HPP_

#include <cmath>
#include <algorithm>

namespace core {
    constexpr double PI = 3.14159265359;

    /**
     *  角度を[-π, π)の範囲で表現し、境界を隠匿する
     */
    class Theta {
    public:
        double raw;

        Theta() = default;
        constexpr explicit Theta(const double& val) : raw(val) { }

        constexpr double get() const {
            if(-PI <= raw && raw < PI) {
                return raw;
            }
            else {
                // 範囲外であった場合、値を正規化する
                double raw_mod = fmod(raw + PI, 2 * PI);
                return (0 <= raw_mod) ? raw_mod - PI : raw_mod + PI;
            }
        }

        constexpr double getDegree() const {
            return (this->get() / PI) * 180;
        }

        constexpr bool isZero() const {
            return this->get() == 0.0;
        }

        constexpr Theta disparityWith(const Theta& other) const {
            const auto t  = this->get();
            const auto ot = other.get();
            return Theta(std::min(std::abs(t - ot), core::PI * 2 - std::abs(t) - std::abs(ot)));
        }

        constexpr Theta operator +() const {
            return *this;
        }

        constexpr Theta operator -() const {
            return Theta(-raw);
        }

        constexpr Theta operator +(const Theta& other) const {
            return Theta(raw + other.raw);
        }

        constexpr Theta operator -(const Theta& other) const {
            return Theta(raw - other.raw);
        }

        constexpr Theta operator *(double s) const {
            return Theta(raw * s);
        }

        constexpr Theta operator /(double s) const {
            return Theta(raw / s);
        }

        constexpr Theta operator +=(const Theta& other) {
            raw += other.raw;
            return *this;
        }

        constexpr Theta operator -=(const Theta& other) {
            raw -= other.raw;
            return *this;
        }

        constexpr bool operator ==(const Theta& other) const {
            return this->get() == other.get();
        }

        constexpr bool operator !=(const Theta& other) const {
            return this->get() != other.get();
        }

        constexpr bool operator <(const Theta& other) const {
            return this->get() < other.get();
        }

        constexpr bool operator <=(const Theta& other) const {
            return this->get() <= other.get();
        }

        constexpr bool operator >(const Theta& other) const {
            return this->get() > other.get();
        }

        constexpr bool operator >=(const Theta& other) const {
            return this->get() >= other.get();
        }
    };
}

#endif /* INCLUDE_THETA_HPP_ */
