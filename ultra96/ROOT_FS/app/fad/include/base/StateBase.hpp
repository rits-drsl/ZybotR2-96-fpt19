/**
 *  StateBase.hpp: 状態を表すクラスの基底クラス
 *
 *  Copyright (C) 2019 Yuya Kudo.
 *  Copyright (C) 2019 Atsushi Takada.
 *  Authors:
 *      Yuya Kudo      <ri0049ee@ed.ritsumei.ac.jp>
 *      Atsushi Takada <ri0051rr@ed.ritsumei.ac.jp>
 *
 */

#ifndef INCLUDE_BASE_STATEBASE_HPP_
#define INCLUDE_BASE_STATEBASE_HPP_

#include <iostream>
#include <cmath>
#include <vector>

#include <Theta.hpp>

namespace core {
    namespace base {
        class StateBase {
        public:
            double x = 0;
            double y = 0;
            Theta  t = Theta(0.0);
            double v = 0;

            StateBase() = default;
            constexpr StateBase(const double& _x,
                                const double& _y,
                                const double& _t = 0,
                                const double& _v = 0) :
                x(_x), y(_y), t(_t), v(_v) {}

            constexpr StateBase(const double& _x,
                                const double& _y,
                                const Theta&  _t,
                                const double& _v = 0) :
                x(_x), y(_y), t(_t), v(_v) {}

            double norm() const {
                return sqrt(normSquare());
            }

            constexpr double normSquare() const {
                return dot(*this);
            }

            constexpr double dot(const StateBase& other) const {
                return x * other.x + y * other.y;
            }

            double distanceFrom(const StateBase& other) const {
                return (other - *this).norm();
            }

            StateBase normalized() const {
                return *this / norm();
            }

            bool isZero() const {
                return x == 0 && y == 0 && t.isZero() && v == 0;
            }

            std::vector<StateBase> getEightStatesOnCircumference(const double& r) const {
                const std::vector<double> state_v{x, y};
                std::vector<StateBase> ret;
                const auto dim = 2;
                for(int i = 0; i < dim * 2; i++) {
                    auto target = state_v;
                    target[i / 2] += (i % 2 == 0) ? r : -r;
                    ret.emplace_back(target[0], target[1]);
                }
                const auto diag_val = std::sin(4 / core::PI) * r;
                for(int i = 0; i < (1 << dim); i++) {
                    auto target = state_v;
                    for(size_t j = 0; j < dim; j++) {
                        target[j] += (i & (1 << j)) ? diag_val : -diag_val;
                    }
                    ret.emplace_back(target[0], target[1]);
                }
                return ret;
            };

            constexpr StateBase operator +() const {
                return *this;
            }

            constexpr StateBase operator -() const {
                return {-x, -y, -t, -v};
            }

            constexpr StateBase operator +(const StateBase& other) const {
                return {x + other.x, y + other.y, t + other.t, v + other.v};
            }

            constexpr StateBase operator -(const StateBase& other) const {
                return {x - other.x, y - other.y, t - other.t, v - other.v};
            }

            constexpr StateBase operator *(double s) const {
                return {x * s, y * s, t, v};
            }

            constexpr StateBase operator/(double s) const {
                return {x / s, y / s, t, v};
            }

            constexpr StateBase operator +=(const StateBase& other) {
                x += other.x;
                y += other.y;
                t += other.t;
                v += other.v;
                return *this;
            }

            constexpr StateBase operator -=(const StateBase& other) {
                x -= other.x;
                y -= other.y;
                t -= other.t;
                v -= other.v;
                return *this;
            }

            constexpr bool operator ==(const StateBase& other) const {
                return (x == other.x && y == other.y && t == other.t && v == other.v);
            }

            constexpr bool operator !=(const StateBase& other) const {
                return !(x == other.x && y == other.y && t == other.t && v == other.v);
            }

            friend std::ostream& operator <<(std::ostream& os, const StateBase& obj) {
                os << "[x : "       << obj.x << ", "
                   << "y : "        << obj.y << ", "
                   << "theta : "    << obj.t.getDegree() << "°, "
                   << "velocity : " << obj.v << "]";
                return os;
            }
        };
    }
}

#endif /* INCLUDE_BASE_STATEBASE_HPP_ */
