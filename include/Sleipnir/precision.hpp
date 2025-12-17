#pragma once

#include <cmath>

namespace cyclone {
using real = float;

inline real real_sqrt(real x) { return std::sqrt(x); }
inline real real_pow(real a, real b) { return std::pow(a, b); }
inline real real_abs(real x) { return std::fabs(x); }

inline real real_sin(real x) { return std::sin(x); }
inline real real_cos(real x) { return std::cos(x); }
inline real real_exp(real x) { return std::exp(x); }

constexpr real real_pi = static_cast<real>(3.14159265358979323846);

constexpr real REAL_MAX = std::numeric_limits<real>::max();

}