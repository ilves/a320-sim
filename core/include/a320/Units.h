#pragma once

namespace a320 {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kFtToM = 0.3048;
constexpr double kMToFt = 1.0 / kFtToM;
constexpr double kNmToM = 1852.0;
constexpr double kKtToMps = kNmToM / 3600.0;
constexpr double kFpsToKt = kFtToM / kKtToMps;
constexpr double kGravityFps2 = 32.174;

template <typename T>
constexpr T clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline double lerp(double a, double b, double t) { return a + (b - a) * t; }

}  // namespace a320
