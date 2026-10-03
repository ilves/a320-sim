#include "a320/Geo.h"

#include <cmath>

#include "a320/Units.h"

namespace a320 {
namespace {

constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);

void geoToEcef(const GeoPos& p, double& x, double& y, double& z) {
  const double lat = p.latDeg * kDegToRad;
  const double lon = p.lonDeg * kDegToRad;
  const double sl = std::sin(lat);
  const double n = kA / std::sqrt(1.0 - kE2 * sl * sl);
  x = (n + p.altM) * std::cos(lat) * std::cos(lon);
  y = (n + p.altM) * std::cos(lat) * std::sin(lon);
  z = (n * (1.0 - kE2) + p.altM) * sl;
}

GeoPos ecefToGeo(double x, double y, double z) {
  const double lon = std::atan2(y, x);
  const double p = std::hypot(x, y);
  double lat = std::atan2(z, p * (1.0 - kE2));
  double alt = 0.0;
  for (int i = 0; i < 6; ++i) {
    const double sl = std::sin(lat);
    const double n = kA / std::sqrt(1.0 - kE2 * sl * sl);
    alt = p / std::cos(lat) - n;
    lat = std::atan2(z, p * (1.0 - kE2 * n / (n + alt)));
  }
  return {lat * kRadToDeg, lon * kRadToDeg, alt};
}

}  // namespace

LocalFrame::LocalFrame(const GeoPos& origin) : origin_(origin) {
  geoToEcef(origin, ox_, oy_, oz_);
  sinLat_ = std::sin(origin.latDeg * kDegToRad);
  cosLat_ = std::cos(origin.latDeg * kDegToRad);
  sinLon_ = std::sin(origin.lonDeg * kDegToRad);
  cosLon_ = std::cos(origin.lonDeg * kDegToRad);
}

Enu LocalFrame::toEnu(const GeoPos& p) const {
  double x, y, z;
  geoToEcef(p, x, y, z);
  const double dx = x - ox_, dy = y - oy_, dz = z - oz_;
  return {-sinLon_ * dx + cosLon_ * dy,
          -sinLat_ * cosLon_ * dx - sinLat_ * sinLon_ * dy + cosLat_ * dz,
          cosLat_ * cosLon_ * dx + cosLat_ * sinLon_ * dy + sinLat_ * dz};
}

GeoPos LocalFrame::toGeo(const Enu& p) const {
  const double dx = -sinLon_ * p.e - sinLat_ * cosLon_ * p.n + cosLat_ * cosLon_ * p.u;
  const double dy = cosLon_ * p.e - sinLat_ * sinLon_ * p.n + cosLat_ * sinLon_ * p.u;
  const double dz = cosLat_ * p.n + sinLat_ * p.u;
  return ecefToGeo(ox_ + dx, oy_ + dy, oz_ + dz);
}

}  // namespace a320
