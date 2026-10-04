#include "a320/Ils.h"

#include <cmath>

#include "a320/Units.h"

namespace a320 {
namespace {

constexpr double kLocDotsFullScale = 2.0;
constexpr double kGsDotDegPerGsDeg = 0.12;  // 1 dot = 0.12 x glidepath angle
constexpr double kHalfSectorAtThresholdM = 105.0;
constexpr double kMaxLocHalfSectorDeg = 3.0;

bool locInCoverage(double offCourseDeg, double rangeNm) {
  const double a = std::fabs(offCourseDeg);
  return (a <= 10.0 && rangeNm <= 25.0) || (a <= 35.0 && rangeNm <= 17.0);
}

}  // namespace

Ils::Ils(const LocalFrame& frame, const Runway& runway)
    : axes_(frame, runway), gsDeg_(runway.ils.glideslopeDeg) {
  locX_ = axes_.landingDistanceM() + runway.ils.localizerBeyondEndM;
  gsX_ = runway.ils.thresholdCrossingHeightFt * kFtToM / std::tan(gsDeg_ * kDegToRad);
  locHalfSectorDeg_ = std::fmin(std::atan(kHalfSectorAtThresholdM / locX_) * kRadToDeg,
                                kMaxLocHalfSectorDeg);
}

IlsSignal Ils::receive(const Enu& aircraft) const {
  const RunwayPoint p = axes_.fromEnu(aircraft);
  IlsSignal s;

  const double toLocX = locX_ - p.x;  // > 0 while the antenna is ahead
  const double horizToLoc = std::hypot(toLocX, p.y);
  s.locRangeNm = std::sqrt(horizToLoc * horizToLoc + p.z * p.z) / kNmToM;
  s.locDeviationDeg = std::atan2(p.y, toLocX) * kRadToDeg;
  s.locValid = toLocX > 0.0 && locInCoverage(s.locDeviationDeg, s.locRangeNm);
  s.locDots = -s.locDeviationDeg / locHalfSectorDeg_ * kLocDotsFullScale;

  const double toGsX = gsX_ - p.x;
  const double horizToGs = std::hypot(toGsX, p.y);
  s.dmeNm = std::sqrt(horizToGs * horizToGs + p.z * p.z) / kNmToM;
  s.gsElevationDeg = std::atan2(p.z, horizToGs) * kRadToDeg;
  const double gsAzimuthDeg = std::atan2(p.y, toGsX) * kRadToDeg;
  s.gsValid = toGsX > 0.0 && std::fabs(gsAzimuthDeg) <= 8.0 && horizToGs <= 10.0 * kNmToM &&
              s.gsElevationDeg <= 1.75 * gsDeg_;
  s.gsDots = (gsDeg_ - s.gsElevationDeg) / (kGsDotDegPerGsDeg * gsDeg_);
  return s;
}

RunwayPoint Ils::glidepathPoint(double distanceFromThresholdM) const {
  const double horiz = gsX_ - distanceFromThresholdM;
  return {distanceFromThresholdM, 0.0, horiz * std::tan(gsDeg_ * kDegToRad)};
}

PapiState computePapi(const Ils& ils, const Enu& eye) {
  // Inner light (closest to the runway, rightmost) has the highest transition angle.
  static const double kTransitionOffsetDeg[4] = {-0.5, -1.0 / 6.0, 1.0 / 6.0, 0.5};
  const RunwayPoint p = ils.axes().fromEnu(eye);
  const double horiz = std::hypot(ils.glideslopeOriginX() - p.x, p.y);
  const double angle = std::atan2(p.z, horiz) * kRadToDeg;
  PapiState s;
  for (int i = 0; i < 4; ++i) s.white[i] = angle > ils.glideslopeDeg() + kTransitionOffsetDeg[i];
  return s;
}

}  // namespace a320
