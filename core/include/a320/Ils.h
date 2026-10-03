#pragma once

#include "a320/Airport.h"

namespace a320 {

// Deviation sign convention follows the PFD: positive LOC dots = localizer is to the
// right (fly right), positive G/S dots = glideslope is above (fly up).
struct IlsSignal {
  bool locValid = false;
  bool gsValid = false;
  double locDots = 0.0;
  double gsDots = 0.0;
  double locDeviationDeg = 0.0;  // aircraft angle off course seen from the LOC antenna, + = right
  double gsElevationDeg = 0.0;   // aircraft elevation angle seen from the G/S origin
  double dmeNm = 0.0;            // slant range to the localizer antenna
};

// Geometry of one ILS, derived from the runway (ICAO Annex 10 sector definitions).
class Ils {
 public:
  Ils(const LocalFrame& frame, const Runway& runway);

  IlsSignal receive(const Enu& aircraft) const;

  // Point on the glidepath at a distance from the threshold (negative x = on approach).
  RunwayPoint glidepathPoint(double distanceFromThresholdM) const;

  double localizerX() const { return locX_; }
  double glideslopeOriginX() const { return gsX_; }
  double locHalfSectorDeg() const { return locHalfSectorDeg_; }
  double glideslopeDeg() const { return gsDeg_; }
  const RunwayAxes& axes() const { return axes_; }

 private:
  RunwayAxes axes_;
  double gsDeg_;
  double locX_;              // LOC antenna, along-course from threshold
  double gsX_;               // G/S origin (where the glidepath meets the threshold elevation)
  double locHalfSectorDeg_;  // full-scale (2 dots): +-105 m at the threshold
};

// PAPI on the left of the runway at the G/S origin; lights ordered left to right as seen
// on approach. On the glidepath: two white, two red.
struct PapiState {
  bool white[4] = {false, false, false, false};
};

PapiState computePapi(const Ils& ils, const Enu& eye);

}  // namespace a320
