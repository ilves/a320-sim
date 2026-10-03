#pragma once

#include <string>
#include <vector>

#include "a320/Geo.h"

namespace a320 {

// Standard ILS values; verify against the AIP (AD 2.19) before relying on them.
struct IlsSpec {
  double glideslopeDeg = 3.0;
  double thresholdCrossingHeightFt = 50.0;
  double localizerBeyondEndM = 300.0;
};

// One landing direction of a runway. Elevations are treated as ellipsoid heights:
// the whole sim shares one reference, so the geoid offset does not matter.
struct Runway {
  std::string ident;      // "26"
  GeoPos start;           // physical start of the paved runway in this direction
  GeoPos threshold;       // landing threshold (start + displacement)
  GeoPos end;             // physical far end
  double trueCourseDeg = 0.0;
  double widthM = 45.0;
  IlsSpec ils;
};

struct Airport {
  std::string icao;
  std::string name;
  GeoPos reference;  // origin of the sim's local frame, at field elevation
  double magneticVariationDeg = 0.0;  // east positive: magnetic = true - variation
  std::vector<Runway> runways;

  const Runway* find(const std::string& ident) const;
};

// Lennart Meri Tallinn (EETN), runway 08/26, from OurAirports (public domain).
Airport makeTallinn();

struct RunwayPoint {
  double x = 0.0;  // metres along the landing course from the threshold (negative = on approach)
  double y = 0.0;  // metres right of the centreline
  double z = 0.0;  // metres above the threshold
};

// Runway-aligned coordinates on top of the local ENU frame.
class RunwayAxes {
 public:
  RunwayAxes(const LocalFrame& frame, const Runway& runway);

  RunwayPoint fromEnu(const Enu& p) const;
  Enu toEnu(const RunwayPoint& p) const;
  double landingDistanceM() const { return landingDistanceM_; }
  double displacementM() const { return displacementM_; }

 private:
  Enu threshold_;
  double dirE_, dirN_;
  double landingDistanceM_;
  double displacementM_;
};

}  // namespace a320
