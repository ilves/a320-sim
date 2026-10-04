#include "a320/Airport.h"

#include <cmath>

#include "a320/Units.h"

namespace a320 {
namespace {

struct RunwayEndData {
  const char* ident;
  double latDeg, lonDeg, elevFt, displacedFt;
};

double courseDeg(const LocalFrame& f, const GeoPos& from, const GeoPos& to) {
  const Enu a = f.toEnu(from), b = f.toEnu(to);
  const double c = std::atan2(b.e - a.e, b.n - a.n) * kRadToDeg;
  return c < 0.0 ? c + 360.0 : c;
}

GeoPos alongRunway(const LocalFrame& f, const GeoPos& from, const GeoPos& to, double metres) {
  const Enu a = f.toEnu(from), b = f.toEnu(to);
  const double len = std::hypot(b.e - a.e, b.n - a.n);
  const double t = metres / len;
  GeoPos p = f.toGeo({a.e + (b.e - a.e) * t, a.n + (b.n - a.n) * t, a.u + (b.u - a.u) * t});
  p.altM = lerp(from.altM, to.altM, t);
  return p;
}

Runway makeDirection(const LocalFrame& f, const RunwayEndData& near, const RunwayEndData& far,
                     double widthM) {
  Runway r;
  r.ident = near.ident;
  r.start = {near.latDeg, near.lonDeg, near.elevFt * kFtToM};
  r.end = {far.latDeg, far.lonDeg, far.elevFt * kFtToM};
  r.threshold = near.displacedFt > 0.0
                    ? alongRunway(f, r.start, r.end, near.displacedFt * kFtToM)
                    : r.start;
  r.trueCourseDeg = courseDeg(f, r.start, r.end);
  r.widthM = widthM;
  return r;
}

}  // namespace

const Runway* Airport::find(const std::string& ident) const {
  for (const Runway& r : runways) {
    if (r.ident == ident) return &r;
  }
  return nullptr;
}

Airport makeTallinn() {
  // OurAirports runways.csv, EETN 08/26: 11417 x 148 ft, 08 threshold displaced 820 ft.
  const RunwayEndData e08{"08", 59.413338, 24.805908, 129.0, 820.0};
  const RunwayEndData e26{"26", 59.413174, 24.867208, 131.0, 0.0};
  const double widthM = 148.0 * kFtToM;

  Airport a;
  a.icao = "EETN";
  a.name = "Tallinn Lennart Meri";
  a.city = "Tallinn";
  a.reference = {(e08.latDeg + e26.latDeg) / 2.0, (e08.lonDeg + e26.lonDeg) / 2.0,
                 131.0 * kFtToM};
  // AIP: 10 E (2025); runway 26 at 270.2 true is the published localizer course 260.
  a.magneticVariationDeg = 10.0;
  const LocalFrame f(a.reference);
  a.runways.push_back(makeDirection(f, e08, e26, widthM));
  a.runways.push_back(makeDirection(f, e26, e08, widthM));
  // Localizer antennas from the AIP coordinates, measured along the centreline past the far end.
  a.runways[0].ils = {"IIB", 108.30, 80.0, 3.0, 54.0, 336.0};
  a.runways[1].ils = {"ILK", 109.30, 260.0, 3.0, 54.0, 262.0};
  return a;
}

Airport makeKuressaare() {
  // AIP AD 2.12: THR 17 58 14 27.69N 022 30 33.61E 10 ft, THR 35 58 13 23.05N 022 30 34.52E 8 ft.
  const RunwayEndData e17{"17", 58.0 + 14.0 / 60.0 + 27.69 / 3600.0, 22.0 + 30.0 / 60.0 + 33.61 / 3600.0, 10.0, 0.0};
  const RunwayEndData e35{"35", 58.0 + 13.0 / 60.0 + 23.05 / 3600.0, 22.0 + 30.0 / 60.0 + 34.52 / 3600.0, 8.0, 0.0};
  Airport a;
  a.icao = "EEKE";
  a.name = "Kuressaare";
  a.city = "Kuressaare";
  // At runway level (9 ft, between the thresholds) rather than the 15 ft aerodrome elevation: the
  // flight model's ground is this height, and the scenery flattens the runway to it.
  a.reference = {58.0 + 13.0 / 60.0 + 48.0 / 3600.0, 22.0 + 30.0 / 60.0 + 34.0 / 3600.0, 9.0 * kFtToM};
  a.magneticVariationDeg = 9.0;  // AIP: 9 E (2025)
  const LocalFrame f(a.reference);
  a.runways.push_back(makeDirection(f, e17, e35, 30.0));
  a.runways.push_back(makeDirection(f, e35, e17, 30.0));
  // ILS 17 (IWA 109.90, course 171): the glide path antenna 302 m past the threshold gives a
  // 52 ft crossing height; the localizer stands 116 m past the far end. Runway 35 has no ILS.
  a.runways[0].ils = {"IWA", 109.90, 171.0, 3.0, 52.0, 116.0};
  a.runways[1].ils = {"", 0.0, 351.0, 3.0, 50.0, 300.0};
  return a;
}

int World::findRunway(const std::string& ident) const {
  for (size_t i = 0; i < runways.size(); ++i)
    if (runways[i].ident == ident) return static_cast<int>(i);
  return -1;
}

int World::nearestAirport(double northM, double eastM) const {
  int best = 0;
  double bestD = 1e18;
  for (size_t i = 0; i < airports.size(); ++i) {
    const double d = std::hypot(northM - airportNorthM[i], eastM - airportEastM[i]);
    if (d < bestD) {
      bestD = d;
      best = static_cast<int>(i);
    }
  }
  return best;
}

World makeWorld(std::vector<Airport> airports) {
  World w;
  w.airports = std::move(airports);
  w.reference = w.airports.front().reference;
  const LocalFrame frame(w.reference);
  for (size_t a = 0; a < w.airports.size(); ++a) {
    const Enu p = frame.toEnu(w.airports[a].reference);
    w.airportNorthM.push_back(p.n);
    w.airportEastM.push_back(p.e);
    for (Runway r : w.airports[a].runways) {
      r.airport = static_cast<int>(a);
      w.runways.push_back(r);
    }
  }
  return w;
}

World makeEstonia() { return makeWorld({makeTallinn(), makeKuressaare()}); }

RunwayAxes::RunwayAxes(const LocalFrame& frame, const Runway& runway)
    : threshold_(frame.toEnu(runway.threshold)) {
  const double c = runway.trueCourseDeg * kDegToRad;
  dirE_ = std::sin(c);
  dirN_ = std::cos(c);
  const Enu end = frame.toEnu(runway.end);
  const Enu start = frame.toEnu(runway.start);
  landingDistanceM_ = (end.e - threshold_.e) * dirE_ + (end.n - threshold_.n) * dirN_;
  displacementM_ = (threshold_.e - start.e) * dirE_ + (threshold_.n - start.n) * dirN_;
}

RunwayPoint RunwayAxes::fromEnu(const Enu& p) const {
  const double de = p.e - threshold_.e, dn = p.n - threshold_.n;
  // Right of course is the course direction rotated +90 degrees: (dirN, -dirE) in (e, n).
  return {de * dirE_ + dn * dirN_, de * dirN_ - dn * dirE_, p.u - threshold_.u};
}

Enu RunwayAxes::toEnu(const RunwayPoint& p) const {
  return {threshold_.e + p.x * dirE_ + p.y * dirN_, threshold_.n + p.x * dirN_ - p.y * dirE_,
          threshold_.u + p.z};
}

}  // namespace a320
