#pragma once

#include <string>
#include <vector>

#include "a320/Geo.h"

namespace a320 {

// One ILS as published in the AIP (AD 2.19). The DME is co-located with the glide path antenna.
struct IlsSpec {
  std::string ident;            // "ILK"
  double frequencyMHz = 0.0;    // localizer; the glide path is paired with it
  double courseMagDeg = 0.0;    // published localizer course
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
  IlsSpec ils;            // ident empty: no ILS on this runway
  int airport = 0;        // index into World::airports
};

struct Airport {
  std::string icao;
  std::string name;
  std::string city;  // as ATC says it: "Tallinn"
  GeoPos reference;  // aerodrome reference point at field elevation; its ILS geometry uses a frame here
  double magneticVariationDeg = 0.0;  // east positive: magnetic = true - variation
  std::vector<Runway> runways;

  const Runway* find(const std::string& ident) const;
};

// Lennart Meri Tallinn (EETN), runway 08/26, from OurAirports (public domain); ILS data from
// the Estonian eAIP, EETN AD 2.19 (AIRAC 2026-10-01).
Airport makeTallinn();
// Kuressaare (EEKE), runway 17/35, ILS 17, from the Estonian eAIP, EEKE AD 2.12 and 2.19.
Airport makeKuressaare();
// The other public airports (eAIP AD 2): Tartu (EETU, ILS 26), Parnu (EEPU), Kardla (EEKA),
// and the grass strips of Ruhnu (EERU) and Kihnu (EEKU).
Airport makeTartu();
Airport makeParnu();
Airport makeKardla();
Airport makeRuhnu();
Airport makeKihnu();

// Every airport of the sim. The flat world (what the front end renders, the aircraft's north and
// east metres) is the first airport's tangent plane; runways are listed together, each knowing
// its airport.
struct World {
  std::vector<Airport> airports;
  std::vector<Runway> runways;
  GeoPos reference;

  const Airport& airportOf(int runway) const { return airports[static_cast<size_t>(runways[static_cast<size_t>(runway)].airport)]; }
  int findRunway(const std::string& ident) const;  // the first with that ident, -1 if none
  int findRunway(int airport, const std::string& ident) const;
  int findAirport(const std::string& icao) const;
  // The airport closest to a point of the flat world (metres north/east).
  int nearestAirport(double northM, double eastM) const;
  std::vector<double> airportNorthM, airportEastM;  // reference points in the flat world
};

World makeWorld(std::vector<Airport> airports);
World makeEstonia();  // EETN, EEKE, EETU, EEPU, EEKA, EERU, EEKU (the order is the API's airport index)

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
