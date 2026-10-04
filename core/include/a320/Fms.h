#pragma once

#include <string>

namespace a320 {

// What the crew entered in the MCDU: departure and arrival, radio tuning and performance data.
// Indices refer to Airport::runways (one per landing direction); -1 = none.
struct Fms {
  int depRunway = -1;
  int arrRunway = -1;      // inserted arrival approach (the runway's ILS)
  int manualIls = -1;      // RAD NAV entry; -1 = auto-tuned
  double manualCrsMagDeg = -1.0;
  bool flown = false;      // airborne after takeoff: auto-tuning moves to the arrival ILS

  // INIT
  std::string flightNumber;
  int costIndex = -1;
  int cruiseFl = -1;

  // PERF TAKE OFF (0 = not entered)
  double v1Kt = 0.0, vrKt = 0.0, v2Kt = 0.0;
  int flexTempC = -100;    // -100 = not entered
  std::string flapsThs;    // "1/UP1.0"
  int transAltFt = 5000;
  int thrRedFt = 1500, accFt = 1500;

  // PERF APPR
  int qnhHpa = 0;          // 0 = not entered
  int tempC = -100;
  int windDirMag = -1, windKt = -1;
  int transFl = 50;
  double vappKt = 0.0;     // 0 = computed
  int dhFt = -1;           // -1 = none
  int mdaFt = -1;
  bool conf3 = false;      // landing CONF 3 instead of FULL

  // The ILS the FMGC tunes: the crew's manual choice, else the departure runway's until airborne
  // with an arrival inserted, then the arrival's.
  int autoIls() const { return flown && arrRunway >= 0 ? arrRunway : depRunway; }
  int tunedIls() const { return manualIls >= 0 ? manualIls : autoIls(); }
};

}  // namespace a320
