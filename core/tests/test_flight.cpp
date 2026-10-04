// End-to-end flights with the JSBSim A320, driven through the C API by a scripted pilot.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "Check.h"
#include "a320/a320_api.h"

namespace {

constexpr double kDt = 1.0 / 120.0;

double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

double wrap180(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

struct Flight {
  A320Sim* sim = nullptr;
  A320Controls c{};
  A320State s{};
  std::vector<std::string> callouts;
  uint32_t lastCalloutSeq = 0;

  explicit Flight(A320Scenario scenario, const char* runway = "26") {
    char err[256] = {0};
    sim = a320_create(A320_DATA_DIR, err, sizeof(err));
    if (!sim) {
      std::printf("  a320_create failed: %s\n", err);
      return;
    }
    int idx = 0;
    for (int i = 0; i < a320_runway_count(sim); ++i) {
      A320RunwayInfo info;
      a320_get_runway(sim, i, &info);
      if (std::strcmp(info.ident, runway) == 0) idx = i;
    }
    a320_reset(sim, scenario, idx);
    a320_get_state(sim, &s);
    a320_get_controls(sim, &c);
    lastCalloutSeq = s.calloutSeq;
  }
  ~Flight() { a320_destroy(sim); }

  // Runs the pilot callback every step until it returns false or the time runs out.
  void fly(double seconds, const std::function<bool()>& pilot) {
    for (double t = 0.0; t < seconds; t += kDt) {
      if (!pilot()) return;
      a320_set_controls(sim, &c);
      a320_update(sim, kDt);
      a320_get_state(sim, &s);
      if (s.calloutSeq != lastCalloutSeq) {
        lastCalloutSeq = s.calloutSeq;
        callouts.push_back(s.callout);
      }
    }
  }

  bool heard(const char* text) const {
    for (const auto& c0 : callouts) if (c0 == text) return true;
    return false;
  }
};

// Position along / across the active runway, from the runway info and flat-world position.
struct RunwayPos {
  double x, y;
};

RunwayPos runwayPos(A320Sim* sim, const A320State& s) {
  A320RunwayInfo r;
  a320_get_runway(sim, s.ilsRunwayIndex, &r);
  const double c = r.gridCourseDeg * 3.14159265358979 / 180.0;
  const double dn = s.northM - r.thresholdNorthM, de = s.eastM - r.thresholdEastM;
  return {de * std::sin(c) + dn * std::cos(c), de * std::cos(c) - dn * std::sin(c)};
}

}  // namespace

TEST(rests_on_runway) {
  Flight f(A320_SCENARIO_RUNWAY);
  if (!f.sim) { CHECK(false); return; }
  f.fly(5.0, [] { return true; });
  std::printf("  at rest: CG %.2f ft above field, RA %.2f ft, pitch %.2f, GS %.2f kt, N1 %.1f, fuel %.0f kg, GW %.0f kg\n",
              f.s.heightAboveFieldM / 0.3048, f.s.radioAltFt, f.s.pitchDeg, f.s.groundSpeedKt, f.s.n1[0], f.s.fuelKg, f.s.grossWeightKg);
  CHECK(f.s.onGround);
  CHECK(f.s.radioAltFt < 2.0);
  CHECK(f.s.groundSpeedKt < 0.5);
  CHECK(f.s.n1[0] > 15.0);  // engines running at idle
  CHECK_NEAR(f.s.headingTrueDeg, 270.2, 0.5);
  CHECK(f.s.pitchLaw == A320_LAW_GROUND);
  CHECK(std::string(a320_flap_config_name(f.s.flapsLever, f.s.onePlusF)) == "1+F");
  CHECK_NEAR(f.s.flapDeg, 10.0, 0.5);
  // Gear lever up on the ground is ignored.
  f.c.gearDown = 0;
  f.fly(2.0, [] { return true; });
  CHECK(f.s.gearLeverDown == 1);
  CHECK_NEAR(f.s.gearPos, 1.0, 1e-6);
}

TEST(nosewheel_steers_with_pedals) {
  Flight f(A320_SCENARIO_RUNWAY);
  if (!f.sim) { CHECK(false); return; }
  f.c.parkBrake = 0;
  double turned = 0.0, prevHdg = f.s.headingTrueDeg;
  f.fly(12.0, [&] {
    f.c.thrustLever = f.s.groundSpeedKt < 10.0 ? 0.3 : 0.0;
    f.c.pedals = 0.5;
    turned += wrap180(f.s.headingTrueDeg - prevHdg);
    prevHdg = f.s.headingTrueDeg;
    return true;
  });
  std::printf("  taxi with right pedal: turned %.1f deg, GS %.1f kt\n", turned, f.s.groundSpeedKt);
  CHECK(turned > 20.0);
}

TEST(pause_freezes_simulation) {
  Flight f(A320_SCENARIO_FINAL_10NM);
  if (!f.sim) { CHECK(false); return; }
  f.fly(1.0, [] { return true; });
  const A320State before = f.s;
  a320_set_paused(f.sim, 1);
  f.fly(3.0, [] { return true; });
  CHECK(f.s.paused == 1);
  CHECK_NEAR(f.s.simTimeS, before.simTimeS, 1e-9);
  CHECK_NEAR(f.s.northM, before.northM, 1e-9);
  CHECK_NEAR(f.s.eastM, before.eastM, 1e-9);
  a320_set_paused(f.sim, 0);
  a320_set_sim_rate(f.sim, 2.0);
  f.fly(1.0, [] { return true; });
  CHECK_NEAR(f.s.simTimeS, before.simTimeS + 2.0, 0.02);
}

TEST(takeoff_and_climb) {
  Flight f(A320_SCENARIO_RUNWAY);
  if (!f.sim) { CHECK(false); return; }
  A320RunwayInfo rw;
  a320_get_runway(f.sim, f.s.ilsRunwayIndex, &rw);

  f.c.parkBrake = 0;
  f.c.thrustLever = 1.0;  // TOGA
  const double v1 = f.s.v1Kt, vr = f.s.vrKt, v2 = f.s.v2Kt;
  std::string calls;
  uint32_t calloutSeq = f.s.calloutSeq;
  double maxGroundPitch = 0.0, liftoffX = 0.0, liftoffY = 0.0, liftoffIas = 0.0;
  bool airborne = false;
  double t = 0.0;
  f.fly(90.0, [&] {
    t += kDt;
    if (f.s.calloutSeq != calloutSeq) {
      calloutSeq = f.s.calloutSeq;
      calls += std::string(f.s.callout) + ", ";
    }
    const RunwayPos p = runwayPos(f.sim, f.s);
    if (f.s.onGround) {
      maxGroundPitch = std::fmax(maxGroundPitch, f.s.pitchDeg);
      f.c.pedals = clampd(-0.02 * p.y - 0.15 * wrap180(f.s.headingTrueDeg - rw.trueCourseDeg), -1, 1);
      // Rotate at VR towards 10 degrees, held until the aircraft flies off.
      f.c.stickPitch = f.s.iasKt > vr ? clampd(0.12 * (10.0 - f.s.pitchDeg), -0.3, 0.7) : 0.0;
    } else {
      if (!airborne) {
        airborne = true;
        liftoffX = p.x;
        liftoffY = p.y;
        liftoffIas = f.s.iasKt;
      }
      f.c.pedals = 0.0;
      // Simplified SRS: 15 degrees, then pitch for 180 kt once thrust is back to CL.
      const double pitchCmd = f.s.radioAltFt > 1500.0 ? clampd(10.0 + 0.3 * (f.s.iasKt - 180.0), 5.0, 15.0) : 15.0;
      f.c.stickPitch = clampd(0.08 * (pitchCmd - f.s.pitchDeg), -0.5, 0.5);
      f.c.stickRoll = clampd(-0.05 * f.s.bankDeg, -0.3, 0.3);
      if (f.s.verticalSpeedFpm > 500.0 && f.s.radioAltFt > 50.0) f.c.gearDown = 0;
      if (f.s.radioAltFt > 1500.0) f.c.thrustLever = 0.75;  // CL
    }
    return true;
  });

  std::printf("  liftoff %.0f m past threshold (rwy %.0f m), %.1f m off centreline, IAS %.0f kt; "
              "max ground pitch %.1f; after 90 s: RA %.0f ft IAS %.0f kt VS %.0f fpm gear %.2f law %d\n",
              liftoffX, rw.landingDistanceM, liftoffY, liftoffIas, maxGroundPitch, f.s.radioAltFt,
              f.s.iasKt, f.s.verticalSpeedFpm, f.s.gearPos, f.s.pitchLaw);
  std::printf("  V1 %.0f VR %.0f V2 %.0f; calls: %s\n", v1, vr, v2, calls.c_str());
  CHECK(v1 > 110.0 && v1 < vr && vr < v2 && v2 < 170.0);
  CHECK(calls.rfind("ONE HUNDRED KNOTS, V ONE, ROTATE, POSITIVE CLIMB, ", 0) == 0);
  CHECK(airborne);
  CHECK(liftoffX < rw.landingDistanceM - 500.0);
  CHECK(std::fabs(liftoffY) < 10.0);
  CHECK(maxGroundPitch < 11.5);  // A320 tail strike attitude with the gear compressed
  CHECK(f.s.radioAltFt > 2000.0);
  CHECK(f.s.gearPos < 0.01);
  CHECK(f.s.pitchLaw == A320_LAW_FLIGHT);
  CHECK(f.s.iasKt > 150.0 && f.s.iasKt < 280.0);
}

TEST(normal_law_holds_path_and_bank) {
  Flight f(A320_SCENARIO_FINAL_10NM);
  if (!f.sim) { CHECK(false); return; }
  f.fly(2.0, [] { return true; });
  const double fpa0 = f.s.flightPathDeg;
  f.fly(20.0, [] { return true; });
  std::printf("  neutral stick: FPA %.2f -> %.2f deg, IAS %.0f, pitch %.1f\n", fpa0,
              f.s.flightPathDeg, f.s.iasKt, f.s.pitchDeg);
  CHECK_NEAR(f.s.flightPathDeg, fpa0, 0.6);

  // Roll in with half stick, release: bank holds.
  f.fly(2.0, [&] { f.c.stickRoll = 0.5; return true; });
  const double rolledBank = f.s.bankDeg;
  f.fly(8.0, [&] { f.c.stickRoll = 0.0; return true; });
  std::printf("  bank after roll-in %.1f, after 8 s neutral %.1f, FPA %.2f\n", rolledBank,
              f.s.bankDeg, f.s.flightPathDeg);
  CHECK(rolledBank > 8.0);
  CHECK(f.s.bankDeg > rolledBank - 3.0 && f.s.bankDeg < rolledBank + 6.0);

  // Beyond 33 degrees the aircraft rolls back when the stick is released.
  f.fly(6.0, [&] { f.c.stickRoll = 1.0; return true; });
  const double steepBank = f.s.bankDeg;
  f.fly(10.0, [&] { f.c.stickRoll = 0.0; return true; });
  std::printf("  steep bank %.1f -> released %.1f\n", steepBank, f.s.bankDeg);
  CHECK(steepBank > 45.0 && steepBank < 69.0);
  CHECK_NEAR(f.s.bankDeg, 33.0, 3.0);
}

// Scripted pilot: ILS tracking, speed on the thrust levers, manual flare and rollout.
static void flyIlsLanding(A320Scenario scenario, double* touchdownOut = nullptr, const char* runway = "26") {
  Flight f(scenario, runway);
  if (!f.sim) { CHECK(false); return; }
  A320RunwayInfo rw;
  a320_get_runway(f.sim, f.s.ilsRunwayIndex, &rw);

  double maxLocDotsInside6 = 0.0, maxGsDotsInside6 = 0.0, lever = f.s.thrustLever, speedInt = 0.0;
  bool touchedDown = false, stopped = false, sawWarning = false;
  double stopX = 0.0, stopY = 0.0;
  const bool trace = std::getenv("A320_TRACE") != nullptr;
  double traceT = 0.0;
  f.fly(600.0, [&] {
    const A320State& s = f.s;
    traceT += kDt;
    if (trace && (s.radioAltFt < 120.0 && !s.onGround ? std::fmod(traceT, 0.25) < kDt : std::fmod(traceT, 4.0) < kDt))
      std::printf("    t %5.1f dme %5.2f RA %6.0f gs %5.2f loc %5.2f fpa %5.2f ias %5.1f vls %5.1f lever %.2f "
                  "ths %5.2f elev %5.2f pitch %5.2f law %d onGnd %d\n", traceT, s.dmeNm, s.radioAltFt,
                  s.gsDots, s.locDots, s.flightPathDeg, s.iasKt, s.vlsKt, s.thrustLever, s.thsDeg,
                  s.elevatorNorm, s.pitchDeg, s.pitchLaw, s.onGround);
    const RunwayPos p = runwayPos(f.sim, s);
    if (s.touchdownSeq > 0) touchedDown = true;

    if (!touchedDown) {
      if (s.dmeNm < 6.5 && s.dmeNm > 2.5) {
        maxLocDotsInside6 = std::fmax(maxLocDotsInside6, std::fabs(s.locDots));
        maxGsDotsInside6 = std::fmax(maxGsDotsInside6, std::fabs(s.gsDots));
      }
      if (s.warnings & ~0u) sawWarning = sawWarning || (s.radioAltFt > 100.0);
      if (s.dmeNm < 7.0) f.c.flapsLever = 4;

      // Lateral: aim the track at the localizer, bank to fly it.
      const double trackCmd = rw.trueCourseDeg + clampd(12.0 * s.locDots, -25.0, 25.0);
      const double bankCmd = clampd(2.5 * wrap180(trackCmd - s.trackTrueDeg), -20.0, 20.0);
      f.c.stickRoll = clampd(0.06 * (bankCmd - s.bankDeg), -0.6, 0.6);

      if (s.radioAltFt > 45.0) {
        const double fpaCmd = -rw.glideslopeDeg + clampd(0.8 * s.gsDots, -1.5, 1.5);
        f.c.stickPitch = clampd(0.25 * (fpaCmd - s.flightPathDeg), -0.4, 0.4);
        const double vapp = s.vlsKt + 5.0;
        speedInt = clampd(speedInt + 0.004 * (vapp - s.iasKt) * kDt, -0.3, 0.6);
        lever = clampd(0.3 + speedInt + 0.03 * (vapp - s.iasKt), 0.0, 0.88);
        f.c.thrustLever = lever;
      } else {
        // Flare as a pilot does: from 30 ft ease the stick back for about +3 degrees of pitch
        // (the flare law adds 10 degrees per full stick), more if still sinking fast.
        const double flare = s.radioAltFt < 30.0 ? 0.3 + clampd(-0.0004 * (s.verticalSpeedFpm + 300.0), 0.0, 0.2) : 0.0;
        f.c.stickPitch = flare;
        if (s.radioAltFt < 20.0) f.c.thrustLever = 0.0;
        f.c.pedals = clampd(-0.15 * wrap180(s.headingTrueDeg - rw.trueCourseDeg), -0.3, 0.3);
      }
    } else {
      // Rollout: lower the nose, reversers, brakes, track the centreline.
      f.c.stickPitch = 0.0;
      f.c.stickRoll = 0.0;
      f.c.speedbrake = 1.0;
      f.c.pedals = clampd(-0.03 * p.y - 0.2 * wrap180(s.headingTrueDeg - rw.trueCourseDeg), -1, 1);
      f.c.reverse = s.groundSpeedKt > 70.0 ? 1 : 0;
      f.c.thrustLever = s.groundSpeedKt > 70.0 ? 0.7 : 0.0;
      const double brake = s.groundSpeedKt < 120.0 ? 0.5 : 0.0;
      f.c.brakeLeft = f.c.brakeRight = brake;
      if (s.groundSpeedKt < 1.0) {
        stopped = true;
        stopX = p.x;
        stopY = p.y;
        return false;
      }
    }
    return true;
  });

  std::printf("  LOC max %.2f dots, G/S max %.2f dots (6.5-2.5 NM); touchdown %.0f m past threshold, "
              "%.1f m off centreline, %.0f fpm; stopped %d at %.0f m (%.1f m off)\n",
              maxLocDotsInside6, maxGsDotsInside6, f.s.touchdownDistanceM, f.s.touchdownCenterlineM,
              f.s.touchdownFpm, stopped, stopX, stopY);
  std::printf("  callouts:");
  for (const auto& c : f.callouts) std::printf(" %s,", c.c_str());
  std::printf("\n");

  CHECK(maxLocDotsInside6 < 0.5);
  CHECK(maxGsDotsInside6 < 0.6);
  CHECK(!sawWarning);
  CHECK(touchedDown);
  CHECK(f.s.touchdownDistanceM > 100.0 && f.s.touchdownDistanceM < 900.0);
  CHECK(std::fabs(f.s.touchdownCenterlineM) < 8.0);
  CHECK(f.s.touchdownFpm > -600.0);
  CHECK(stopped);
  CHECK(stopX < rw.landingDistanceM);
  CHECK(std::fabs(stopY) < rw.widthM / 2.0);
  CHECK(f.heard("FIFTY"));
  CHECK(f.heard("RETARD"));
  if (touchdownOut) *touchdownOut = f.s.touchdownDistanceM;
}

TEST(ils_approach_and_landing_from_4nm) { flyIlsLanding(A320_SCENARIO_FINAL_4NM); }

// Kuressaare, 120 ft lower than Tallinn and 95 NM away: the ground, ILS and radio altimeter are
// its own.
TEST(ils_landing_at_kuressaare_17) {
  flyIlsLanding(A320_SCENARIO_FINAL_10NM, nullptr, "17");
  Flight f(A320_SCENARIO_RUNWAY, "17");
  if (!f.sim) { CHECK(false); return; }
  f.fly(3.0, [] { return true; });
  A320AirportInfo eeke;
  CHECK(a320_airport_count(f.sim) == 2 && a320_get_airport(f.sim, 1, &eeke) && std::strcmp(eeke.icao, "EEKE") == 0);
  std::printf("  on runway 17: nearest %d, RA %.1f ft, height above Tallinn's field %.1f m (EEKE %.1f m)\n",
              f.s.nearestAirport, f.s.radioAltFt, f.s.heightAboveFieldM, eeke.elevationM);
  CHECK(f.s.nearestAirport == 1 && f.s.onGround && f.s.radioAltFt < 3.0);
  CHECK(std::fabs(f.s.heightAboveFieldM - eeke.elevationM) < 6.0);
  CHECK(std::fabs(std::remainder(f.s.gridHeadingDeg - f.s.headingTrueDeg, 360.0)) > 1.0);
}

TEST(ils_approach_and_landing_from_10nm) {
  flyIlsLanding(A320_SCENARIO_FINAL_10NM);
}
