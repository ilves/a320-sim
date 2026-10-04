// Autopilot and autothrust flown on the JSBSim A320 through the C API.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>

#include "Check.h"
#include "a320/a320_api.h"

namespace {

constexpr double kDt = 1.0 / 120.0;

double wrap180(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

struct ApFlight {
  A320Sim* sim = nullptr;
  A320Controls c{};
  A320State s{};
  A320RunwayInfo rw{};
  double magVar = 0.0;
  int maxLat = 0, maxVert = 0;
  bool sawLat[5] = {}, sawVert[12] = {};

  explicit ApFlight(A320Scenario scenario) {
    char err[256] = {0};
    sim = a320_create(A320_DATA_DIR, err, sizeof(err));
    if (!sim) { std::printf("  create failed: %s\n", err); return; }
    int idx = 0;
    for (int i = 0; i < a320_runway_count(sim); ++i) {
      A320RunwayInfo info;
      a320_get_runway(sim, i, &info);
      if (std::strcmp(info.ident, "26") == 0) idx = i;
    }
    a320_reset(sim, scenario, idx);
    a320_get_runway(sim, idx, &rw);
    magVar = a320_magnetic_variation_deg(sim);
    a320_get_state(sim, &s);
    a320_get_controls(sim, &c);
    c.thrustLever = 0.75;  // CL detent: autothrust range
  }
  ~ApFlight() { a320_destroy(sim); }

  double courseMag() const { return rw.trueCourseDeg - magVar; }
  void targets(double spd, double hdg, double alt, double vs) { a320_fcu_set_targets(sim, spd, hdg, alt, vs); refresh(); }
  void fcu(A320FcuCommand cmd) { a320_fcu_command(sim, cmd); refresh(); }
  void refresh() { a320_get_state(sim, &s); }

  void fly(double seconds, const std::function<bool()>& pilot = [] { return true; }) {
    for (double t = 0.0; t < seconds; t += kDt) {
      if (!pilot()) return;
      a320_set_controls(sim, &c);
      a320_update(sim, kDt);
      refresh();
      sawLat[s.latMode] = true;
      sawVert[s.vertMode] = true;
    }
  }
};

}  // namespace

TEST(ap_climb_turn_and_descend) {
  ApFlight f(A320_SCENARIO_FINAL_10NM);
  if (!f.sim) { CHECK(false); return; }
  f.c.gearDown = 0;
  f.c.flapsLever = 1;
  f.fcu(A320_FCU_ATHR);
  f.targets(200, f.courseMag(), 5000, 0);
  f.fcu(A320_FCU_AP1);
  CHECK(f.s.apEngaged == 1);
  f.fcu(A320_FCU_ALT_PULL);
  CHECK(f.s.vertMode == A320_VERT_OP_CLB);
  f.fly(240.0);
  std::printf("  OP CLB -> alt %.0f ft (target 5000), IAS %.0f (200), mode %s, A/THR %s\n", f.s.altitudeFt,
              f.s.iasKt, a320_vert_mode_name(f.s.vertMode), a320_athr_mode_name(f.s.athrMode));
  CHECK(f.sawVert[A320_VERT_ALT_STAR]);
  CHECK(f.s.vertMode == A320_VERT_ALT);
  CHECK_NEAR(f.s.altitudeFt, 5000.0, 60.0);
  CHECK_NEAR(f.s.iasKt, 200.0, 8.0);
  CHECK(f.s.athrMode == A320_ATHR_SPEED);

  const double newHdg = std::fmod(f.courseMag() + 270.0, 360.0);  // 90 degrees left
  f.targets(200, newHdg, 5000, 0);
  f.fcu(A320_FCU_HDG_PULL);
  f.fly(70.0);
  const double hdgMag = f.s.headingTrueDeg - f.magVar;
  std::printf("  HDG -> heading %.1f (target %.0f), bank %.1f, alt %.0f\n", std::fmod(hdgMag + 360.0, 360.0), newHdg,
              f.s.bankDeg, f.s.altitudeFt);
  CHECK(std::fabs(wrap180(hdgMag - newHdg)) < 3.0);
  CHECK_NEAR(f.s.altitudeFt, 5000.0, 100.0);

  f.targets(200, newHdg, 3000, -1500);
  f.fcu(A320_FCU_VS_PULL);
  CHECK(f.s.vertMode == A320_VERT_VS);
  f.targets(200, newHdg, 3000, -1500);
  f.fly(150.0);
  std::printf("  V/S -1500 -> alt %.0f (3000), mode %s, IAS %.0f\n", f.s.altitudeFt, a320_vert_mode_name(f.s.vertMode), f.s.iasKt);
  CHECK(f.s.vertMode == A320_VERT_ALT);
  CHECK_NEAR(f.s.altitudeFt, 3000.0, 60.0);
  CHECK_NEAR(f.s.iasKt, 200.0, 10.0);
}

TEST(ap_takeover_disconnects) {
  ApFlight f(A320_SCENARIO_FINAL_10NM);
  if (!f.sim) { CHECK(false); return; }
  f.fcu(A320_FCU_AP1);
  const uint32_t seq = f.s.apDisconnectSeq;
  CHECK(f.s.apEngaged == 1);
  f.fly(2.0, [&] { f.c.stickPitch = 0.8; return true; });
  CHECK(f.s.apEngaged == 0);
  CHECK(f.s.apDisconnectSeq == seq + 1);
}

TEST(ap_loc_intercept_from_heading) {
  ApFlight f(A320_SCENARIO_FINAL_10NM);
  if (!f.sim) { CHECK(false); return; }
  f.fcu(A320_FCU_ATHR);
  // Engaging syncs the heading target to the current heading, so select the new one after.
  f.fcu(A320_FCU_AP1);
  f.targets(170, std::fmod(f.courseMag() + 40.0, 360.0), 3000, 0);
  f.fcu(A320_FCU_HDG_PULL);
  f.fcu(A320_FCU_ALT_PULL);
  f.fly(45.0);
  const double offDots = f.s.locDots;
  // Turn back towards the localizer at 30 degrees and arm the approach.
  f.targets(170, std::fmod(f.courseMag() - 30.0 + 360.0, 360.0), 3000, 0);
  f.fcu(A320_FCU_HDG_PULL);
  f.fcu(A320_FCU_APPR);
  CHECK(f.s.armed & A320_ARMED_LOC);
  f.fly(150.0);
  std::printf("  displaced to %.2f dots, then LOC %s at %.2f dots, track err %.1f\n", offDots,
              a320_lat_mode_name(f.s.latMode), f.s.locDots, wrap180(f.s.trackTrueDeg - f.rw.trueCourseDeg));
  CHECK(std::fabs(offDots) > 1.0);
  CHECK(f.sawLat[A320_LAT_LOC_STAR]);
  CHECK(f.s.latMode == A320_LAT_LOC);
  CHECK(std::fabs(f.s.locDots) < 0.3);
}

TEST(ap_autoland) {
  ApFlight f(A320_SCENARIO_FINAL_10NM);
  if (!f.sim) { CHECK(false); return; }
  f.fcu(A320_FCU_ATHR);
  f.targets(f.s.vlsKt + 5.0, f.courseMag(), 3000, 0);
  f.fcu(A320_FCU_APPR);
  f.fcu(A320_FCU_AP1);
  bool touched = false, stopped = false;
  double stopY = 0.0, stopX = 0.0, maxGs = 0.0;
  const double c = f.rw.trueCourseDeg * 3.14159265358979 / 180.0;
  f.fly(600.0, [&] {
    const A320State& s = f.s;
    if (!s.onGround && s.dmeNm < 7.0 && f.c.flapsLever != 4) f.c.flapsLever = 4;
    if (!s.onGround) {
      a320_fcu_set_targets(f.sim, s.vlsKt + 5.0, f.courseMag(), 3000, 0);
      if (s.dmeNm < 6.5 && s.dmeNm > 2.5) maxGs = std::fmax(maxGs, std::fabs(s.gsDots));
    }
    if (std::getenv("A320_TRACE") && std::fmod(s.simTimeS, 4.0) < kDt)
      std::printf("    t %5.1f dme %5.2f RA %5.0f gs %5.2f loc %5.2f fpa %5.2f ias %5.1f vls %5.1f thr %.2f flaps %.0f %s/%s/%s\n",
                  s.simTimeS, s.dmeNm, s.radioAltFt, s.gsDots, s.locDots, s.flightPathDeg, s.iasKt, s.vlsKt,
                  s.n1[0], s.flapDeg, a320_athr_mode_name(s.athrMode), a320_vert_mode_name(s.vertMode), a320_lat_mode_name(s.latMode));
    if (s.touchdownSeq > 0) touched = true;
    if (touched) {
      // Pilot after an autoland: idle, reversers, brakes; the AP keeps the centreline.
      f.c.thrustLever = s.groundSpeedKt > 70.0 ? 0.7 : 0.0;
      f.c.reverse = s.groundSpeedKt > 70.0 ? 1 : 0;
      if (!f.c.reverse && s.groundSpeedKt > 70.0) f.c.thrustLever = 0.0;
      f.c.speedbrake = 1.0;
      f.c.brakeLeft = f.c.brakeRight = s.groundSpeedKt < 120.0 ? 0.5 : 0.0;
      if (s.groundSpeedKt < 1.0) {
        const double dn = s.northM - f.rw.thresholdNorthM, de = s.eastM - f.rw.thresholdEastM;
        stopX = de * std::sin(c) + dn * std::cos(c);
        stopY = de * std::cos(c) - dn * std::sin(c);
        stopped = true;
        return false;
      }
    }
    return true;
  });
  // Reversers are selected at idle in the same step as touchdown, so set the lever first.
  std::printf("  modes: LOC %d G/S %d LAND %d FLARE %d ROLLOUT %d; G/S max %.2f dots; touchdown %.0f m, %.1f m, %.0f fpm; "
              "stop %.0f m at %.1f m\n", f.sawLat[A320_LAT_LOC], f.sawVert[A320_VERT_GS], f.sawVert[A320_VERT_LAND],
              f.sawVert[A320_VERT_FLARE], f.sawLat[A320_LAT_ROLLOUT], maxGs, f.s.touchdownDistanceM,
              f.s.touchdownCenterlineM, f.s.touchdownFpm, stopX, stopY);
  CHECK(f.sawLat[A320_LAT_LOC] && f.sawVert[A320_VERT_GS] && f.sawVert[A320_VERT_LAND]);
  CHECK(f.sawVert[A320_VERT_FLARE] && f.sawLat[A320_LAT_ROLLOUT]);
  CHECK(maxGs < 0.6);
  CHECK(touched);
  CHECK(f.s.touchdownDistanceM > 100.0 && f.s.touchdownDistanceM < 900.0);
  CHECK(std::fabs(f.s.touchdownCenterlineM) < 8.0);
  CHECK(f.s.touchdownFpm > -600.0);
  CHECK(stopped);
  CHECK(stopX < f.rw.landingDistanceM);
  CHECK(std::fabs(stopY) < 10.0);
}

TEST(athr_vls_protection) {
  ApFlight f(A320_SCENARIO_APPROACH);
  const uint32_t seq = f.s.hintSeq;
  f.targets(100.0, f.s.fcuHdgMagDeg, f.s.fcuAltFt, 0.0);  // far below VLS
  CHECK(f.s.hintSeq == seq + 1 && std::strstr(f.s.hint, "VLS") != nullptr);
  double minIas = 1000.0;
  f.fly(150.0, [&] {
    if (f.s.simTimeS > 90.0) minIas = std::fmin(minIas, f.s.iasKt - f.s.vlsKt);
    return true;
  });
  std::printf("  SPD 100 selected: IAS %.0f kt, VLS %.0f kt, lowest margin over VLS once settled %.1f kt, mode %s\n", f.s.iasKt,
              f.s.vlsKt, minIas, a320_athr_mode_name(f.s.athrMode));
  CHECK(minIas > -3.0);
  CHECK(f.s.iasKt < f.s.vlsKt + 8.0);
  CHECK(f.s.athrMode == A320_ATHR_SPEED);
}

TEST(alpha_floor_and_toga_lock) {
  ApFlight f(A320_SCENARIO_APPROACH);
  f.fcu(A320_FCU_AP1);   // AP off
  f.fcu(A320_FCU_ATHR);  // A/THR off
  f.c.thrustLever = 0.0;  // idle: the aircraft slows down
  f.c.stickPitch = 1.0;   // full back stick: alpha protection, then alpha floor
  bool floorSeen = false;
  double floorN1 = 0.0, maxAlpha = 0.0;
  f.fly(90.0, [&] {
    if (std::getenv("A320_TRACE") && std::fmod(f.s.simTimeS, 3.0) < kDt)
      std::printf("    t %5.1f ias %5.1f alpha %5.1f pitch %5.1f fpa %5.1f n1 %5.1f elev %5.2f ths %5.1f %s\n", f.s.simTimeS, f.s.iasKt,
                  f.s.alphaDeg, f.s.pitchDeg, f.s.flightPathDeg, f.s.n1[0], f.s.elevatorNorm, f.s.thsDeg, a320_athr_mode_name(f.s.athrMode));
    maxAlpha = std::fmax(maxAlpha, f.s.alphaDeg);
    if (f.s.athrMode == A320_ATHR_AFLOOR) {
      floorSeen = true;
      floorN1 = std::fmax(floorN1, f.s.n1[0]);
    }
    return true;
  });
  std::printf("  full back stick at idle: max alpha %.1f deg, A.FLOOR %d, N1 %.0f %%, hint: %s\n", maxAlpha, floorSeen,
              floorN1, f.s.hint);
  CHECK(floorSeen && f.s.athrEngaged);
  CHECK(floorN1 > 80.0);
  CHECK(std::strstr(f.s.hint, "ALPHA FLOOR") != nullptr);
  CHECK(maxAlpha < 16.2);  // alpha max 15 plus a small overshoot, below the model's stall

  // Recovery: ease the stick forward out of alpha protection. TOGA stays locked until A/THR is
  // disconnected.
  f.c.stickPitch = -0.3;
  f.fly(30.0, [&] { return f.s.athrMode != A320_ATHR_TOGA_LK; });
  f.fly(6.0);
  f.c.stickPitch = 0.0;
  CHECK(f.s.athrMode == A320_ATHR_TOGA_LK);
  f.fly(5.0);
  CHECK(f.s.athrMode == A320_ATHR_TOGA_LK && f.s.n1[0] > 80.0);
  f.fcu(A320_FCU_ATHR);
  f.fly(15.0);
  std::printf("  after A/THR off: mode %s, N1 %.0f %% (levers at idle)\n", a320_athr_mode_name(f.s.athrMode), f.s.n1[0]);
  CHECK(!f.s.athrEngaged && f.s.athrMode == A320_ATHR_OFF);
  CHECK(f.s.n1[0] < 50.0);
}
