// The flight plan's route (real FRA points, the ILS transition) and NAV flying it, through the C API.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "Check.h"
#include "a320/a320_api.h"

namespace {

constexpr double kDt = 1.0 / 120.0;

double wrap180(double deg) {
  while (deg > 180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}

int runway(A320Sim* sim, const char* icao, const char* ident) {
  for (int i = 0; i < a320_runway_count(sim); ++i) {
    A320RunwayInfo r;
    a320_get_runway(sim, i, &r);
    if (std::strcmp(r.icao, icao) == 0 && std::strcmp(r.ident, ident) == 0) return i;
  }
  return -1;
}

std::vector<A320Waypoint> route(A320Sim* sim) {
  std::vector<A320Waypoint> out(static_cast<size_t>(a320_route_count(sim)));
  for (size_t i = 0; i < out.size(); ++i) a320_get_waypoint(sim, static_cast<int>(i), &out[i]);
  return out;
}

A320State state(A320Sim* sim) {
  A320State s;
  a320_get_state(sim, &s);
  return s;
}

void printRoute(const std::vector<A320Waypoint>& r) {
  std::printf("  ");
  for (const A320Waypoint& w : r) std::printf("%s %03.0f/%.0f  ", w.ident, w.legCourseMagDeg, w.legNm);
  std::printf("\n");
}

}  // namespace

TEST(route_from_departure_and_arrival) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  const int dep = runway(sim, "EETN", "26"), arr = runway(sim, "EEKE", "17");
  a320_start_flight(sim, A320_SCENARIO_RUNWAY, dep, arr, 0.0, A320_PLAN_ROUTE);
  const std::vector<A320Waypoint> r = route(sim);
  printRoute(r);
  CHECK(r.size() >= 6);
  if (r.size() < 6) { a320_destroy(sim); return; }
  // The departure runway, the climb on its track, a real departure point, then the ILS transition.
  CHECK(std::strcmp(r[0].ident, "EETN26") == 0 && r[0].kind == A320_WPT_RUNWAY);
  CHECK(std::strcmp(r[1].ident, "(1630)") == 0 && r[1].kind == A320_WPT_ALTITUDE && r[1].altFt == 1630);
  CHECK(std::strcmp(r[2].ident, "LONSA") == 0);  // eAIP ENR 4.4: FRA (D) (EETN)
  CHECK(std::strcmp(r.back().ident, "RW17") == 0 && r.back().kind == A320_WPT_RUNWAY);
  const A320Waypoint& ff = r[r.size() - 2];
  const A320Waypoint& cf = r[r.size() - 3];
  CHECK(std::strcmp(ff.ident, "FF17") == 0 && std::strcmp(cf.ident, "CF17") == 0);
  // FF and RW on the localizer course; CF 4 NM before FF.
  A320RunwayInfo info;
  a320_get_runway(sim, arr, &info);
  const double courseMag = info.trueCourseDeg - 9.0;  // EEKE: 9 E
  CHECK(std::fabs(wrap180(r.back().legCourseMagDeg - courseMag)) < 1.5);
  CHECK(std::fabs(wrap180(ff.legCourseMagDeg - courseMag)) < 1.5 && std::fabs(ff.legNm - 4.0) < 0.1);
  CHECK(cf.altFt == 2100 && ff.altFt == 2100);
  // No long detour: within a third of the direct distance.
  double total = 0.0;
  for (const A320Waypoint& w : r) total += w.legNm;
  const double direct = std::hypot(info.thresholdNorthM - r[0].northM, info.thresholdEastM - r[0].eastM) / 1852.0;
  std::printf("  %.0f NM along the route, %.0f NM direct\n", total, direct);
  CHECK(total < direct * 1.35);
  const A320State s = state(sim);
  CHECK(s.armed & A320_ARMED_NAV);
  CHECK(s.routeActive == 1 && std::strcmp(s.toWaypoint, "(1630)") == 0);

  // A circuit has no route, and nothing is armed.
  a320_start_flight(sim, A320_SCENARIO_RUNWAY, dep, dep, 0.0, A320_PLAN_FULL);
  CHECK(a320_route_count(sim) == 0 && !(state(sim).armed & A320_ARMED_NAV) && state(sim).routeActive == -1);

  // A new arrival typed into the MCDU on the ground makes a new route.
  a320_start_flight(sim, A320_SCENARIO_RUNWAY, dep, dep, 0.0, A320_PLAN_ROUTE);
  for (const char* k = "EETN/EETU"; *k; ++k) a320_mcdu_key(sim, *k);
  a320_mcdu_key(sim, A320_MCDU_INIT);
  a320_mcdu_key(sim, A320_MCDU_LSK1R);
  a320_mcdu_key(sim, A320_MCDU_FPLN);
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 5);  // destination line: LAT REV
  a320_mcdu_key(sim, A320_MCDU_LSK1R);      // ARRIVAL
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 2);  // the first approach
  a320_mcdu_key(sim, A320_MCDU_LSK1R + 5);  // INSERT
  const std::vector<A320Waypoint> tartu = route(sim);
  printRoute(tartu);
  CHECK(tartu.size() >= 5 && std::strncmp(tartu.back().ident, "RW", 2) == 0);
  CHECK(state(sim).armed & A320_ARMED_NAV);
  a320_destroy(sim);
}

// Tallinn 08 to Tartu: NAV from 30 ft along the real points, a downwind and base onto the ILS 26,
// LOC and G/S captured from NAV with APPR armed.
TEST(nav_flies_the_route_onto_the_ils) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_start_flight(sim, A320_SCENARIO_RUNWAY, runway(sim, "EETN", "08"), runway(sim, "EETU", "26"), 0.0, A320_PLAN_FULL);
  const std::vector<A320Waypoint> r = route(sim);
  printRoute(r);
  const int last = static_cast<int>(r.size()) - 1;
  A320Controls c;
  a320_get_controls(sim, &c);
  c.parkBrake = 0;
  c.thrustLever = 1.0;
  A320State s = state(sim);
  int stage = 0, maxActive = 0;
  bool navAt30 = false, apprArmed = false, locFromNav = false;
  double maxXtkNm = 0.0, legTimeS = 0.0, t = 0.0;
  int lastActive = s.routeActive;
  std::string sequenced, worstLeg;
  for (; t < 3000.0 && stage < 4; t += kDt) {
    a320_set_controls(sim, &c);
    a320_update(sim, kDt);
    a320_get_state(sim, &s);
    if (s.routeActive != lastActive) {
      if (s.routeActive >= 0) sequenced += std::string(" ") + s.toWaypoint;
      lastActive = s.routeActive;
      legTimeS = 0.0;
    }
    legTimeS += kDt;
    maxActive = std::max(maxActive, s.routeActive);
    if (stage == 0) {
      // Rotate at VR, gear up, AP1 at 400 ft: climb to FL090 at 250 kt, A/THR in CL.
      c.stickPitch = s.iasKt > s.vrKt && s.radioAltFt < 400.0 ? std::fmax(-0.3, std::fmin(0.7, 0.12 * (12.0 - s.pitchDeg))) : 0.0;
      if (!s.onGround && s.radioAltFt > 30.0 && s.radioAltFt < 200.0 && s.latMode == A320_LAT_NAV) navAt30 = true;
      if (!s.onGround && s.radioAltFt > 400.0) {
        c.stickPitch = 0.0;
        c.gearDown = 0;
        a320_fcu_command(sim, A320_FCU_AP1);
        a320_fcu_set_targets(sim, 250.0, s.fcuHdgMagDeg, 9000.0, 0.0);
        a320_fcu_command(sim, A320_FCU_ALT_PULL);
        stage = 1;
      }
    } else if (stage == 1 && s.radioAltFt > 1500.0) {
      c.thrustLever = 0.75;
      c.flapsLever = 0;
      a320_fcu_command(sim, A320_FCU_ATHR);
      stage = 2;
    } else if (stage == 2) {
      // Down to the intercept altitude 40 NM out, slower by the downwind, APPR on the base leg.
      if (s.routeRemainingNm < 45.0 && s.fcuAltFt > 2300.0) {
        a320_fcu_set_targets(sim, 220.0, s.fcuHdgMagDeg, 2300.0, 0.0);
        a320_fcu_command(sim, A320_FCU_ALT_PULL);
      }
      if (s.routeActive >= last - 3 && !apprArmed) {
        a320_fcu_command(sim, A320_FCU_APPR);
        apprArmed = true;
      }
      if (apprArmed && (s.latMode == A320_LAT_LOC_STAR || s.latMode == A320_LAT_LOC)) {
        locFromNav = true;
        stage = 3;
      }
      // On a leg, after its turn: how far off it the aircraft is.
      if (s.latMode == A320_LAT_NAV && legTimeS > 60.0 && s.toDistanceNm > 3.0 && std::fabs(s.crossTrackNm) > maxXtkNm) {
        maxXtkNm = std::fabs(s.crossTrackNm);
        worstLeg = s.toWaypoint;
      }
    } else if (stage == 3 && (s.vertMode == A320_VERT_GS || s.dmeNm < 4.0)) {
      stage = 4;
    }
  }
  std::printf("  after %.0f s: sequenced%s; NAV at 30 ft %d, max %.2f NM off a leg (to %s), %s/%s at %.1f NM, %.0f ft, IAS %.0f\n",
              t, sequenced.c_str(), navAt30, maxXtkNm, worstLeg.c_str(), a320_lat_mode_name(s.latMode), a320_vert_mode_name(s.vertMode),
              s.dmeNm, s.altitudeFt, s.iasKt);
  CHECK(navAt30);
  CHECK(maxActive >= last - 1);  // every fix up to CF or FF sequenced
  CHECK(maxXtkNm < 0.3);
  CHECK(locFromNav);
  CHECK(s.latMode == A320_LAT_LOC && s.vertMode == A320_VERT_GS);
  CHECK(std::fabs(s.locDots) < 0.3 && std::fabs(s.gsDots) < 0.5);
  a320_destroy(sim);
}

// In the air (20 NM out), HDG push engages NAV straight to the approach.
TEST(hdg_push_engages_nav_in_the_air) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  const int rw = runway(sim, "EETN", "26");
  a320_start_flight(sim, A320_SCENARIO_APPROACH, rw, rw, 20.0, A320_PLAN_FULL);
  A320Controls c;
  a320_get_controls(sim, &c);
  A320State s = state(sim);
  CHECK(s.latMode == A320_LAT_HDG && std::strcmp(s.toWaypoint, "CF26") == 0);
  a320_fcu_command(sim, A320_FCU_HDG_PUSH);
  s = state(sim);
  CHECK(s.latMode == A320_LAT_NAV);
  a320_fcu_command(sim, A320_FCU_APPR);
  double t = 0.0;
  for (; t < 400.0 && s.latMode != A320_LAT_LOC; t += kDt) {
    a320_set_controls(sim, &c);
    a320_update(sim, kDt);
    a320_get_state(sim, &s);
  }
  std::printf("  NAV to %s, then %s after %.0f s at %.1f NM, %.2f dots\n", s.toWaypoint, a320_lat_mode_name(s.latMode), t,
              s.dmeNm, s.locDots);
  CHECK(s.latMode == A320_LAT_LOC && s.dmeNm > 6.0);
  // HDG pull leaves NAV.
  a320_fcu_command(sim, A320_FCU_HDG_PULL);
  CHECK(state(sim).latMode == A320_LAT_HDG);
  a320_destroy(sim);
}

// Tallinn 08 to Tartu managed: CLB from 1500 ft to FL090, T/D ahead, DES on the path from it down
// to the approach's constraint (ALT CST), then the ILS.
TEST(managed_climb_and_descent) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_start_flight(sim, A320_SCENARIO_RUNWAY, runway(sim, "EETN", "08"), runway(sim, "EETU", "26"), 0.0, A320_PLAN_FULL);
  A320State s = state(sim);
  CHECK(s.armed & A320_ARMED_CLB);
  a320_fcu_set_targets(sim, 250.0, s.fcuHdgMagDeg, 9000.0, 0.0);
  A320Controls c;
  a320_get_controls(sim, &c);
  c.parkBrake = 0;
  c.thrustLever = 1.0;
  int stage = 0;
  bool clbAt1500 = false, todHint = false, desSeen = false, cstSeen = false, apprArmed = false;
  double todAtCruiseNm = 0.0, maxDevFt = 0.0, desS = 0.0, altAtCfFt = 0.0, t = 0.0;
  for (; t < 3000.0 && stage < 5; t += kDt) {
    a320_set_controls(sim, &c);
    a320_update(sim, kDt);
    a320_get_state(sim, &s);
    if (stage == 0) {
      c.stickPitch = s.iasKt > s.vrKt && s.radioAltFt < 400.0 ? std::fmax(-0.3, std::fmin(0.7, 0.12 * (12.0 - s.pitchDeg))) : 0.0;
      if (!s.onGround && s.radioAltFt > 400.0) {
        c.stickPitch = 0.0;
        c.gearDown = 0;
        a320_fcu_command(sim, A320_FCU_AP1);
        stage = 1;
      }
    } else if (stage == 1 && s.vertMode == A320_VERT_CLB) {
      clbAt1500 = s.radioAltFt > 1400.0 && s.radioAltFt < 1800.0;
      c.thrustLever = 0.75;
      c.flapsLever = 0;
      a320_fcu_command(sim, A320_FCU_ATHR);
      stage = 2;
    } else if (stage == 2 && s.vertMode == A320_VERT_ALT && s.altitudeFt > 8800.0) {
      todAtCruiseNm = s.todDistanceNm;
      stage = 3;
    } else if (stage == 3) {
      // The FCU altitude below the constraint (2300 ft): DES stops at the constraint, ALT CST.
      if (!todHint && std::strstr(s.hint, "T/D REACHED")) {
        todHint = true;
        a320_fcu_set_targets(sim, 220.0, s.fcuHdgMagDeg, 2000.0, 0.0);
        a320_fcu_command(sim, A320_FCU_ALT_PUSH);
      }
      if (s.vertMode == A320_VERT_DES) {
        desSeen = true;
        desS += kDt;
        if (desS > 60.0) maxDevFt = std::fmax(maxDevFt, std::fabs(s.altitudeFt - s.descentPathAltFt));
      }
      if (s.vertMode == A320_VERT_ALT_CST || s.vertMode == A320_VERT_ALT_CST_STAR) cstSeen = true;
      if (std::strncmp(s.toWaypoint, "FF", 2) == 0 && altAtCfFt == 0.0) altAtCfFt = s.altitudeFt;
      if (s.routeActive >= a320_route_count(sim) - 4 && !apprArmed) {
        a320_fcu_command(sim, A320_FCU_APPR);
        apprArmed = true;
      }
      if (s.vertMode == A320_VERT_GS) stage = 5;
    }
  }
  std::printf("  CLB at 1500 ft %d; at FL090 T/D %.1f NM ahead; T/D REACHED %d; DES %d, %.0f ft off the path at most;"
              " ALT CST %d, %.0f ft at CF; then %s/%s at %.1f NM after %.0f s\n",
              clbAt1500, todAtCruiseNm, todHint, desSeen, maxDevFt, cstSeen, altAtCfFt, a320_lat_mode_name(s.latMode),
              a320_vert_mode_name(s.vertMode), s.dmeNm, t);
  CHECK(clbAt1500);
  CHECK(todAtCruiseNm > 20.0);  // 6700 ft at 318 ft/NM before the constraint
  CHECK(todHint && desSeen);
  CHECK(maxDevFt < 250.0);
  CHECK(cstSeen && std::fabs(altAtCfFt - 2300.0) < 100.0);
  CHECK(s.vertMode == A320_VERT_GS);
  CHECK(std::strcmp(a320_vert_mode_name(A320_VERT_ALT_CST), "ALT CST") == 0);
  a320_destroy(sim);
}
