// The MCDU through the C API, as the front end drives it: pages, entries and what they change.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "Check.h"
#include "a320/a320_api.h"

namespace {

constexpr double kDt = 1.0 / 120.0;

int runwayIndex(A320Sim* sim, const char* ident) {
  for (int i = 0; i < a320_runway_count(sim); ++i) {
    A320RunwayInfo info;
    a320_get_runway(sim, i, &info);
    if (std::strcmp(info.ident, ident) == 0) return i;
  }
  return -1;
}

std::string row(A320Sim* sim, int r) {
  A320McduDisplay d;
  a320_mcdu_get_display(sim, &d);
  return std::string(d.text[r], A320_MCDU_COLS);
}

int colorAt(A320Sim* sim, int r, int c) {
  A320McduDisplay d;
  a320_mcdu_get_display(sim, &d);
  return d.color[r][c];
}

void type(A320Sim* sim, const char* text) {
  for (const char* p = text; *p; ++p) a320_mcdu_key(sim, *p);
}

A320State state(A320Sim* sim) {
  A320State s;
  a320_get_state(sim, &s);
  return s;
}

void run(A320Sim* sim, double seconds) {
  for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) a320_update(sim, kDt);
}

void dump(A320Sim* sim) {
  for (int r = 0; r < A320_MCDU_ROWS; ++r) std::printf("    |%s|\n", row(sim, r).c_str());
}

}  // namespace

TEST(mcdu_departure_runway_ils_is_tuned_on_the_ground) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_reset(sim, A320_SCENARIO_RUNWAY, runwayIndex(sim, "26"));
  run(sim, 0.1);
  A320State s = state(sim);
  CHECK(std::strcmp(s.ilsIdent, "ILK") == 0);
  CHECK(std::fabs(s.ilsFreqMHz - 109.30) < 1e-6);
  CHECK(std::fabs(s.ilsCourseMagDeg - 260.0) < 1e-6);
  CHECK(s.depRunwayIndex == runwayIndex(sim, "26") && s.arrRunwayIndex == -1);
  a320_mcdu_key(sim, A320_MCDU_FPLN);
  CHECK(row(sim, 2).rfind("EETN26", 0) == 0);
  a320_mcdu_key(sim, A320_MCDU_RADNAV);
  CHECK(row(sim, 6).rfind("ILK/109.30", 0) == 0);
  dump(sim);
  a320_destroy(sim);
}

TEST(mcdu_arrival_switches_the_ils_in_flight) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_reset(sim, A320_SCENARIO_APPROACH, runwayIndex(sim, "26"));
  run(sim, 0.1);
  CHECK(std::strcmp(state(sim).ilsIdent, "ILK") == 0);

  // F-PLN -> destination -> LAT REV -> ARRIVAL -> ILS08 (temporary, yellow) -> INSERT.
  a320_mcdu_key(sim, A320_MCDU_FPLN);
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 2);
  CHECK(row(sim, 2).find("ARRIVAL>") != std::string::npos);
  a320_mcdu_key(sim, A320_MCDU_LSK1R);
  CHECK(row(sim, 0).find("ARRIVAL TO EETN") != std::string::npos);
  dump(sim);
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 2);  // first approach in the list: ILS08
  CHECK(row(sim, 2).find("ILS08") != std::string::npos);
  CHECK(colorAt(sim, 2, 1) == A320_MCDU_YELLOW);
  CHECK(row(sim, 12).find("INSERT*") != std::string::npos);
  CHECK(std::strcmp(state(sim).ilsIdent, "ILK") == 0);  // nothing changes until INSERT
  a320_mcdu_key(sim, A320_MCDU_LSK1R + 5);
  run(sim, 0.1);
  A320State s = state(sim);
  CHECK(std::strcmp(s.ilsIdent, "IIB") == 0);
  CHECK(std::fabs(s.ilsFreqMHz - 108.30) < 1e-6);
  CHECK(s.arrRunwayIndex == runwayIndex(sim, "08"));
  CHECK(!s.locValid);  // east of the field: behind the 08 localizer
  CHECK(row(sim, 6).rfind("EETN08", 0) == 0);

  // RAD NAV: a manual entry wins over the auto-tuning; CLR gives it back.
  a320_mcdu_key(sim, A320_MCDU_RADNAV);
  type(sim, "109.3");
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 2);
  run(sim, 0.1);
  s = state(sim);
  CHECK(std::strcmp(s.ilsIdent, "ILK") == 0 && s.ilsManual);
  A320McduDisplay d;
  a320_mcdu_get_display(sim, &d);
  CHECK(d.small[6][0] == 0);  // manual entries in the large font
  a320_mcdu_key(sim, A320_MCDU_CLR);
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 2);
  run(sim, 0.1);
  s = state(sim);
  CHECK(std::strcmp(s.ilsIdent, "IIB") == 0 && !s.ilsManual);
  type(sim, "ITL");
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 2);
  CHECK(row(sim, 13).rfind("NOT IN DATA BASE", 0) == 0);
  a320_mcdu_key(sim, A320_MCDU_CLR);  // message first
  CHECK(row(sim, 13).rfind("ITL", 0) == 0);
  a320_mcdu_key(sim, A320_MCDU_CLR);
  a320_mcdu_key(sim, A320_MCDU_CLR);
  a320_mcdu_key(sim, A320_MCDU_CLR);
  CHECK(row(sim, 13).rfind("    ", 0) == 0);
  // Course entry.
  type(sim, "081");
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 3);
  run(sim, 0.1);
  CHECK(std::fabs(state(sim).ilsCourseMagDeg - 81.0) < 1e-6);
  a320_destroy(sim);
}

TEST(mcdu_takeoff_speeds_drive_the_callouts) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_reset(sim, A320_SCENARIO_RUNWAY, runwayIndex(sim, "26"));
  run(sim, 0.1);
  a320_mcdu_key(sim, A320_MCDU_PERF);
  CHECK(row(sim, 0).find("TAKE OFF RWY 26") != std::string::npos);
  CHECK(row(sim, 2).rfind("###", 0) == 0);
  dump(sim);
  type(sim, "140");
  a320_mcdu_key(sim, A320_MCDU_LSK1L);  // V1 140
  type(sim, "135");
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 1);  // VR below V1: refused
  CHECK(row(sim, 13).rfind("V1/VR/V2 DISAGREE", 0) == 0);
  a320_mcdu_key(sim, A320_MCDU_CLR);
  a320_mcdu_key(sim, A320_MCDU_CLR);
  a320_mcdu_key(sim, A320_MCDU_CLR);
  a320_mcdu_key(sim, A320_MCDU_CLR);
  type(sim, "142");
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 1);
  type(sim, "146");
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 2);
  run(sim, 0.1);
  A320State s = state(sim);
  CHECK(s.vSpeedsEntered && s.v1Kt == 140.0 && s.vrKt == 142.0 && s.v2Kt == 146.0);

  // The arrival is ILS 08; the departure ILS stays tuned until after takeoff.
  a320_mcdu_key(sim, A320_MCDU_FPLN);
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 2);
  a320_mcdu_key(sim, A320_MCDU_LSK1R);
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 2);
  a320_mcdu_key(sim, A320_MCDU_LSK1R + 5);
  run(sim, 0.1);
  CHECK(std::strcmp(state(sim).ilsIdent, "ILK") == 0);

  A320Controls c;
  a320_get_controls(sim, &c);
  c.parkBrake = 0;
  c.thrustLever = 1.0;
  bool v1 = false, airborneIb = false;
  uint32_t seq = s.calloutSeq;
  for (int i = 0; i < 120 * 70; ++i) {
    s = state(sim);
    c.stickPitch = s.iasKt > s.vrKt ? std::fmax(-0.3, std::fmin(0.7, 0.12 * (10.0 - s.pitchDeg))) : 0.0;
    a320_set_controls(sim, &c);
    a320_update(sim, kDt);
    s = state(sim);
    if (s.calloutSeq != seq) {
      seq = s.calloutSeq;
      if (std::strcmp(s.callout, "V ONE") == 0) {
        v1 = true;
        CHECK(std::fabs(s.iasKt - 140.0) < 2.0);
      }
    }
    if (!s.onGround && std::strcmp(s.ilsIdent, "IIB") == 0) airborneIb = true;
  }
  std::printf("  V1 call %d, arrival ILS after takeoff %d (%s), %.0f ft\n", v1, airborneIb, s.ilsIdent, s.radioAltFt);
  CHECK(v1 && airborneIb);
  a320_destroy(sim);
}

TEST(mcdu_minimums_and_vapp) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_reset(sim, A320_SCENARIO_FINAL_4NM, runwayIndex(sim, "26"));
  run(sim, 0.1);
  a320_mcdu_key(sim, A320_MCDU_PERF);
  CHECK(row(sim, 0).find("APPR") != std::string::npos);
  type(sim, "200");
  a320_mcdu_key(sim, A320_MCDU_LSK1R + 1);  // RADIO (DH)
  type(sim, "270/15");
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 2);  // MAG WIND: 10 kt headwind component on 26
  dump(sim);
  run(sim, 0.1);
  A320State s = state(sim);
  CHECK(s.dhFt == 200);
  CHECK(s.vappKt >= s.vlsKt - 3.0 && s.vappKt <= s.vlsKt + 15.0);
  std::printf("  VAPP %.0f, VLS %.0f\n", s.vappKt, s.vlsKt);

  // Down the ILS on the autopilot: HUNDRED ABOVE then MINIMUM.
  a320_fcu_command(sim, A320_FCU_APPR);
  if (!state(sim).apEngaged) a320_fcu_command(sim, A320_FCU_AP1);
  A320Controls c;
  a320_get_controls(sim, &c);
  bool above = false, minimum = false;
  uint32_t seq = s.calloutSeq;
  for (int i = 0; i < 120 * 150 && !minimum; ++i) {
    a320_set_controls(sim, &c);
    a320_update(sim, kDt);
    s = state(sim);
    if (s.calloutSeq != seq) {
      seq = s.calloutSeq;
      if (std::strcmp(s.callout, "HUNDRED ABOVE") == 0) above = true;
      if (std::strcmp(s.callout, "MINIMUM") == 0) {
        minimum = true;
        CHECK(std::fabs(s.radioAltFt - 200.0) < 10.0);
      }
    }
  }
  CHECK(above && minimum);
  a320_destroy(sim);
}

// INIT FROM/TO on the ground picks the airports; DEPARTURE and ARRIVAL then list their runways,
// and the ILS follows the inserted arrival once airborne.
TEST(mcdu_from_to_another_airport) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_reset(sim, A320_SCENARIO_RUNWAY, runwayIndex(sim, "26"));
  run(sim, 0.1);
  a320_mcdu_key(sim, A320_MCDU_INIT);
  type(sim, "EETN/EEKE");
  a320_mcdu_key(sim, A320_MCDU_LSK1R);
  CHECK(row(sim, 2).find("EETN/EEKE") != std::string::npos);
  a320_mcdu_key(sim, A320_MCDU_FPLN);
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 2);
  a320_mcdu_key(sim, A320_MCDU_LSK1R);
  CHECK(row(sim, 0).find("ARRIVAL TO EEKE") != std::string::npos);
  dump(sim);
  bool listed17 = false, noIls35 = false;
  for (int r = 0; r < A320_MCDU_ROWS; ++r) {
    listed17 = listed17 || row(sim, r).find("ILS17") != std::string::npos;
    noIls35 = noIls35 || row(sim, r).find("NO ILS") != std::string::npos;
  }
  CHECK(listed17 && noIls35);
  a320_destroy(sim);
}

// The FLIGHT menu's choice: the whole flight plan entered, or none of it.
TEST(mcdu_flight_plan_full_or_empty) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  const int r26 = runwayIndex(sim, "26"), r17 = runwayIndex(sim, "17");

  a320_start_flight(sim, A320_SCENARIO_COLD_DARK, r26, r17, 0.0, A320_PLAN_FULL);
  run(sim, 0.1);
  A320State s = state(sim);
  CHECK(s.fmsFlightNumberSet && s.vSpeedsEntered && s.fmsFlapsThsSet && s.fmsFlexTempC == 50);
  CHECK(s.depRunwayIndex == r26 && s.arrRunwayIndex == r17);
  a320_mcdu_key(sim, A320_MCDU_PERF);
  a320_mcdu_key(sim, A320_MCDU_NEXTPAGE);
  dump(sim);
  CHECK(row(sim, 2).rfind("1013", 0) == 0);
  CHECK(row(sim, 2).find("ILS17") != std::string::npos);

  a320_start_flight(sim, A320_SCENARIO_RUNWAY, r26, r17, 0.0, A320_PLAN_EMPTY);
  run(sim, 0.1);
  s = state(sim);
  CHECK(!s.fmsFlightNumberSet && !s.vSpeedsEntered && s.depRunwayIndex == -1 && s.arrRunwayIndex == -1);
  CHECK(s.ilsFreqMHz <= 0.0 || s.ilsIdent[0] == '\0');
  a320_mcdu_key(sim, A320_MCDU_INIT);
  CHECK(row(sim, 2).find("####/####") != std::string::npos);
  a320_mcdu_key(sim, A320_MCDU_FPLN);
  a320_mcdu_key(sim, A320_MCDU_LSK1L);
  CHECK(row(sim, 13).rfind("ENTER FROM/TO ON INIT", 0) == 0);
  a320_mcdu_key(sim, A320_MCDU_CLR);
  // The crew's way: INIT FROM/TO, then F-PLN > departure > DEPARTURE > 26 > INSERT.
  a320_mcdu_key(sim, A320_MCDU_INIT);
  type(sim, "EETN/EEKE");
  a320_mcdu_key(sim, A320_MCDU_LSK1R);
  a320_mcdu_key(sim, A320_MCDU_FPLN);
  a320_mcdu_key(sim, A320_MCDU_LSK1L);
  a320_mcdu_key(sim, A320_MCDU_LSK1L);
  dump(sim);
  int line = -1;
  for (int r = 4; r <= 8; r += 2)
    if (row(sim, r).find("<26") == 0) line = r / 2 - 1;
  CHECK(line >= 0);
  a320_mcdu_key(sim, A320_MCDU_LSK1L + line);
  a320_mcdu_key(sim, A320_MCDU_LSK1R + 5);
  run(sim, 0.1);
  s = state(sim);
  CHECK(s.depRunwayIndex == r26 && std::strcmp(s.ilsIdent, "ILK") == 0);
  a320_destroy(sim);
}
