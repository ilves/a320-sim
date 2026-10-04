// The ILS autoland guide flown by a pilot who does exactly what each step says.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "Check.h"
#include "a320/a320_api.h"

namespace {

constexpr double kDt = 1.0 / 120.0;

int runway26(A320Sim* sim) {
  for (int i = 0; i < a320_runway_count(sim); ++i) {
    A320RunwayInfo info;
    a320_get_runway(sim, i, &info);
    if (std::strcmp(info.ident, "26") == 0) return i;
  }
  return 0;
}

}  // namespace

TEST(guide_texts_complete) {
  CHECK(a320_guide_count() >= 1);
  for (int g = 0; g < a320_guide_count(); ++g) {
    CHECK(std::strlen(a320_guide_name(g)) > 0);
    CHECK(a320_guide_step_count(g) > 0 && a320_guide_step_count(g) <= 32);
    for (int st = 0; st < a320_guide_step_count(g); ++st)
      for (int f = A320_GUIDE_PHASE; f <= A320_GUIDE_MSFS; ++f)
        CHECK(std::strlen(a320_guide_step_text(g, st, static_cast<A320GuideText>(f))) > 0);
  }
  CHECK(std::strlen(a320_guide_step_text(0, 99, A320_GUIDE_TITLE)) == 0);
}

TEST(approach_scenario_holds_and_hints) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_reset(sim, A320_SCENARIO_APPROACH, runway26(sim));
  A320Controls c;
  a320_get_controls(sim, &c);
  A320State s;
  for (int i = 0; i < 120 * 30; ++i) {
    a320_set_controls(sim, &c);
    a320_update(sim, kDt);
  }
  a320_get_state(sim, &s);
  std::printf("  after 30 s: alt %.0f ft, IAS %.0f kt, %s/%s, AP %d A/THR %d active %d, gear %.2f\n", s.altitudeFt, s.iasKt,
              a320_vert_mode_name(s.vertMode), a320_lat_mode_name(s.latMode), s.apEngaged, s.athrEngaged, s.athrActive, s.gearPos);
  CHECK(std::fabs(s.altitudeFt - 3000.0) < 60.0);
  CHECK(std::fabs(s.iasKt - 220.0) < 5.0);
  CHECK(s.ap1Engaged && !s.ap2Engaged && s.athrActive);
  CHECK(s.gearPos < 0.01);

  // AP2 outside the approach replaces AP1 and explains why.
  const uint32_t seq = s.hintSeq;
  a320_fcu_command(sim, A320_FCU_AP2);
  a320_get_state(sim, &s);
  CHECK(s.ap2Engaged && !s.ap1Engaged && s.hintSeq == seq + 1);
  std::printf("  hint: %s\n", s.hint);
  // Gear can't go up on the ground... but here it is just a reminder-free move.
  c.flapsLever = 2;  // at 220 kt, above VFE 2 (200 kt)
  a320_set_controls(sim, &c);
  a320_get_state(sim, &s);
  CHECK(s.hintSeq == seq + 2 && std::strstr(s.hint, "FLAPS") != nullptr);
  a320_destroy(sim);
}

TEST(guide_ils_autoland_followed_step_by_step) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  const int guide = 0;
  a320_reset(sim, a320_guide_scenario(guide), runway26(sim));
  a320_guide_start(sim, guide);
  A320Controls c;
  a320_get_controls(sim, &c);
  A320State s;
  a320_get_state(sim, &s);
  A320GuideStatus g;
  int lastStep = -1, alerts = 0;
  double touchdownKt = 0.0;
  bool sawLocStar = false, sawGsStar = false, sawGs = false;
  bool pressedAppr = false, pressedAp2 = false, pressedApOff = false;
  for (double t = 0.0; t < 900.0; t += kDt) {
    a320_guide_get_status(sim, &g);
    if (g.complete) break;
    if (g.step != lastStep) {
      std::printf("  %5.0f s  step %2d %-26s  %s/%s  %.0f ft  %.0f kt (target %.0f, VLS %.0f)  dme %.1f\n", t, g.step,
                  a320_guide_step_text(guide, g.step, A320_GUIDE_TITLE), a320_vert_mode_name(s.vertMode),
                  a320_lat_mode_name(s.latMode), s.altitudeFt, s.iasKt, s.fcuSpdKt, s.vlsKt, s.dmeNm);
      lastStep = g.step;
    }
    if (std::strlen(a320_guide_alert(sim)) > 0 && ++alerts < 3) std::printf("  alert: %s\n", a320_guide_alert(sim));
    auto spd = [&](double kt) { a320_fcu_set_targets(sim, kt, s.fcuHdgMagDeg, s.fcuAltFt, s.fcuVsFpm); };
    switch (g.step) {
      case 1: c.autobrake = A320_AUTOBRAKE_LO; break;
      case 2: c.efisLs = 1; c.ndMode = A320_ND_ROSE_LS; break;
      case 4: spd(180.0); break;
      case 5: if (s.iasKt < 228.0) c.flapsLever = 1; break;
      case 6: if (!pressedAppr) { a320_fcu_command(sim, A320_FCU_APPR); pressedAppr = true; } break;
      case 7: if (!pressedAp2) { a320_fcu_command(sim, A320_FCU_AP2); pressedAp2 = true; } break;
      case 9:
        if (s.iasKt < 198.0) c.flapsLever = 2;
        if (c.flapsLever == 2) spd(160.0);
        break;
      case 11: c.gearDown = 1; break;
      case 12:
        if (c.flapsLever == 3 && s.flapDeg > 19.0 && s.iasKt < 175.0) c.flapsLever = 4;
        if (s.iasKt < 183.0 && c.flapsLever < 3) c.flapsLever = 3;
        break;
      case 13: if (s.flapDeg > 34.0) spd(std::ceil(s.vlsKt) + 5.0); break;
      case 14: c.spoilersArmed = 1; break;
      case 17: if (s.radioAltFt < 25.0 || s.onGround) c.thrustLever = 0.0; break;
      case 19: if (s.onGround) { c.reverse = 1; c.thrustLever = 1.0; } break;
      case 20: if (s.groundSpeedKt < 70.0) { c.reverse = 0; c.thrustLever = 0.0; } break;
      case 21:
        if (s.groundSpeedKt < 50.0) c.brakeLeft = c.brakeRight = 0.6;
        if (s.groundSpeedKt < 1.0) c.parkBrake = 1;
        if (s.apEngaged && !pressedApOff) { a320_fcu_command(sim, A320_FCU_AP1); a320_fcu_command(sim, A320_FCU_AP2); pressedApOff = true; }
        break;
      default:
        if (g.manual) a320_guide_next(sim);
        break;
    }
    a320_set_controls(sim, &c);
    a320_update(sim, kDt);
    const bool wasFlying = !s.onGround;
    a320_get_state(sim, &s);
    if (wasFlying && s.onGround) touchdownKt = s.iasKt;
    sawLocStar = sawLocStar || s.latMode == A320_LAT_LOC_STAR;
    sawGsStar = sawGsStar || s.vertMode == A320_VERT_GS_STAR;
    sawGs = sawGs || s.vertMode == A320_VERT_GS;
  }
  a320_guide_get_status(sim, &g);
  std::printf("  guide %s after %.0f s sim time; touchdown at %.0f kt (VLS %.0f), %.0f m past threshold, %.1f m off "
              "centreline, %.0f fpm\n", g.complete ? "complete" : "NOT complete", s.simTimeS, touchdownKt, s.vlsKt,
              s.touchdownDistanceM, s.touchdownCenterlineM, s.touchdownFpm);
  CHECK(g.complete);
  CHECK(g.doneMask == (1u << g.stepCount) - 1u);
  CHECK(s.touchdownSeq > 0 && s.touchdownDistanceM > 100.0 && s.touchdownDistanceM < 900.0);
  CHECK(std::fabs(s.touchdownCenterlineM) < 8.0 && s.touchdownFpm > -600.0);
  CHECK(touchdownKt < s.vlsKt + 12.0);
  CHECK(sawLocStar && sawGsStar && sawGs);  // the FMA shows capture, then tracking
  a320_destroy(sim);
}

TEST(tutor_hints_on_the_ground) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_reset(sim, A320_SCENARIO_RUNWAY, runway26(sim));
  A320State s;
  a320_get_state(sim, &s);
  const uint32_t seq = s.hintSeq;
  a320_fcu_command(sim, A320_FCU_AP1);
  a320_get_state(sim, &s);
  CHECK(!s.apEngaged && s.hintSeq == seq + 1 && std::strstr(s.hint, "AP:") != nullptr);
  a320_fcu_command(sim, A320_FCU_APPR);
  a320_get_state(sim, &s);
  CHECK(s.armed == 0 && std::strstr(s.hint, "400 ft") != nullptr);
  A320Controls c;
  a320_get_controls(sim, &c);
  c.gearDown = 0;
  a320_set_controls(sim, &c);
  a320_get_state(sim, &s);
  CHECK(std::strstr(s.hint, "GEAR") != nullptr);
  a320_destroy(sim);
}

// Too high on the localizer: ALT never meets the beam, the tutor says why, and a V/S descent
// captures G/S from above (FCOM technique) while G/S stays armed.
TEST(glideslope_from_above) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_reset(sim, A320_SCENARIO_APPROACH, runway26(sim));
  A320Controls c;
  a320_get_controls(sim, &c);
  A320State s;
  a320_get_state(sim, &s);
  a320_fcu_set_targets(sim, s.fcuSpdKt, s.fcuHdgMagDeg, 4500.0, s.fcuVsFpm);
  a320_fcu_command(sim, A320_FCU_ALT_PULL);
  a320_fcu_command(sim, A320_FCU_APPR);
  bool hinted = false, captured = false, descending = false;
  for (int i = 0; i < 120 * 600 && !captured; ++i) {
    a320_set_controls(sim, &c);
    a320_update(sim, kDt);
    a320_get_state(sim, &s);
    if (!hinted && std::strstr(s.hint, "above the glideslope")) {
      hinted = true;
      std::printf("  at %.1f NM, %.0f ft, VS %.0f, %s: %s\n", s.dmeNm, s.altitudeFt, s.verticalSpeedFpm,
                  a320_lat_mode_name(s.latMode), s.hint);
      CHECK(s.latMode == A320_LAT_LOC || s.latMode == A320_LAT_LOC_STAR);
      CHECK(s.vertMode != A320_VERT_GS_STAR && s.vertMode != A320_VERT_GS);
    }
    if (hinted && !descending && s.latMode == A320_LAT_LOC) {
      descending = true;
      a320_fcu_command(sim, A320_FCU_VS_PULL);
      a320_fcu_set_targets(sim, 180.0, s.fcuHdgMagDeg, 2000.0, -1500.0);
    }
    captured = s.vertMode == A320_VERT_GS_STAR || s.vertMode == A320_VERT_GS;
  }
  std::printf("  G/S captured at %.1f NM, %.0f ft (%s/%s, loc %.2f gs %.2f valid %d/%d, hint: %s)\n", s.dmeNm, s.altitudeFt,
              a320_vert_mode_name(s.vertMode), a320_lat_mode_name(s.latMode), s.locDots, s.gsDots, s.locValid, s.gsValid, s.hint);
  CHECK(hinted && descending && captured);
  // Then it settles on the beam like a capture from below, configured as a pilot would.
  c.flapsLever = 2;
  double worst = 0.0;
  for (int i = 0; i < 120 * 90; ++i) {
    a320_set_controls(sim, &c);
    a320_update(sim, kDt);
    a320_get_state(sim, &s);
    if (i > 120 * 30) worst = std::fmax(worst, std::fabs(s.gsDots));
    if (i % (120 * 5) == 0) std::printf("    %3d s %5.0f ft %+.2f dots VS %.0f IAS %.0f\n", i / 120, s.altitudeFt, s.gsDots, s.verticalSpeedFpm, s.iasKt);
  }
  std::printf("  then %s, worst %.2f dots, %.0f ft\n", a320_vert_mode_name(s.vertMode), worst, s.altitudeFt);
  CHECK(s.vertMode == A320_VERT_GS && worst < 0.3);
  a320_destroy(sim);
}

// The takeoff lesson from cold and dark, done the way its steps say, to radar contact.
TEST(guide_takeoff_from_cold_and_dark) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  int guide = -1;
  for (int g = 0; g < a320_guide_count(); ++g)
    if (std::strstr(a320_guide_name(g), "Takeoff")) guide = g;
  CHECK(guide >= 0);
  if (guide < 0) { a320_destroy(sim); return; }
  a320_reset(sim, a320_guide_scenario(guide), runway26(sim));
  a320_guide_start(sim, guide);
  A320Controls c;
  a320_get_controls(sim, &c);
  A320State s;
  auto run = [&](double seconds) {
    for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) {
      a320_set_controls(sim, &c);
      a320_update(sim, kDt);
    }
    a320_get_state(sim, &s);
  };
  auto type = [&](const char* text) {
    for (const char* p = text; *p; ++p) a320_mcdu_key(sim, *p);
  };
  auto reply = [&](const char* text) {
    A320AtcStatus st;
    a320_atc_get_status(sim, &st);
    for (int i = 0; i < st.optionCount; ++i)
      if (std::strstr(st.options[i], text)) {
        a320_atc_choose(sim, i);
        return true;
      }
    return false;
  };
  auto waitFor = [&](auto cond, double maxS) {
    for (double t = 0.0; t < maxS; t += 0.5) {
      if (cond()) return true;  // once: conditions may act (pick a reply)
      run(0.5);
    }
    return false;
  };
  run(0.5);
  CHECK(c.flapsLever == 0 && !c.spoilersArmed);  // cold and dark is clean

  c.apuMaster = 1;
  c.apuStart = 1;
  run(1.0);
  c.apuStart = 0;
  CHECK(waitFor([&] { return s.apuAvail != 0; }, 120.0));
  c.apuBleed = 1;
  c.lights |= A320_LT_BEACON;
  c.signs |= A320_SIGN_SEATBELTS;
  a320_mcdu_key(sim, A320_MCDU_INIT);
  type("EST123");
  a320_mcdu_key(sim, A320_MCDU_LSK1L + 2);
  run(1.0);
  a320_guide_next(sim);  // F-PLN check
  a320_mcdu_key(sim, A320_MCDU_PERF);
  for (int k = 0; k < 3; ++k) a320_mcdu_key(sim, A320_MCDU_LSK1L + k);  // computed V-speeds
  type("1/UP0.0");
  a320_mcdu_key(sim, A320_MCDU_LSK1R + 2);
  type("50");
  a320_mcdu_key(sim, A320_MCDU_LSK1R + 3);
  std::swap(c.com1ActiveKhz, c.com1StandbyKhz);  // ATIS
  run(20.0);
  std::swap(c.com1ActiveKhz, c.com1StandbyKhz);
  run(1.0);
  CHECK(reply("request IFR clearance"));
  CHECK(waitFor([&] { return s.atcAwaitingReadback != 0 && reply("climb altitude 4000 feet, squawk"); }, 30.0));
  // The readback offered first may be a wrong one: answer until accepted.
  for (int i = 0; i < 6 && !s.atcIfrCleared; ++i) {
    run(12.0);
    if (s.atcAwaitingReadback) {
      A320AtcStatus st;
      a320_atc_get_status(sim, &st);
      a320_atc_choose(sim, i % 3);
    }
  }
  run(8.0);
  CHECK(s.atcIfrCleared);
  c.xpdrCode = s.atcSquawk;
  c.xpdrMode = A320_XPDR_AUTO;
  a320_fcu_set_targets(sim, s.fcuSpdKt, s.fcuHdgMagDeg, 4000.0, s.fcuVsFpm);

  c.engMode = A320_ENG_MODE_IGN_START;
  c.engMaster[1] = 1;
  CHECK(waitFor([&] { return s.engRunning[1] != 0; }, 120.0));
  c.engMaster[0] = 1;
  CHECK(waitFor([&] { return s.engRunning[0] != 0; }, 120.0));
  c.engMode = A320_ENG_MODE_NORM;
  c.apuBleed = 0;
  c.flapsLever = 1;
  c.spoilersArmed = 1;
  c.autobrake = A320_AUTOBRAKE_MAX;
  c.lights |= A320_LT_STROBE | A320_LT_LANDING | A320_LT_TAKEOFF;
  run(1.0);
  CHECK(waitFor([&] { return reply("ready for departure"); }, 20.0));
  for (int i = 0; i < 6 && !s.atcTakeoffCleared; ++i) {
    run(8.0);
    if (s.atcAwaitingReadback) a320_atc_choose(sim, i % 3);
  }
  run(4.0);
  CHECK(s.atcTakeoffCleared);

  c.parkBrake = 0;
  c.thrustLever = 0.88;  // FLX/MCT
  bool gearUp = false, ap = false, cl = false, flaps0 = false, checkedIn = false;
  int tries = 0;
  A320GuideStatus g;
  for (double t = 0.0; t < 600.0; t += 0.25) {
    a320_guide_get_status(sim, &g);
    if (g.complete) break;
    run(0.25);
    if (!ap) c.stickPitch = s.iasKt > s.vrKt ? std::fmax(-0.3, std::fmin(0.7, 0.12 * (12.0 - s.pitchDeg))) : 0.0;
    if (!s.onGround && s.verticalSpeedFpm > 300.0 && !gearUp) {
      c.gearDown = 0;
      gearUp = true;
    }
    if (!s.onGround && s.radioAltFt > 150.0 && !ap) {
      c.stickPitch = 0.0;
      a320_fcu_command(sim, A320_FCU_AP1);
      a320_fcu_command(sim, A320_FCU_ALT_PULL);
      ap = true;
    }
    if (ap && s.radioAltFt > 1500.0 && !cl) {
      c.thrustLever = 0.75;
      if (!s.athrEngaged) a320_fcu_command(sim, A320_FCU_ATHR);
      cl = true;
    }
    if (cl && !flaps0 && s.iasKt > 190.0) {
      c.flapsLever = 0;
      flaps0 = true;
    }
    if (s.atcAwaitingReadback) {
      // Try each readback in turn ("say again" is last) until ATC accepts one.
      A320AtcStatus st;
      a320_atc_get_status(sim, &st);
      if (st.optionCount > 1) a320_atc_choose(sim, tries++ % (st.optionCount - 1));
    } else if (s.atcPhase == A320_ATC_PHASE_RADAR && !checkedIn) {
      c.com1ActiveKhz = 127905;
      if (reply("Tallinn Radar, ")) checkedIn = true;
    }
    if (flaps0 && s.atcRadarContact) {
      c.spoilersArmed = 0;
      c.apuMaster = 0;
    }
  }
  run(1.0);
  a320_guide_get_status(sim, &g);
  std::printf("  takeoff lesson: step %d of %d, complete %d, done mask %08x, radar contact %d, A/THR %d/%d detent %d\n",
              g.step, g.stepCount, g.complete, g.doneMask, s.atcRadarContact, s.athrEngaged, s.athrActive, s.thrustDetent);
  if (!g.complete) {
    std::printf("  ATC phase %d awaiting %d, COM1 %d, alt %.0f, squawk %d/%d mode %d\n", s.atcPhase, s.atcAwaitingReadback,
                c.com1ActiveKhz, s.altitudeFt, c.xpdrCode, s.atcSquawk, c.xpdrMode);
    for (uint32_t q = 1; q <= s.atcMessageSeq; ++q) {
      A320AtcMessage m;
      if (a320_atc_message(sim, q, &m)) std::printf("    [%.0f %s] %s\n", m.simTimeS, m.station, m.text);
    }
  }
  CHECK(g.complete);
  a320_destroy(sim);
}
