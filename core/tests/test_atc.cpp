// ATC through the C API: a scripted crew that answers the radio, tunes and squawks what it is
// told, and flies the vectors, from the IFR clearance on runway 26 to "vacate the runway".
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

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

A320State state(A320Sim* sim) {
  A320State s;
  a320_get_state(sim, &s);
  return s;
}

A320AtcStatus status(A320Sim* sim) {
  A320AtcStatus st;
  a320_atc_get_status(sim, &st);
  return st;
}

int findOption(const A320AtcStatus& st, const char* text) {
  for (int i = 0; i < st.optionCount; ++i)
    if (std::strstr(st.options[i], text)) return i;
  return -1;
}

// Prints and collects the radio messages since the last call.
struct Radio {
  uint32_t seen = 0;
  std::vector<std::string> heard;
  std::string lastAtc;
  void poll(A320Sim* sim, bool print = true) {
    const A320State s = state(sim);
    for (; seen < s.atcMessageSeq; ++seen) {
      A320AtcMessage m;
      if (!a320_atc_message(sim, seen + 1, &m)) continue;
      if (print && m.speaker != A320_ATC_SPEAKER_ATIS)
        std::printf("    [%6.1f s %s%s] %s\n", m.simTimeS, m.station, m.heard ? "" : " (not heard)", m.text);
      if (m.heard) heard.push_back(m.text);
      if (m.speaker == A320_ATC_SPEAKER_ATC && m.heard) lastAtc = m.text;
    }
  }
  bool said(const char* text) const {
    for (const std::string& h : heard)
      if (h.find(text) != std::string::npos) return true;
    return false;
  }
};

void run(A320Sim* sim, A320Controls& c, double seconds, Radio& radio) {
  for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) {
    a320_set_controls(sim, &c);
    a320_update(sim, kDt);
  }
  radio.poll(sim);
}

}  // namespace

TEST(atc_clearance_readback_and_takeoff_clearance) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_reset(sim, A320_SCENARIO_RUNWAY, runwayIndex(sim, "26"));
  A320Controls c;
  a320_get_controls(sim, &c);
  CHECK(c.com1ActiveKhz == 135905 && c.com1StandbyKhz == 124880 && c.xpdrMode == A320_XPDR_STBY);
  Radio radio;
  run(sim, c, 1.0, radio);
  CHECK(std::strcmp(status(sim).station, "TALLINN TOWER") == 0);

  // ATIS first: swap to 124.880 and listen.
  std::swap(c.com1ActiveKhz, c.com1StandbyKhz);
  run(sim, c, 20.0, radio);
  CHECK(radio.said("This is Tallinn Information, information"));
  std::swap(c.com1ActiveKhz, c.com1StandbyKhz);
  run(sim, c, 1.0, radio);

  A320AtcStatus st = status(sim);
  int i = findOption(st, "request IFR clearance");
  CHECK(i >= 0 && std::strstr(st.options[i], "with information") != nullptr);
  a320_atc_choose(sim, i);
  run(sim, c, 12.0, radio);
  CHECK(radio.said("cleared to Tallinn via radar vectors"));
  st = status(sim);
  CHECK(st.awaitingReadback && st.optionCount == 4);

  // A wrong altitude is corrected, then the right readback is accepted.
  const uint32_t hints = state(sim).hintSeq;
  a320_atc_choose(sim, findOption(st, "5000 feet"));
  run(sim, c, 12.0, radio);
  CHECK(radio.said("negative, I say again"));
  CHECK(state(sim).hintSeq > hints);
  std::printf("  hint: %s\n", state(sim).hint);
  const char* sq = std::strstr(radio.lastAtc.c_str(), "squawk ");
  const std::string code = sq ? std::string(sq + 7, 4) : "";
  st = status(sim);
  for (int k = 0; k < st.optionCount; ++k)
    if (std::strstr(st.options[k], "4000 feet") && std::strstr(st.options[k], ("squawk " + code).c_str())) i = k;
  a320_atc_choose(sim, i);
  run(sim, c, 14.0, radio);
  CHECK(radio.said("readback correct, report ready for departure"));
  A320State s = state(sim);
  CHECK(s.atcIfrCleared && s.atcClearedAltFt == 4000 && s.atcSquawk == std::atoi(code.c_str()));

  a320_atc_choose(sim, findOption(status(sim), "ready for departure"));
  run(sim, c, 10.0, radio);
  CHECK(radio.said("runway 26, cleared for takeoff"));
  a320_atc_choose(sim, findOption(status(sim), "Cleared for takeoff runway 26"));
  run(sim, c, 2.0, radio);
  CHECK(state(sim).atcTakeoffCleared);
  a320_destroy(sim);
}

TEST(atc_full_flight_vectors_ils_landing) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_reset(sim, A320_SCENARIO_RUNWAY, runwayIndex(sim, "26"));
  A320Controls c;
  a320_get_controls(sim, &c);
  Radio radio;
  run(sim, c, 1.0, radio);

  // Clearance and takeoff clearance, answering the first acceptable readback (a crew that
  // learns from "negative").
  int tries = 0;
  auto answer = [&]() {
    const A320AtcStatus st = status(sim);
    if (!st.awaitingReadback || st.optionCount == 0) return;
    a320_atc_choose(sim, tries % (st.optionCount - 1));  // the last option is "say again"
    ++tries;
  };
  a320_atc_choose(sim, findOption(status(sim), "request IFR clearance"));
  bool ready = false;
  double t = 0.0;
  A320State s = state(sim);
  int apStage = 0, flapsStage = 0;
  bool apprPressed = false, landed = false;
  int lastHeading = -1, lastAlt = 0, lastSpeed = 0;
  int phaseSeen = 0;
  for (; t < 2400.0; t += 0.5) {
    run(sim, c, 0.5, radio);
    s = state(sim);
    phaseSeen |= 1 << s.atcPhase;
    if (s.atcAwaitingReadback) {
      answer();
    } else {
      tries = 0;
    }
    const A320AtcStatus st = status(sim);
    if (s.atcIfrCleared && !ready && !s.atcAwaitingReadback) {
      const int i = findOption(st, "ready for departure");
      if (i >= 0) {
        a320_atc_choose(sim, i);
        ready = true;
      }
    }
    // Squawk and frequencies as instructed.
    if (s.atcSquawk > 0) {
      c.xpdrCode = s.atcSquawk;
      c.xpdrMode = A320_XPDR_AUTO;
    }
    if (!s.atcAwaitingReadback) {
      const char* contact = std::strstr(radio.lastAtc.c_str(), "contact Tallinn ");
      if (contact) {
        const char* f = std::strpbrk(contact, "0123456789");
        if (f) {
          const int khz = static_cast<int>(std::lround(std::atof(f) * 1000.0));
          if (khz != c.com1ActiveKhz) {
            c.com1StandbyKhz = khz;
            std::swap(c.com1ActiveKhz, c.com1StandbyKhz);
          }
        }
      }
      for (const char* checkIn : {"Tallinn Radar, ", "Tallinn Tower, "}) {
        const int i = findOption(st, checkIn);
        if (i >= 0 && std::strstr(st.options[i], "request") == nullptr) a320_atc_choose(sim, i);
      }
    }

    // Takeoff: TOGA, rotate at VR, then the autopilot flies what ATC says.
    if (s.atcTakeoffCleared && s.onGround && !landed && s.atcPhase <= A320_ATC_PHASE_DEPARTURE) {
      c.parkBrake = 0;
      c.thrustLever = 1.0;
    }
    if (apStage == 0) {
      c.stickPitch = s.iasKt > s.vrKt && s.radioAltFt < 400.0 ? std::fmax(-0.3, std::fmin(0.7, 0.12 * (12.0 - s.pitchDeg))) : 0.0;
      if (!s.onGround && s.radioAltFt > 400.0) {
        c.stickPitch = 0.0;
        c.gearDown = 0;
        a320_fcu_command(sim, A320_FCU_AP1);
        a320_fcu_set_targets(sim, 210.0, 260.0, s.atcClearedAltFt, 0.0);
        a320_fcu_command(sim, A320_FCU_ALT_PULL);
        apStage = 1;
      }
    }
    if (apStage == 1 && s.radioAltFt > 1500.0) {
      c.thrustLever = 0.75;  // CL
      if (!s.athrEngaged) a320_fcu_command(sim, A320_FCU_ATHR);
      c.flapsLever = 0;
      apStage = 2;
    }
    if (apStage >= 2) {
      const int hdg = s.atcHeadingMag > 0 ? s.atcHeadingMag : -1;
      const int spd = s.atcSpeedKt > 0 ? s.atcSpeedKt : (s.atcApproachCleared ? 180 : 210);
      if (hdg > 0 && hdg != lastHeading && !s.atcApproachCleared) {
        a320_fcu_set_targets(sim, s.fcuSpdKt, hdg, s.fcuAltFt, s.fcuVsFpm);
        a320_fcu_command(sim, A320_FCU_HDG_PULL);
        lastHeading = hdg;
      }
      if (s.atcApproachCleared && hdg > 0 && hdg != lastHeading) {
        a320_fcu_set_targets(sim, s.fcuSpdKt, hdg, s.fcuAltFt, s.fcuVsFpm);
        a320_fcu_command(sim, A320_FCU_HDG_PULL);
        lastHeading = hdg;
      }
      if (s.atcClearedAltFt != lastAlt && s.atcClearedAltFt > 0 && !s.atcApproachCleared) {
        a320_fcu_set_targets(sim, s.fcuSpdKt, s.fcuHdgMagDeg, s.atcClearedAltFt, s.fcuVsFpm);
        if (std::fabs(s.altitudeFt - s.atcClearedAltFt) > 200.0) a320_fcu_command(sim, A320_FCU_ALT_PULL);
        lastAlt = s.atcClearedAltFt;
      }
      if (spd != lastSpeed && flapsStage == 0) {
        a320_fcu_set_targets(sim, spd, s.fcuHdgMagDeg, s.fcuAltFt, s.fcuVsFpm);
        lastSpeed = spd;
      }
      if (s.atcApproachCleared && !apprPressed) {
        a320_fcu_command(sim, A320_FCU_APPR);
        a320_fcu_command(sim, A320_FCU_AP2);
        apprPressed = true;
      }
      // Configuration down the approach, as in the ILS lesson.
      if (flapsStage == 0 && s.latMode == A320_LAT_LOC) {
        c.flapsLever = 1;
        flapsStage = 1;
      }
      if (flapsStage == 1 && s.iasKt < 200.0) {
        c.flapsLever = 2;
        a320_fcu_set_targets(sim, 160.0, s.fcuHdgMagDeg, s.fcuAltFt, s.fcuVsFpm);
        flapsStage = 2;
      }
      if (flapsStage == 2 && (s.vertMode == A320_VERT_GS || s.vertMode == A320_VERT_GS_STAR)) {
        c.gearDown = 1;
        c.flapsLever = 3;
        c.spoilersArmed = 1;
        c.autobrake = A320_AUTOBRAKE_LO;
        flapsStage = 3;
      }
      if (flapsStage == 3 && s.iasKt < 180.0 && s.radioAltFt < 2000.0) {
        c.flapsLever = 4;
        a320_fcu_set_targets(sim, s.vappKt, s.fcuHdgMagDeg, s.fcuAltFt, s.fcuVsFpm);
        flapsStage = 4;
      }
      if (s.vertMode == A320_VERT_FLARE && s.radioAltFt < 25.0) c.thrustLever = 0.0;
      if (s.onGround && apStage >= 2 && s.atcPhase >= A320_ATC_PHASE_TOWER) landed = true;
    }
    if (s.atcPhase == A320_ATC_PHASE_DONE && !s.atcAwaitingReadback && s.groundSpeedKt < 5.0) break;
  }
  std::printf("  done after %.0f s: phase %d, landed %d, landing clearance %d, %.1f NM from the field\n", t, s.atcPhase,
              landed, s.atcLandingCleared, std::hypot(s.northM, s.eastM) / 1852.0);
  CHECK(radio.said("radar contact"));
  CHECK(radio.said("cleared ILS approach runway 26"));
  CHECK(radio.said("cleared to land"));
  CHECK(radio.said("vacate the runway"));
  CHECK(s.atcLandingCleared && landed);
  CHECK(s.atcPhase == A320_ATC_PHASE_DONE && !s.atcAwaitingReadback);
  CHECK((phaseSeen & (1 << A320_ATC_PHASE_RADAR)) && (phaseSeen & (1 << A320_ATC_PHASE_APPROACH)));
  a320_destroy(sim);
}

TEST(atc_approach_scenario_clears_the_ils) {
  char err[256] = {0};
  A320Sim* sim = a320_create(A320_DATA_DIR, err, sizeof(err));
  if (!sim) { CHECK(false); return; }
  a320_reset(sim, A320_SCENARIO_APPROACH, runwayIndex(sim, "26"));
  A320Controls c;
  a320_get_controls(sim, &c);
  CHECK(c.com1ActiveKhz == 127905 && c.xpdrMode == A320_XPDR_AUTO);
  Radio radio;
  run(sim, c, 8.0, radio);
  CHECK(radio.said("continue present heading, cleared ILS approach runway 26"));
  // Tuned away: the repeat isn't heard, and the tutor says where Radar is calling.
  c.com1ActiveKhz = 124880;
  run(sim, c, 30.0, radio);
  CHECK(std::strstr(state(sim).hint, "127.905") != nullptr);
  a320_destroy(sim);
}
