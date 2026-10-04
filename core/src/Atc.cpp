#include "a320/Atc.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "a320/Units.h"

namespace a320 {
namespace {

struct Station {
  int khz;
  const char* name;    // written, as in the log
  const char* spoken;  // as ATC says it
};
// EETN AD 2.18. Tower also gives the IFR clearance (AD 2.20).
const Station kStations[] = {{124880, "TALLINN INFORMATION", "Tallinn Information"},
                             {135905, "TALLINN TOWER", "Tallinn Tower"},
                             {127905, "TALLINN RADAR", "Tallinn Radar"},
                             {131905, "TALLINN HANDLING", "Tallinn Handling"}};
constexpr int kAtis = 0, kTower = 1, kRadar = 2, kHandling = 3;
constexpr int kStationCount = 4;

constexpr int kInitialAltFt = 4000;
constexpr int kApproachAltFt = 3000;
constexpr int kQnh = 1013;
constexpr int kTransitionAltFt = 5000;
constexpr size_t kLogSize = 80;

const char* kPhonetic[26] = {"Alpha", "Bravo", "Charlie", "Delta", "Echo", "Foxtrot", "Golf", "Hotel", "India",
                             "Juliett", "Kilo", "Lima", "Mike", "November", "Oscar", "Papa", "Quebec", "Romeo",
                             "Sierra", "Tango", "Uniform", "Victor", "Whiskey", "X-ray", "Yankee", "Zulu"};
const char* kDigit[10] = {"zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "niner"};

std::string fmt(const char* f, int v) {
  char b[32];
  std::snprintf(b, sizeof(b), f, v);
  return b;
}

// Each character of a code or number, as ICAO says it: "two three four one", "niner".
std::string spell(const std::string& s) {
  std::string out;
  for (const char c : s) {
    std::string w;
    if (std::isdigit(static_cast<unsigned char>(c))) w = kDigit[c - '0'];
    else if (std::isalpha(static_cast<unsigned char>(c))) w = kPhonetic[std::toupper(static_cast<unsigned char>(c)) - 'A'];
    else if (c == '.') w = "decimal";
    else continue;
    if (!out.empty()) out += ' ';
    out += w;
  }
  return out;
}

int wrapHeading(double deg) {
  int h = static_cast<int>(std::lround(deg)) % 360;
  if (h <= 0) h += 360;
  return h;
}

// Headings are given in tens of degrees, as controllers do.
int roundHeading(double deg) { return wrapHeading(std::round(deg / 10.0) * 10.0); }

double wrap180(double d) { return std::remainder(d, 360.0); }

std::string headingText(int h) { return fmt("%03d", h); }
std::string headingSpeech(int h) { return spell(headingText(h)); }

// "4000 feet" / "four thousand feet"; above the transition altitude, flight levels.
std::string altText(int ft) {
  if (ft > kTransitionAltFt) return fmt("flight level %03d", ft / 100);
  return fmt("altitude %d feet", ft);
}
std::string altSpeech(int ft) {
  if (ft > kTransitionAltFt) return "flight level " + spell(fmt("%d", ft / 100));
  std::string s = "altitude ";
  const int th = ft / 1000, hu = (ft % 1000) / 100;
  if (th > 0) s += std::string(kDigit[th]) + " thousand";
  if (hu > 0) s += std::string(th > 0 ? " " : "") + kDigit[hu] + " hundred";
  return s + " feet";
}
// Passing levels in check-ins, to the nearest hundred.
std::string passingSpeech(int ft) {
  const std::string a = altSpeech(ft);
  return a.substr(std::strlen("altitude "));
}

std::string freqText(int khz) {
  char b[16];
  std::snprintf(b, sizeof(b), "%d.%03d", khz / 1000, khz % 1000);
  return b;
}
std::string freqSpeech(int khz) { return spell(freqText(khz)); }

double estimateSeconds(const std::string& speech) {
  const long words = std::count(speech.begin(), speech.end(), ' ') + 1;
  return 0.6 + 0.36 * static_cast<double>(words);
}

uint32_t nextRandom(uint32_t& s) {
  s = s * 1664525u + 1013904223u;
  return s >> 8;
}

}  // namespace

Atc::Words& Atc::Words::add(const std::string& t, const std::string& s) {
  if (!text.empty() && !t.empty() && t[0] != ',') text += ' ';
  if (!speech.empty() && !s.empty() && s[0] != ',') speech += ' ';
  text += t;
  speech += s;
  return *this;
}

std::string Atc::callsign(const Fms& fms) const { return fms.flightNumber.empty() ? "SIM320" : fms.flightNumber; }

Atc::Words Atc::cs(const AtcContext& ctx) const {
  Words w;
  const std::string c = callsign(ctx.fms);
  return w.add(c, spell(c));
}

std::string Atc::stationName(int khz) const {
  for (const Station& s : kStations)
    if (s.khz == khz) return s.name;
  return "";
}

void Atc::reset(A320Scenario scenario, int runwayIndex, A320Controls& controls, uint32_t seed, int utcMinutes) {
  const bool enabled = enabled_;
  *this = Atc{};
  enabled_ = enabled;
  runway_ = runwayIndex;
  // A discrete code: no 0, 7500/7600/7700 or the conspicuity codes.
  uint32_t r = seed ^ 0x5eed1234u;
  squawk_ = static_cast<int>(2 + nextRandom(r) % 5) * 1000 + static_cast<int>(1 + nextRandom(r) % 7) * 100 +
            static_cast<int>(nextRandom(r) % 8) * 10 + static_cast<int>(1 + nextRandom(r) % 7);
  // ATIS is updated twice an hour (time hh20 and hh50), each a new letter.
  const int day = ((utcMinutes % 1440) + 1440) % 1440;
  const int issue = (day - 20 + 1440) % 1440 / 30;
  atisLetter_ = static_cast<char>('A' + issue % 26);
  const int issuedAt = (issue * 30 + 20) % 1440;
  atisTime_ = issuedAt / 60 * 100 + issuedAt % 60;

  const bool onGround = scenario == A320_SCENARIO_RUNWAY || scenario == A320_SCENARIO_COLD_DARK;
  if (onGround) {
    phase_ = A320_ATC_PHASE_CLEARANCE;
    controls.com1ActiveKhz = kStations[kTower].khz;
    controls.com1StandbyKhz = kStations[kAtis].khz;
    controls.xpdrCode = 2000;
    controls.xpdrMode = A320_XPDR_STBY;
  } else if (scenario == A320_SCENARIO_APPROACH) {
    // With Radar on a vector to the localizer: the approach clearance comes next.
    phase_ = A320_ATC_PHASE_RADAR;
    ifrCleared_ = takeoffCleared_ = radarContact_ = checkedInRadar_ = atisHeard_ = true;
    approachScenario_ = true;
    clearedAltFt_ = kApproachAltFt;
    controls.com1ActiveKhz = kStations[kRadar].khz;
    controls.com1StandbyKhz = kStations[kTower].khz;
    controls.xpdrCode = squawk_;
    controls.xpdrMode = A320_XPDR_AUTO;
  } else {
    // On final, just handed over to Tower: check in.
    phase_ = A320_ATC_PHASE_TOWER;
    ifrCleared_ = takeoffCleared_ = radarContact_ = checkedInRadar_ = approachCleared_ = atisHeard_ = true;
    clearedAltFt_ = kApproachAltFt;
    controls.com1ActiveKhz = kStations[kTower].khz;
    controls.com1StandbyKhz = kStations[kHandling].khz;
    controls.xpdrCode = squawk_;
    controls.xpdrMode = A320_XPDR_AUTO;
  }
  lastActiveKhz_ = controls.com1ActiveKhz;
}

void Atc::emit(int speaker, const std::string& station, int khz, const Words& w, const AtcContext& ctx) {
  A320AtcMessage m{};
  m.seq = ++seq_;
  m.speaker = speaker;
  m.frequencyKhz = khz;
  m.heard = khz == ctx.controls.com1ActiveKhz ? 1 : 0;
  m.simTimeS = now(ctx);
  std::snprintf(m.station, sizeof(m.station), "%s", station.c_str());
  std::snprintf(m.text, sizeof(m.text), "%s", w.text.c_str());
  std::snprintf(m.speech, sizeof(m.speech), "%s", w.speech.c_str());
  log_.push_back(m);
  while (log_.size() > kLogSize) log_.pop_front();
  // A crew transmission starts now (it may step on the end of ATC's); ATC's wait their turn.
  const double start = speaker == A320_ATC_SPEAKER_PILOT ? now(ctx) : std::max(busyUntil_, now(ctx));
  busyUntil_ = start + estimateSeconds(w.speech);
}

void Atc::say(int station, const Words& w, double delayS, const AtcContext& ctx) {
  pending_.push_back({now(ctx) + delayS, station, w});
}

void Atc::issue(Instruction in, const AtcContext& ctx, double delayS, const std::string& prefix,
                const std::string& prefixSpeech) {
  Words w = cs(ctx);
  if (!prefix.empty()) w.add(", " + prefix, ", " + prefixSpeech);
  w.add(", " + in.body.text, ", " + in.body.speech);
  in.issuedAt = now(ctx) + delayS;
  in.repeats = 0;
  ++instructionCount_;
  instr_ = in;
  last_ = in;
  awaiting_ = true;
  notOnFreqHinted_ = false;
  say(in.station, w, delayS, ctx);
}

Atc::Instruction Atc::headingInstruction(int headingMag, const AtcContext& ctx, const std::string& extra) const {
  const double current = ctx.state.headingTrueDeg - ctx.airport.magneticVariationDeg;
  const bool left = wrap180(headingMag - current) < 0.0;
  const std::string dir = left ? "left" : "right", other = left ? "right" : "left";
  Instruction in;
  in.kind = Kind::Heading;
  in.station = kRadar;
  in.heading = headingMag;
  in.body.add("turn " + dir + " heading " + headingText(headingMag), "turn " + dir + " heading " + headingSpeech(headingMag));
  const Words me = cs(ctx);
  Reply ok, wrongDir, wrongHdg;
  ok.words.add(dir == "left" ? "Left" : "Right").add("heading " + headingText(headingMag), "heading " + headingSpeech(headingMag));
  wrongDir.words.add(other == "left" ? "Left" : "Right").add("heading " + headingText(headingMag), "heading " + headingSpeech(headingMag));
  wrongDir.why = "The turn direction is part of the instruction: ATC said turn " + dir + ".";
  const int off = wrapHeading(headingMag + (instructionCount_ % 2 ? 20 : -20));
  wrongHdg.words.add(dir == "left" ? "Left" : "Right").add("heading " + headingText(off), "heading " + headingSpeech(off));
  wrongHdg.why = "Wrong heading: ATC said " + headingText(headingMag) + ". Read back the numbers exactly.";
  for (Reply* r : {&ok, &wrongDir, &wrongHdg}) {
    if (!extra.empty()) r->words.add(", " + extra);
    r->words.add(", " + me.text, ", " + me.speech);
  }
  in.replies = {ok, wrongDir, wrongHdg};
  return in;
}

void Atc::update(const AtcContext& ctx) {
  const A320State& s = ctx.state;
  const double t = now(ctx);
  const double dt = lastUpdate_ < 0.0 ? 0.0 : std::max(0.0, t - lastUpdate_);
  lastUpdate_ = t;
  if (!enabled_) return;
  const int active = ctx.controls.com1ActiveKhz;

  // ATIS: a loop while tuned, starting a moment after tuning in.
  if (active != lastActiveKhz_) {
    lastActiveKhz_ = active;
    nextAtisAt_ = t + 1.0;
  }
  if (active == kStations[kAtis].khz) {
    atisTunedS_ += dt;
    if (atisTunedS_ > 15.0) atisHeard_ = true;
    if (t >= nextAtisAt_) {
      Words w;
      const std::string rwy = ctx.airport.runways[static_cast<size_t>(runway_)].ident;
      const std::string letter(1, atisLetter_);
      char time[8];
      std::snprintf(time, sizeof(time), "%04d", atisTime_);
      w.add("This is Tallinn Information, information " + letter + ", time " + time + ".",
            "This is Tallinn Information, information " + spell(letter) + ", time " + spell(time) + ".");
      w.add("Runway in use " + rwy + ". Expect ILS approach.", "Runway in use " + spell(rwy) + ". Expect I L S approach.");
      w.add("Transition level 60.", "Transition level " + spell("60") + ".");
      w.add("Wind calm. Visibility 10 kilometres or more, no significant cloud.",
            "Wind calm. Visibility one zero kilometres or more, no significant cloud.");
      w.add("Temperature 15, dew point 9. QNH 1013.", "Temperature one five, dew point niner. Q N H one zero one three.");
      w.add("Acknowledge information " + letter + " on first contact.",
            "Acknowledge information " + spell(letter) + " on first contact.");
      emit(A320_ATC_SPEAKER_ATIS, kStations[kAtis].name, kStations[kAtis].khz, w, ctx);
      nextAtisAt_ = t + estimateSeconds(w.speech) + 4.0;
    }
  }

  // Controllers talk when the frequency is free, a moment after the last transmission.
  while (!pending_.empty() && t >= pending_.front().at && t >= busyUntil_) {
    const Pending p = pending_.front();
    pending_.pop_front();
    emit(A320_ATC_SPEAKER_ATC, kStations[p.station].name, kStations[p.station].khz, p.words, ctx);
  }
  if (!pending_.empty()) return;

  // An instruction not read back: say it again, then ask.
  if (awaiting_ && t > instr_.issuedAt) {
    const double waited = t - instr_.issuedAt;
    const Station& st = kStations[instr_.station];
    if (active != st.khz && waited > 6.0 && !notOnFreqHinted_) {
      notOnFreqHinted_ = true;
      hint_ = std::string(st.spoken) + " is calling you on " + freqText(st.khz) + ", but COM 1 is on " +
              freqText(active) + ". Tune the RADIO panel back.";
    }
    if (waited > 20.0 && instr_.repeats == 0) {
      instr_.repeats = 1;
      Words w = cs(ctx);
      w.add(", " + instr_.body.text, ", " + instr_.body.speech);
      say(instr_.station, w, 0.0, ctx);
      instr_.issuedAt = t;
    } else if (waited > 25.0 && instr_.repeats == 1) {
      instr_.repeats = 2;
      Words w = cs(ctx);
      w.add(", " + std::string(st.spoken) + ", how do you read?");
      say(instr_.station, w, 0.0, ctx);
      instr_.issuedAt = t;
    }
    return;
  }

  // Takeoff without a takeoff clearance, landing without a landing clearance.
  if (s.onGround && !takeoffCleared_ && s.iasKt > 40.0 && !noTakeoffHinted_ && phase_ <= A320_ATC_PHASE_DEPARTURE) {
    noTakeoffHinted_ = true;
    hint_ = "ATC: you are taking off without a takeoff clearance. Ask Tower: \"ready for departure\", and read back "
            "\"cleared for takeoff\" first.";
  }
  if (!s.onGround && takeoffCleared_ && !landingCleared_ && s.radioAltFt < 500.0 && s.verticalSpeedFpm < -200.0 &&
      (phase_ == A320_ATC_PHASE_APPROACH || phase_ == A320_ATC_PHASE_TOWER) && !noLandingHinted_) {
    noLandingHinted_ = true;
    hint_ = "ATC: no landing clearance yet. Below 500 ft without one, go around (TOGA). Contact Tower in time.";
  }

  switch (phase_) {
    case A320_ATC_PHASE_DEPARTURE:
      if (!s.onGround && s.radioAltFt > 800.0 && s.verticalSpeedFpm > 0.0) {
        Instruction in;
        in.kind = Kind::ContactRadar;
        in.station = kTower;
        in.freqKhz = kStations[kRadar].khz;
        in.body.add("contact Tallinn Radar " + freqText(in.freqKhz) + ", goodbye",
                    "contact Tallinn Radar " + freqSpeech(in.freqKhz) + ", goodbye");
        const Words me = cs(ctx);
        Reply ok, wrongFreq, roger;
        ok.words.add("Tallinn Radar " + freqText(in.freqKhz), "Tallinn Radar " + freqSpeech(in.freqKhz));
        wrongFreq.words.add("Tallinn Radar 127.950", "Tallinn Radar " + freqSpeech(127950));
        wrongFreq.why = "Wrong frequency: Radar is 127.905. A wrong readback here puts you on a frequency nobody "
                        "listens to.";
        roger.words.add("Roger, goodbye");
        roger.why = "Frequency changes are always read back with the frequency.";
        for (Reply* r : {&ok, &wrongFreq, &roger}) r->words.add(", " + me.text, ", " + me.speech);
        in.replies = {ok, wrongFreq, roger};
        phase_ = A320_ATC_PHASE_RADAR;
        issue(in, ctx, 0.5);
      }
      break;
    case A320_ATC_PHASE_RADAR:
      if (checkedInRadar_ && !radarContact_) {
        const bool replying = ctx.controls.xpdrMode != A320_XPDR_STBY && ctx.controls.xpdrCode == squawk_;
        if (replying) {
          radarContact_ = true;
          vectors(ctx);
        } else if (!squawkAsked_) {
          squawkAsked_ = true;
          Instruction in;
          in.kind = Kind::Squawk;
          in.station = kRadar;
          const std::string code = fmt("%04d", squawk_);
          in.body.add("not identified, squawk " + code, "not identified, squawk " + spell(code));
          const Words me = cs(ctx);
          Reply ok, wrong;
          ok.words.add("Squawk " + code, "squawk " + spell(code));
          std::string swapped = code;
          std::swap(swapped[2], swapped[3]);
          wrong.words.add("Squawk " + swapped, "squawk " + spell(swapped));
          wrong.why = "Wrong code: the squawk is " + code + ". Set it on the transponder (ATC panel) and switch it to AUTO.";
          for (Reply* r : {&ok, &wrong}) r->words.add(", " + me.text, ", " + me.speech);
          in.replies = {ok, wrong};
          issue(in, ctx, 1.5);
          hint_ = "Radar can't see you: set the transponder code " + code + " and the mode AUTO on the RADIO panel.";
        }
      } else if (radarContact_) {
        vectors(ctx);
      }
      monitor(ctx);
      break;
    case A320_ATC_PHASE_APPROACH: {
      const Runway& r = ctx.airport.runways[static_cast<size_t>(runway_)];
      const RunwayPoint p = RunwayAxes(ctx.frame, r).fromEnu({s.eastM, s.northM, 0.0});
      const bool established = s.latMode == A320_LAT_LOC || s.latMode == A320_LAT_LOC_STAR;
      if ((established && p.x > -15.0 * kNmToM) || p.x > -8.0 * kNmToM) {
        Instruction in;
        in.kind = Kind::ContactTower;
        in.station = kRadar;
        in.freqKhz = kStations[kTower].khz;
        in.body.add("contact Tallinn Tower " + freqText(in.freqKhz), "contact Tallinn Tower " + freqSpeech(in.freqKhz));
        const Words me = cs(ctx);
        Reply ok, wrongFreq;
        ok.words.add("Tower " + freqText(in.freqKhz), "Tower " + freqSpeech(in.freqKhz));
        wrongFreq.words.add("Tower 135.950", "Tower " + freqSpeech(135950));
        wrongFreq.why = "Wrong frequency: Tower is 135.905.";
        for (Reply* rr : {&ok, &wrongFreq}) rr->words.add(", " + me.text, ", " + me.speech);
        in.replies = {ok, wrongFreq};
        phase_ = A320_ATC_PHASE_TOWER;
        issue(in, ctx, 1.0);
      }
      monitor(ctx);
      break;
    }
    case A320_ATC_PHASE_TOWER:
      if (s.onGround && s.groundSpeedKt < 40.0) phase_ = A320_ATC_PHASE_LANDED;
      break;
    case A320_ATC_PHASE_LANDED: {
      Instruction in;
      in.kind = Kind::Vacate;
      in.station = kTower;
      in.freqKhz = kStations[kHandling].khz;
      in.body.add("vacate the runway when able, contact Tallinn Handling " + freqText(in.freqKhz) + ", goodbye",
                  "vacate the runway when able, contact Tallinn Handling " + freqSpeech(in.freqKhz) + ", goodbye");
      const Words me = cs(ctx);
      Reply ok, roger;
      ok.words.add("Vacate when able, Handling " + freqText(in.freqKhz), "vacate when able, Handling " + freqSpeech(in.freqKhz));
      roger.words.add("Roger");
      roger.why = "Read back the frequency change: Handling " + freqText(in.freqKhz) + ".";
      for (Reply* r : {&ok, &roger}) r->words.add(", " + me.text, ", " + me.speech);
      in.replies = {ok, roger};
      phase_ = A320_ATC_PHASE_DONE;
      issue(in, ctx, 2.0);
      break;
    }
    default:
      break;
  }
}

// Radar vectors to the ILS: downwind to a point 17 NM out and 5 NM to the side, a base turn,
// then a 30 degree intercept with the approach clearance, so the localizer is captured about
// 12 NM out and the glideslope from below at 3000 ft.
void Atc::vectors(const AtcContext& ctx) {
  if (approachCleared_ || awaiting_) return;
  const A320State& s = ctx.state;
  const Runway& r = ctx.airport.runways[static_cast<size_t>(runway_)];
  const RunwayAxes axes(ctx.frame, r);
  const RunwayPoint p = axes.fromEnu({s.eastM, s.northM, 0.0});
  const double courseMag = r.ils.courseMagDeg;
  const double var = ctx.airport.magneticVariationDeg;
  const double hdgMag = s.headingTrueDeg - var;
  const std::string rwy = r.ident;

  if (approachScenario_) {
    // Already on an intercept heading: clear the approach straight away.
    approachScenario_ = false;
    Instruction in;
    in.kind = Kind::Approach;
    in.station = kRadar;
    in.heading = roundHeading(hdgMag);
    in.body.add("continue present heading, cleared ILS approach runway " + rwy,
                "continue present heading, cleared I L S approach runway " + spell(rwy));
    const Words me = cs(ctx);
    Reply ok, wrongRwy, roger;
    ok.words.add("Present heading, cleared ILS approach runway " + rwy,
                 "present heading, cleared I L S approach runway " + spell(rwy));
    const std::string other = ctx.airport.runways.size() > 1 ? ctx.airport.runways[static_cast<size_t>(1 - runway_)].ident : rwy;
    wrongRwy.words.add("Present heading, cleared ILS approach runway " + other,
                       "present heading, cleared I L S approach runway " + spell(other));
    wrongRwy.why = "Wrong runway: the clearance is for runway " + rwy + ".";
    roger.words.add("Roger");
    roger.why = "An approach clearance is read back in full: heading and \"cleared ILS approach runway " + rwy + "\".";
    for (Reply* rr : {&ok, &wrongRwy, &roger}) rr->words.add(", " + me.text, ", " + me.speech);
    in.replies = {ok, wrongRwy, roger};
    issue(in, ctx, 3.0);
    return;
  }

  // The side of the final approach course the aircraft is on (chosen once, at radar contact).
  if (vectorStage_ == 0) side_ = p.y > 0.0 ? 1 : -1;
  const double dx = -17.0 * kNmToM - p.x, dy = side_ * 5.0 * kNmToM - p.y;
  const Enu target = axes.toEnu({-17.0 * kNmToM, side_ * 5.0 * kNmToM, 0.0});
  const double brgTrue = std::atan2(target.e - s.eastM, target.n - s.northM) * kRadToDeg;
  const double dist = std::hypot(dx, dy);

  if (vectorStage_ == 0) {
    vectorStage_ = 1;
    Instruction in = headingInstruction(roundHeading(brgTrue - var), ctx);
    issue(in, ctx, 2.0, "Tallinn Radar, radar contact", "Tallinn Radar, radar contact");
    return;
  }
  if (vectorStage_ == 1) {
    if (!descentIssued_ && clearedAltFt_ > kApproachAltFt && p.x < -4.0 * kNmToM) {
      descentIssued_ = true;
      Instruction in;
      in.kind = Kind::Altitude;
      in.station = kRadar;
      in.altFt = kApproachAltFt;
      in.body.add("descend " + altText(kApproachAltFt) + ", QNH 1013",
                  "descend " + altSpeech(kApproachAltFt) + ", Q N H one zero one three");
      const Words me = cs(ctx);
      Reply ok, wrongAlt, wrongQnh;
      ok.words.add("Descend " + altText(kApproachAltFt) + ", QNH 1013",
                   "descend " + altSpeech(kApproachAltFt) + ", Q N H one zero one three");
      wrongAlt.words.add("Descend " + altText(2000) + ", QNH 1013", "descend " + altSpeech(2000) + ", Q N H one zero one three");
      wrongAlt.why = "Wrong altitude: ATC cleared 3000 ft. Descending below a cleared altitude is a level bust.";
      wrongQnh.words.add("Descend " + altText(kApproachAltFt) + ", QNH 1003",
                         "descend " + altSpeech(kApproachAltFt) + ", Q N H one zero zero three");
      wrongQnh.why = "Wrong QNH: 1013. A 10 hPa error puts the aircraft 280 ft off its altitude.";
      for (Reply* rp : {&ok, &wrongAlt, &wrongQnh}) rp->words.add(", " + me.text, ", " + me.speech);
      in.replies = {ok, wrongAlt, wrongQnh};
      issue(in, ctx, 1.0);
      return;
    }
    // Base turn abeam the point, or when there.
    if (dist < 2.0 * kNmToM || p.x < -17.0 * kNmToM) {
      vectorStage_ = 2;
      issue(headingInstruction(wrapHeading(courseMag - side_ * 90.0), ctx), ctx, 1.0);
      return;
    }
    // Re-vector once the aircraft flies the heading but the point has moved off it.
    const int wanted = roundHeading(brgTrue - var);
    if (headingMag_ > 0 && dist > 5.0 * kNmToM && std::fabs(wrap180(wanted - headingMag_)) >= 30.0 &&
        std::fabs(wrap180(hdgMag - headingMag_)) < 15.0 && now(ctx) - headingSetAt_ > 45.0) {
      issue(headingInstruction(wanted, ctx), ctx, 1.0);
    }
    return;
  }
  if (vectorStage_ == 2 && (std::fabs(p.y) < 3.2 * kNmToM || p.y * side_ < 0.0)) {
    const int intercept = wrapHeading(courseMag - side_ * 30.0);
    const bool slow = s.iasKt > 195.0;
    const std::string speedText = slow ? ", reduce speed 180 knots" : "";
    Instruction in = headingInstruction(intercept, ctx, "cleared ILS approach runway " + rwy + speedText);
    in.kind = Kind::Approach;
    in.speedKt = slow ? 180 : 0;
    in.body = Words{};
    const double current = s.headingTrueDeg - var;
    const std::string dir = wrap180(intercept - current) < 0.0 ? "left" : "right";
    in.body.add("turn " + dir + " heading " + headingText(intercept) + ", cleared ILS approach runway " + rwy + speedText,
                "turn " + dir + " heading " + headingSpeech(intercept) + ", cleared I L S approach runway " + spell(rwy) +
                    (slow ? ", reduce speed one eight zero knots" : ""));
    // The replies from headingInstruction carry "cleared ILS approach runway" as written; give the
    // spoken form too, and a wrong-runway alternative.
    for (Reply& rr : in.replies) {
      const std::string from = "cleared ILS approach runway " + rwy + speedText;
      const size_t at = rr.words.speech.find(from);
      if (at != std::string::npos)
        rr.words.speech.replace(at, from.size(), "cleared I L S approach runway " + spell(rwy) +
                                                     (slow ? ", reduce speed one eight zero knots" : ""));
    }
    if (in.replies.size() == 3) {
      const std::string other = ctx.airport.runways.size() > 1 ? ctx.airport.runways[static_cast<size_t>(1 - runway_)].ident : rwy;
      Reply& wrong = in.replies[2];
      wrong = in.replies[0];
      const std::string from = "runway " + rwy;
      size_t at = wrong.words.text.find(from);
      if (at != std::string::npos) wrong.words.text.replace(at, from.size(), "runway " + other);
      at = wrong.words.speech.find("runway " + spell(rwy));
      if (at != std::string::npos) wrong.words.speech.replace(at, ("runway " + spell(rwy)).size(), "runway " + spell(other));
      wrong.why = "Wrong runway: the approach clearance is for runway " + rwy + ".";
    }
    issue(in, ctx, 1.0);
  }
}

// Queries a heading or altitude that isn't being flown, once per instruction.
void Atc::monitor(const AtcContext& ctx) {
  if (awaiting_ || !pending_.empty()) return;
  const A320State& s = ctx.state;
  const double t = now(ctx);
  // Time to set the FCU and turn at about 3 degrees per second, plus a margin.
  const double allowed = 25.0 + headingTurnDeg_ / 2.0;
  if (headingMag_ > 0 && !headingQueried_ && !approachCleared_ && t - headingSetAt_ > allowed) {
    const double hdgMag = s.headingTrueDeg - ctx.airport.magneticVariationDeg;
    if (std::fabs(wrap180(hdgMag - headingMag_)) > 25.0) {
      headingQueried_ = true;
      Instruction in = headingInstruction(headingMag_, ctx);
      issue(in, ctx, 0.5, "check heading", "check heading");
      hint_ = "ATC expects heading " + headingText(headingMag_) + ". Set it on the FCU (HDG) and pull the knob [Key: U].";
      return;
    }
  }
  if (clearedAltFt_ > 0 && !altQueried_ && !s.onGround) {
    const double err = s.altitudeFt - clearedAltFt_;
    const bool bust = (err > 400.0 && s.verticalSpeedFpm > 0.0) || (err < -400.0 && s.verticalSpeedFpm < 0.0 && t - altSetAt_ > 5.0);
    if (bust && t - altSetAt_ > 20.0) {
      altQueried_ = true;
      Instruction in;
      in.kind = Kind::Altitude;
      in.station = kRadar;
      in.altFt = clearedAltFt_;
      in.body.add("check altitude, maintain " + altText(clearedAltFt_), "check altitude, maintain " + altSpeech(clearedAltFt_));
      const Words me = cs(ctx);
      Reply ok;
      ok.words.add("Maintain " + altText(clearedAltFt_), "maintain " + altSpeech(clearedAltFt_));
      ok.words.add(", " + me.text, ", " + me.speech);
      in.replies = {ok};
      issue(in, ctx, 0.5);
      hint_ = "Level bust: you were cleared to " + std::to_string(clearedAltFt_) + " ft. Set it in the FCU ALT window.";
    }
  }
}

std::vector<Atc::Option> Atc::buildOptions(const AtcContext& ctx) const {
  std::vector<Option> out;
  if (!enabled_) return out;
  const A320State& s = ctx.state;
  const int active = ctx.controls.com1ActiveKhz;
  const Words me = cs(ctx);
  const std::string rwy = ctx.airport.runways[static_cast<size_t>(runway_)].ident;
  auto on = [&](int station) { return active == kStations[station].khz; };

  if (awaiting_ && on(instr_.station) && pending_.empty()) {
    // The correct readback among the wrong ones, in a different place each time.
    const size_t n = instr_.replies.size();
    for (size_t k = 0; k < n; ++k) {
      const size_t i = (k + static_cast<size_t>(instructionCount_)) % n;
      Option o;
      o.words = instr_.replies[i].words;
      o.reply = static_cast<int>(i);
      out.push_back(o);
    }
    Option again;
    again.words.add("Say again", "say again").add(", " + me.text, ", " + me.speech);
    again.request = Request::SayAgain;
    out.push_back(again);
    return out;
  }
  // Nothing new to say while waiting for an answer, or while ATC is about to speak.
  if (awaiting_ || !pending_.empty()) return out;

  const std::string letter(1, atisLetter_);
  if (phase_ == A320_ATC_PHASE_CLEARANCE && s.onGround && on(kTower)) {
    Option o;
    o.words.add("Tallinn Tower,", "Tallinn Tower,").add(me.text + ", A320 at runway " + rwy,
                                                       me.speech + ", A three twenty at runway " + spell(rwy));
    if (atisHeard_) o.words.add("with information " + letter, "with information " + spell(letter));
    o.words.add(", request IFR clearance to Tallinn", ", request I F R clearance to Tallinn");
    o.request = Request::Clearance;
    out.push_back(o);
  }
  if (phase_ == A320_ATC_PHASE_DEPARTURE && s.onGround && on(kTower) && !takeoffCleared_) {
    Option o;
    o.words.add(me.text + ", ready for departure runway " + rwy, me.speech + ", ready for departure runway " + spell(rwy));
    o.request = Request::Ready;
    out.push_back(o);
  }
  if (phase_ == A320_ATC_PHASE_RADAR && !checkedInRadar_ && on(kRadar)) {
    const int passing = static_cast<int>(std::lround(s.altitudeFt / 100.0)) * 100;
    const int hdg = roundHeading(s.headingTrueDeg - ctx.airport.magneticVariationDeg);
    Option o;
    o.words.add("Tallinn Radar, " + me.text + ",", "Tallinn Radar, " + me.speech + ",");
    if (std::fabs(s.verticalSpeedFpm) > 300.0 && clearedAltFt_ > 0)
      o.words.add(fmt("passing %d feet", passing) + ", climbing " + altText(clearedAltFt_),
                  "passing " + passingSpeech(passing) + ", climbing " + altSpeech(clearedAltFt_));
    else
      o.words.add(fmt("maintaining %d feet", passing), "maintaining " + passingSpeech(passing));
    o.words.add(", heading " + headingText(hdg), ", heading " + headingSpeech(hdg));
    o.request = Request::CheckInRadar;
    out.push_back(o);
  }
  if (phase_ == A320_ATC_PHASE_TOWER && !checkedInTower_ && on(kTower)) {
    Option o;
    o.words.add("Tallinn Tower, " + me.text + ", established ILS runway " + rwy,
                "Tallinn Tower, " + me.speech + ", established I L S runway " + spell(rwy));
    o.request = Request::CheckInTower;
    out.push_back(o);
  }
  if (phase_ == A320_ATC_PHASE_RADAR && radarContact_ && !approachCleared_ && on(kRadar)) {
    for (size_t i = 0; i < ctx.airport.runways.size() && out.size() + 1 < A320_ATC_MAX_OPTIONS; ++i) {
      if (static_cast<int>(i) == runway_) continue;
      const std::string other = ctx.airport.runways[i].ident;
      Option o;
      o.words.add(me.text + ", request ILS approach runway " + other,
                  me.speech + ", request I L S approach runway " + spell(other));
      o.request = Request::OtherRunway;
      o.runway = static_cast<int>(i);
      out.push_back(o);
    }
  }
  if (last_.kind != Kind::None && phase_ != A320_ATC_PHASE_DONE && on(last_.station)) {
    Option again;
    again.words.add("Say again", "say again").add(", " + me.text, ", " + me.speech);
    again.request = Request::SayAgain;
    out.push_back(again);
  }
  if (out.size() > A320_ATC_MAX_OPTIONS) out.resize(A320_ATC_MAX_OPTIONS);
  return out;
}

std::vector<std::string> Atc::options(const AtcContext& ctx) const {
  std::vector<std::string> out;
  for (const Option& o : buildOptions(ctx)) out.push_back(o.words.text);
  return out;
}

void Atc::choose(int option, const AtcContext& ctx) {
  const std::vector<Option> opts = buildOptions(ctx);
  if (option < 0 || option >= static_cast<int>(opts.size())) return;
  const Option o = opts[static_cast<size_t>(option)];
  const int active = ctx.controls.com1ActiveKhz;
  emit(A320_ATC_SPEAKER_PILOT, callsign(ctx.fms), active, o.words, ctx);
  const double replyDelay = 1.2;

  if (o.reply >= 0) {
    const Reply& r = instr_.replies[static_cast<size_t>(o.reply)];
    if (r.why.empty()) {
      awaiting_ = false;
      readbackDone(instr_, ctx);
    } else {
      // Corrected by the controller; still waiting for the right readback.
      hint_ = "Wrong readback. " + r.why;
      Words w = cs(ctx);
      w.add(", negative, I say again, " + instr_.body.text, ", negative, I say again, " + instr_.body.speech);
      say(instr_.station, w, replyDelay, ctx);
      instr_.issuedAt = now(ctx) + replyDelay + 3.0;
      instr_.repeats = 0;
    }
    return;
  }
  request(o, ctx);
}

void Atc::readbackDone(const Instruction& in, const AtcContext& ctx) {
  const double t = now(ctx);
  switch (in.kind) {
    case Kind::Clearance: {
      ifrCleared_ = true;
      clearedAltFt_ = in.altFt;
      altSetAt_ = t;
      phase_ = A320_ATC_PHASE_DEPARTURE;
      Words w = cs(ctx);
      w.add(", readback correct, report ready for departure");
      say(kTower, w, 1.2, ctx);
      break;
    }
    case Kind::Takeoff: takeoffCleared_ = true; break;
    case Kind::ContactRadar: break;
    case Kind::Squawk: break;
    case Kind::Heading: {
      headingMag_ = in.heading;
      headingSetAt_ = t;
      headingQueried_ = false;
      const double hdgMag = ctx.state.headingTrueDeg - ctx.airport.magneticVariationDeg;
      headingTurnDeg_ = std::fabs(wrap180(in.heading - hdgMag));
      break;
    }
    case Kind::Altitude:
      clearedAltFt_ = in.altFt;
      altSetAt_ = t;
      altQueried_ = false;
      break;
    case Kind::Approach:
      approachCleared_ = true;
      headingMag_ = in.heading;
      headingSetAt_ = t;
      if (in.speedKt > 0) speedKt_ = in.speedKt;
      phase_ = A320_ATC_PHASE_APPROACH;
      break;
    case Kind::ContactTower: break;
    case Kind::Landing: landingCleared_ = true; break;
    case Kind::Vacate: break;
    case Kind::None: break;
  }
}

void Atc::request(const Option& o, const AtcContext& ctx) {
  const std::string rwy = ctx.airport.runways[static_cast<size_t>(runway_)].ident;
  const Words me = cs(ctx);
  const double delay = 1.5;
  switch (o.request) {
    case Request::Clearance: {
      Instruction in;
      in.kind = Kind::Clearance;
      in.station = kTower;
      in.altFt = kInitialAltFt;
      const std::string code = fmt("%04d", squawk_);
      in.body.add("cleared to Tallinn via radar vectors, after departure runway heading, climb " + altText(kInitialAltFt) +
                      ", squawk " + code,
                  "cleared to Tallinn via radar vectors, after departure runway heading, climb " + altSpeech(kInitialAltFt) +
                      ", squawk " + spell(code));
      Reply ok, wrongAlt, wrongCode;
      ok.words.add("Cleared to Tallinn via radar vectors, runway heading, climb " + altText(kInitialAltFt) + ", squawk " + code,
                   "cleared to Tallinn via radar vectors, runway heading, climb " + altSpeech(kInitialAltFt) + ", squawk " + spell(code));
      wrongAlt.words.add("Cleared to Tallinn via radar vectors, runway heading, climb " + altText(5000) + ", squawk " + code,
                         "cleared to Tallinn via radar vectors, runway heading, climb " + altSpeech(5000) + ", squawk " + spell(code));
      wrongAlt.why = "Wrong altitude: the clearance limit is 4000 ft. Set 4000 in the FCU ALT window.";
      std::string swapped = code;
      std::swap(swapped[1], swapped[2]);
      wrongCode.words.add("Cleared to Tallinn via radar vectors, runway heading, climb " + altText(kInitialAltFt) + ", squawk " + swapped,
                          "cleared to Tallinn via radar vectors, runway heading, climb " + altSpeech(kInitialAltFt) +
                              ", squawk " + spell(swapped));
      wrongCode.why = "Wrong squawk: the code is " + code + ".";
      for (Reply* r : {&ok, &wrongAlt, &wrongCode}) r->words.add(", " + me.text, ", " + me.speech);
      in.replies = {ok, wrongAlt, wrongCode};
      if (!atisHeard_) {
        const std::string letter(1, atisLetter_);
        issue(in, ctx, delay, "information " + letter + " is current, QNH 1013",
              "information " + spell(letter) + " is current, Q N H one zero one three");
      } else {
        issue(in, ctx, delay);
      }
      break;
    }
    case Request::Ready: {
      if (!ifrCleared_) {
        Words w = me;
        w.add(", negative, you have no IFR clearance yet. Request your clearance first.");
        say(kTower, w, delay, ctx);
        hint_ = "IFR flights need a clearance before departure: request it from Tower first.";
        break;
      }
      Instruction in;
      in.kind = Kind::Takeoff;
      in.station = kTower;
      in.body.add("wind calm, runway " + rwy + ", cleared for takeoff", "wind calm, runway " + spell(rwy) + ", cleared for takeoff");
      const std::string other = ctx.airport.runways.size() > 1 ? ctx.airport.runways[static_cast<size_t>(1 - runway_)].ident : rwy;
      Reply ok, wrongRwy, roger;
      ok.words.add("Cleared for takeoff runway " + rwy, "cleared for takeoff runway " + spell(rwy));
      wrongRwy.words.add("Cleared for takeoff runway " + other, "cleared for takeoff runway " + spell(other));
      wrongRwy.why = "You are on runway " + rwy + ". A wrong runway in a takeoff readback is a classic error ATC must "
                     "catch.";
      roger.words.add("Roger, rolling");
      roger.why = "A takeoff clearance is read back with the runway: \"cleared for takeoff runway " + rwy + "\".";
      for (Reply* r : {&ok, &wrongRwy, &roger}) r->words.add(", " + me.text, ", " + me.speech);
      in.replies = {ok, wrongRwy, roger};
      issue(in, ctx, delay);
      break;
    }
    case Request::CheckInRadar:
      checkedInRadar_ = true;  // radar contact (or the squawk query) follows in update()
      break;
    case Request::CheckInTower: {
      checkedInTower_ = true;
      Instruction in;
      in.kind = Kind::Landing;
      in.station = kTower;
      in.body.add("wind calm, runway " + rwy + ", cleared to land", "wind calm, runway " + spell(rwy) + ", cleared to land");
      Reply ok, roger;
      ok.words.add("Cleared to land runway " + rwy, "cleared to land runway " + spell(rwy));
      roger.words.add("Roger");
      roger.why = "Landing clearances are read back with the runway: \"cleared to land runway " + rwy + "\".";
      for (Reply* r : {&ok, &roger}) r->words.add(", " + me.text, ", " + me.speech);
      in.replies = {ok, roger};
      issue(in, ctx, delay, "Tallinn Tower", "Tallinn Tower");
      break;
    }
    case Request::OtherRunway: {
      runway_ = o.runway;
      vectorStage_ = 0;  // new vectors for the other runway
      const std::string other = ctx.airport.runways[static_cast<size_t>(runway_)].ident;
      Words w = me;
      w.add(", roger, expect ILS approach runway " + other, ", roger, expect I L S approach runway " + spell(other));
      say(kRadar, w, delay, ctx);
      hint_ = "Also select the approach on the MCDU (F-PLN > ARRIVAL > ILS" + other + ") so the right ILS is tuned.";
      break;
    }
    case Request::SayAgain: {
      const Instruction& again = awaiting_ ? instr_ : last_;
      Words w = me;
      w.add(", I say again, " + again.body.text, ", I say again, " + again.body.speech);
      say(again.station, w, delay, ctx);
      if (awaiting_) instr_.issuedAt = now(ctx) + delay + 3.0;
      break;
    }
  }
}

bool Atc::message(uint32_t seq, A320AtcMessage& out) const {
  for (const A320AtcMessage& m : log_) {
    if (m.seq == seq) {
      out = m;
      return true;
    }
  }
  return false;
}

void Atc::fillState(A320State& s) const {
  s.atcEnabled = enabled_ ? 1 : 0;
  s.atcPhase = phase_;
  s.atcClearedAltFt = clearedAltFt_;
  s.atcHeadingMag = headingMag_;
  s.atcSpeedKt = speedKt_;
  s.atcSquawk = ifrCleared_ ? squawk_ : -1;
  s.atcIfrCleared = ifrCleared_;
  s.atcTakeoffCleared = takeoffCleared_;
  s.atcApproachCleared = approachCleared_;
  s.atcLandingCleared = landingCleared_;
  s.atcRadarContact = radarContact_;
  s.atcAwaitingReadback = awaiting_;
  s.atcRunwayIndex = runway_;
  s.atcMessageSeq = seq_;
}

std::string Atc::takeHint() {
  std::string h;
  h.swap(hint_);
  return h;
}

}  // namespace a320
