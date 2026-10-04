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
  bool afis;           // an AFIS officer informs and relays, but gives no clearances
  bool traffic;        // no ATS at all: pilots announce themselves, nobody answers
};
// AD 2.18 of each airport (eAIP, AIRAC 2026-10-01). Tallinn Tower also gives the IFR clearance
// (EETN AD 2.20); Tallinn Radar serves the whole country and relays clearances through the AFIS
// units, or gives them itself where there is no ATS (Kihnu). Names: written in ASCII for the log,
// spoken in UTF-8 for the voice.
const Station kStations[] = {
    {124880, "TALLINN INFORMATION", "Tallinn Information", false, false},
    {135905, "TALLINN TOWER", "Tallinn Tower", false, false},
    {127905, "TALLINN RADAR", "Tallinn Radar", false, false},
    {131905, "TALLINN HANDLING", "Tallinn Handling", false, false},
    {118055, "KURESSAARE INFORMATION", "Kuressaare Information", true, false},
    {133905, "TARTU INFORMATION", "Tartu Information", true, false},
    {123130, "TARTU INFORMATION", "Tartu Information", false, false},  // ATIS
    {135305, "PARNU INFORMATION", "P\xC3\xA4rnu Information", true, false},
    {133405, "KARDLA INFORMATION", "K\xC3\xA4rdla Information", true, false},
    {118055, "RUHNU RADIO", "Ruhnu Radio", true, false},
    {135305, "KIHNU TRAFFIC", "Kihnu Traffic", false, true}};
constexpr int kAtis = 0, kTower = 1, kRadar = 2, kHandling = 3, kKuressaare = 4, kTartu = 5, kTartuAtis = 6,
              kParnu = 7, kKardla = 8, kRuhnu = 9, kKihnu = 10;

int towerOf(const Airport& a) {
  if (a.icao == "EEKE") return kKuressaare;
  if (a.icao == "EETU") return kTartu;
  if (a.icao == "EEPU") return kParnu;
  if (a.icao == "EEKA") return kKardla;
  if (a.icao == "EERU") return kRuhnu;
  if (a.icao == "EEKU") return kKihnu;
  return kTower;
}
int atisOf(const Airport& a) { return a.icao == "EETN" ? kAtis : a.icao == "EETU" ? kTartuAtis : -1; }
int groundOf(const Airport& a) { return a.icao == "EETN" ? kHandling : -1; }

constexpr int kInitialAltFt = 4000;
constexpr int kApproachAltFt = 3000;
constexpr int kQnh = 1013;
constexpr int kTransitionAltFt = 5000;
constexpr int kCruiseAltFt = 9000;
constexpr double kLongRouteNm = 40.0;
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
// A plausible misheard frequency for a wrong readback: 135.905 -> 135.950.
int confusable(int khz) { return khz % 100 == 5 ? khz + 45 : khz + 100; }

std::string approachText(const Runway& r) { return r.ils.ident.empty() ? "visual approach" : "ILS approach"; }
std::string approachSpeech(const Runway& r) { return r.ils.ident.empty() ? "visual approach" : "I L S approach"; }

// Another landing direction at the same airport, for wrong-runway readbacks and requests.
std::string otherIdent(const World& w, int runway) {
  const Runway& r = w.runways[static_cast<size_t>(runway)];
  for (const Runway& o : w.runways)
    if (o.airport == r.airport && o.ident != r.ident) return o.ident;
  return r.ident;
}

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
  // Some frequencies are shared (Kuressaare and Ruhnu, P\xC3\xA4rnu and Kihnu): this flight's first.
  for (const int i : {clearanceStation_, depStation_, arrStation_, kRadar})
    if (kStations[i].khz == khz) return kStations[i].name;
  for (const Station& s : kStations)
    if (s.khz == khz) return s.name;
  return "";
}

int Atc::atisRunway() const { return phase_ <= A320_ATC_PHASE_DEPARTURE ? depRunway_ : runway_; }

void Atc::reset(const World& world, A320Scenario scenario, int depRunway, int arrRunway, A320Controls& controls,
                uint32_t seed, int utcMinutes) {
  const bool enabled = enabled_;
  *this = Atc{};
  enabled_ = enabled;
  depRunway_ = depRunway;
  runway_ = arrRunway;
  const int depAirport = world.runways[static_cast<size_t>(depRunway)].airport;
  const int arrAirport = world.runways[static_cast<size_t>(arrRunway)].airport;
  const Airport& dep = world.airports[static_cast<size_t>(depAirport)];
  const Airport& arr = world.airports[static_cast<size_t>(arrAirport)];
  depStation_ = towerOf(dep);
  clearanceStation_ = kStations[depStation_].traffic ? kRadar : depStation_;
  arrStation_ = towerOf(arr);
  groundStation_ = groundOf(arr);
  const bool onGround = scenario == A320_SCENARIO_RUNWAY || scenario == A320_SCENARIO_COLD_DARK;
  // The ATIS the crew listens to: the departure's before takeoff, else the arrival's.
  atisStation_ = onGround ? atisOf(dep) : atisOf(arr);
  const double routeM = std::hypot(world.airportNorthM[static_cast<size_t>(arrAirport)] - world.airportNorthM[static_cast<size_t>(depAirport)],
                                   world.airportEastM[static_cast<size_t>(arrAirport)] - world.airportEastM[static_cast<size_t>(depAirport)]);
  longRoute_ = onGround && routeM > kLongRouteNm * kNmToM;
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

  if (onGround) {
    phase_ = A320_ATC_PHASE_CLEARANCE;
    // No ATIS: the AFIS gives runway, wind and QNH with the clearance.
    atisHeard_ = atisStation_ < 0;
    controls.com1ActiveKhz = kStations[clearanceStation_].khz;
    controls.com1StandbyKhz = kStations[atisStation_ >= 0 ? atisStation_ : (clearanceStation_ == kRadar ? depStation_ : kRadar)].khz;
    controls.xpdrCode = 2000;
    controls.xpdrMode = A320_XPDR_STBY;
  } else if (scenario == A320_SCENARIO_APPROACH) {
    // With Radar: near in, on a vector to the localizer and the approach clearance comes next;
    // further out, vectors and a descent first. The cleared level is the one found at the start.
    phase_ = A320_ATC_PHASE_RADAR;
    ifrCleared_ = takeoffCleared_ = radarContact_ = checkedInRadar_ = atisHeard_ = true;
    approachScenario_ = true;
    clearedAltFt_ = 0;
    controls.com1ActiveKhz = kStations[kRadar].khz;
    controls.com1StandbyKhz = kStations[arrStation_].khz;
    controls.xpdrCode = squawk_;
    controls.xpdrMode = A320_XPDR_AUTO;
  } else {
    // On final, just handed over to Tower (or the AFIS): check in.
    phase_ = A320_ATC_PHASE_TOWER;
    ifrCleared_ = takeoffCleared_ = radarContact_ = checkedInRadar_ = approachCleared_ = atisHeard_ = true;
    clearedAltFt_ = kApproachAltFt;
    controls.com1ActiveKhz = kStations[arrStation_].khz;
    controls.com1StandbyKhz = kStations[groundStation_ >= 0 ? groundStation_ : kRadar].khz;
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
  const double current = ctx.state.headingTrueDeg - ctx.state.magneticVariationDeg;
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
  if (approachScenario_ && clearedAltFt_ == 0)
    clearedAltFt_ = std::max(kApproachAltFt, static_cast<int>(std::lround(s.altitudeFt / 1000.0)) * 1000);

  // ATIS: a loop while tuned, starting a moment after tuning in.
  if (active != lastActiveKhz_) {
    lastActiveKhz_ = active;
    nextAtisAt_ = t + 1.0;
  }
  if (atisStation_ >= 0 && active == kStations[atisStation_].khz) {
    atisTunedS_ += dt;
    if (atisTunedS_ > 15.0) atisHeard_ = true;
    if (t >= nextAtisAt_) {
      Words w;
      const std::string rwy = ctx.world.runways[static_cast<size_t>(atisRunway())].ident;
      const std::string letter(1, atisLetter_);
      char time[8];
      std::snprintf(time, sizeof(time), "%04d", atisTime_);
      const std::string atisName = kStations[atisStation_].spoken;
      w.add("This is " + atisName + ", information " + letter + ", time " + time + ".",
            "This is " + atisName + ", information " + spell(letter) + ", time " + spell(time) + ".");
      w.add("Runway in use " + rwy + ". Expect ILS approach.", "Runway in use " + spell(rwy) + ". Expect I L S approach.");
      w.add("Transition level 60.", "Transition level " + spell("60") + ".");
      w.add("Wind calm. Visibility 10 kilometres or more, no significant cloud.",
            "Wind calm. Visibility one zero kilometres or more, no significant cloud.");
      w.add("Temperature 15, dew point 9. QNH 1013.", "Temperature one five, dew point niner. Q N H one zero one three.");
      w.add("Acknowledge information " + letter + " on first contact.",
            "Acknowledge information " + spell(letter) + " on first contact.");
      emit(A320_ATC_SPEAKER_ATIS, kStations[atisStation_].name, kStations[atisStation_].khz, w, ctx);
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
    if (kStations[depStation_].traffic)
      hint_ = std::string("No ATS here: announce your departure on ") + kStations[depStation_].spoken + " " +
              freqText(kStations[depStation_].khz) + " before taking off.";
    else if (kStations[depStation_].afis)
      hint_ = std::string("ATC: report \"ready for departure\" to ") + kStations[depStation_].spoken +
              " and wait for \"runway free\" before taking off.";
    else
      hint_ = "ATC: you are taking off without a takeoff clearance. Ask Tower: \"ready for departure\", and read back "
              "\"cleared for takeoff\" first.";
  }
  if (!s.onGround && takeoffCleared_ && !landingCleared_ && s.radioAltFt < 500.0 && s.verticalSpeedFpm < -200.0 &&
      (phase_ == A320_ATC_PHASE_APPROACH || phase_ == A320_ATC_PHASE_TOWER) && !noLandingHinted_) {
    noLandingHinted_ = true;
    if (kStations[arrStation_].traffic)
      hint_ = std::string("No ATS here: announce your final on ") + kStations[arrStation_].spoken + " " +
              freqText(kStations[arrStation_].khz) + ", and look for other traffic.";
    else if (kStations[arrStation_].afis)
      hint_ = std::string("ATC: no runway report from ") + kStations[arrStation_].spoken +
              " yet. Below 500 ft without knowing the runway is free, go around (TOGA). Call in time.";
    else
      hint_ = "ATC: no landing clearance yet. Below 500 ft without one, go around (TOGA). Contact Tower in time.";
  }

  switch (phase_) {
    case A320_ATC_PHASE_DEPARTURE:
      // After the thrust reduction, once the crew has the gear, autopilot and thrust done.
      if (!s.onGround && s.radioAltFt > 1600.0 && s.verticalSpeedFpm > 0.0 && kStations[depStation_].traffic) {
        // Nobody to hand over: the crew calls Radar itself.
        phase_ = A320_ATC_PHASE_RADAR;
        hint_ = "Airborne: contact Tallinn Radar " + freqText(kStations[kRadar].khz) + " and check in.";
      } else if (!s.onGround && s.radioAltFt > 1600.0 && s.verticalSpeedFpm > 0.0) {
        Instruction in;
        in.kind = Kind::ContactRadar;
        in.station = depStation_;
        in.freqKhz = kStations[kRadar].khz;
        in.body.add("contact Tallinn Radar " + freqText(in.freqKhz) + ", goodbye",
                    "contact Tallinn Radar " + freqSpeech(in.freqKhz) + ", goodbye");
        const Words me = cs(ctx);
        Reply ok, wrongFreq, roger;
        ok.words.add("Tallinn Radar " + freqText(in.freqKhz), "Tallinn Radar " + freqSpeech(in.freqKhz));
        wrongFreq.words.add("Tallinn Radar " + freqText(confusable(in.freqKhz)), "Tallinn Radar " + freqSpeech(confusable(in.freqKhz)));
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
      const Runway& r = ctx.world.runways[static_cast<size_t>(runway_)];
      const RunwayPoint p = RunwayAxes(ctx.frame, r).fromEnu({s.eastM, s.northM, 0.0});
      const bool established = s.latMode == A320_LAT_LOC || s.latMode == A320_LAT_LOC_STAR;
      if ((established && p.x > -15.0 * kNmToM) || p.x > -8.0 * kNmToM) {
        const Station& arr = kStations[arrStation_];
        const std::string spoken = arr.spoken;
        // "Tower 135.905" for a Tower; an AFIS keeps its full name.
        const std::string shortName = arr.afis ? spoken : "Tower";
        Instruction in;
        in.kind = Kind::ContactTower;
        in.station = kRadar;
        in.freqKhz = arr.khz;
        const Words me = cs(ctx);
        if (arr.traffic) {
          // No ATS to hand over to: Radar lets the crew go.
          in.body.add("radar service terminated, frequency change approved");
          Reply ok;
          ok.words.add("Frequency change approved").add(", " + me.text, ", " + me.speech);
          in.replies = {ok};
          phase_ = A320_ATC_PHASE_TOWER;
          issue(in, ctx, 1.0);
          hint_ = std::string("No ATS at the destination: tune ") + spoken + " " + freqText(arr.khz) +
                  " and announce your final yourself.";
          monitor(ctx);
          break;
        }
        in.body.add("contact " + spoken + " " + freqText(in.freqKhz), "contact " + spoken + " " + freqSpeech(in.freqKhz));
        Reply ok, wrongFreq;
        ok.words.add(shortName + " " + freqText(in.freqKhz), shortName + " " + freqSpeech(in.freqKhz));
        wrongFreq.words.add(shortName + " " + freqText(confusable(in.freqKhz)), shortName + " " + freqSpeech(confusable(in.freqKhz)));
        wrongFreq.why = "Wrong frequency: " + spoken + " is " + freqText(in.freqKhz) + ".";
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
      if (kStations[arrStation_].traffic) {
        phase_ = A320_ATC_PHASE_DONE;  // nobody to talk to
        break;
      }
      Instruction in;
      in.kind = Kind::Vacate;
      in.station = arrStation_;
      const Words me = cs(ctx);
      if (groundStation_ >= 0) {
        const Station& g = kStations[groundStation_];
        const std::string spoken = g.spoken;
        in.freqKhz = g.khz;
        in.body.add("vacate the runway when able, contact " + spoken + " " + freqText(in.freqKhz) + ", goodbye",
                    "vacate the runway when able, contact " + spoken + " " + freqSpeech(in.freqKhz) + ", goodbye");
        Reply ok, roger;
        ok.words.add("Vacate when able, Handling " + freqText(in.freqKhz), "vacate when able, Handling " + freqSpeech(in.freqKhz));
        roger.words.add("Roger");
        roger.why = "Read back the frequency change: Handling " + freqText(in.freqKhz) + ".";
        for (Reply* r : {&ok, &roger}) r->words.add(", " + me.text, ", " + me.speech);
        in.replies = {ok, roger};
      } else {
        // No ground station: taxi to the apron on the same frequency.
        in.body.add("vacate the runway when able, taxi to the apron, goodbye");
        Reply ok;
        ok.words.add("Vacate when able, taxi to the apron").add(", " + me.text, ", " + me.speech);
        in.replies = {ok};
      }
      phase_ = A320_ATC_PHASE_DONE;
      issue(in, ctx, 2.0);
      break;
    }
    default:
      break;
  }
}

// Radar vectors to the ILS. From ahead of the runway: downwind to a point 17 NM out and 5 NM to
// the side, a base turn, then a 30 degree intercept with the approach clearance, so the localizer
// is captured about 12 NM out and the glideslope from below at 3000 ft. From behind (another
// airport, or far out on the approach side): to a point 20 NM out and 3 NM to the side, then the
// intercept. On a route to another airport, a climb to the cruise level first; the descent to
// 3000 ft comes in time for a 3:1 profile to the turn point.
void Atc::vectors(const AtcContext& ctx) {
  if (approachCleared_ || awaiting_) return;
  const A320State& s = ctx.state;
  const Runway& r = ctx.world.runways[static_cast<size_t>(runway_)];
  const RunwayAxes axes(ctx.frame, r);
  const RunwayPoint p = axes.fromEnu({s.eastM, s.northM, 0.0});
  const double courseMag = r.ils.courseMagDeg > 0.0 ? r.ils.courseMagDeg : r.trueCourseDeg - s.magneticVariationDeg;
  const double var = s.magneticVariationDeg;
  const double hdgMag = s.headingTrueDeg - var;

  bool radarContactCall = true;
  if (approachScenario_) {
    approachScenario_ = false;
    radarContactCall = false;
    // Near in and already on an intercept heading: clear the approach straight away.
    if (p.x > -22.5 * kNmToM) {
      clearApproach(ctx, roundHeading(hdgMag), true);
      return;
    }
  }

  // The turn point, chosen once at radar contact on the side of the final course the aircraft is on.
  if (vectorStage_ == 0) {
    side_ = p.y > 0.0 ? 1 : -1;
    fromAhead_ = p.x > -17.0 * kNmToM;
  }
  const double tx = (fromAhead_ ? -17.0 : -20.0) * kNmToM, ty = side_ * (fromAhead_ ? 5.0 : 3.0) * kNmToM;
  const Enu target = axes.toEnu({tx, ty, 0.0});
  const double brgTrue = std::atan2(target.e - s.eastM, target.n - s.northM) * kRadToDeg;
  const double dist = std::hypot(tx - p.x, ty - p.y);

  if (vectorStage_ == 0) {
    vectorStage_ = 1;
    Instruction in = headingInstruction(roundHeading(brgTrue - var), ctx);
    if (radarContactCall)
      issue(in, ctx, 2.0, "Tallinn Radar, radar contact", "Tallinn Radar, radar contact");
    else
      issue(in, ctx, 2.0, "for sequencing", "for sequencing");
    return;
  }
  if (vectorStage_ == 1) {
    // Top of descent: a 3:1 profile to the turn point plus a few miles to slow down.
    const double descentNm = std::max(12.0, (s.altitudeFt - kApproachAltFt) / 300.0 + 6.0);
    if (!descentIssued_ && clearedAltFt_ > kApproachAltFt && dist < descentNm * kNmToM) {
      descentIssued_ = true;
      issueAltitude(kApproachAltFt, ctx);
      return;
    }
    if (longRoute_ && !cruiseIssued_ && !descentIssued_ && dist > 40.0 * kNmToM) {
      cruiseIssued_ = true;
      issueAltitude(kCruiseAltFt, ctx);
      return;
    }
    if (!fromAhead_ && dist < 2.5 * kNmToM) {
      clearApproach(ctx, roundHeading(courseMag - side_ * 30.0), false);
      return;
    }
    // Base turn abeam the point, or when there.
    if (fromAhead_ && (dist < 2.0 * kNmToM || p.x < -17.0 * kNmToM)) {
      vectorStage_ = 2;
      issue(headingInstruction(roundHeading(courseMag - side_ * 90.0), ctx), ctx, 1.0);
      return;
    }
    // Re-vector once the aircraft flies the heading but the point has moved off it.
    const int wanted = roundHeading(brgTrue - var);
    if (headingMag_ > 0 && dist > 5.0 * kNmToM && std::fabs(wrap180(wanted - headingMag_)) >= (dist > 30.0 * kNmToM ? 15.0 : 30.0) &&
        std::fabs(wrap180(hdgMag - headingMag_)) < 15.0 && now(ctx) - headingSetAt_ > 45.0) {
      issue(headingInstruction(wanted, ctx), ctx, 1.0);
    }
    return;
  }
  if (vectorStage_ == 2 && (std::fabs(p.y) < 3.2 * kNmToM || p.y * side_ < 0.0))
    clearApproach(ctx, roundHeading(courseMag - side_ * 30.0), false);
}

// The approach clearance: an intercept heading (or the present one) with "cleared ILS approach".
void Atc::clearApproach(const AtcContext& ctx, int headingMag, bool presentHeading) {
  const A320State& s = ctx.state;
  const Runway& r = ctx.world.runways[static_cast<size_t>(runway_)];
  const std::string rwy = r.ident, other = otherIdent(ctx.world, runway_);
  const std::string appr = approachText(r), apprSpeech = approachSpeech(r);
  const Words me = cs(ctx);
  Instruction in;
  in.kind = Kind::Approach;
  in.station = kRadar;
  in.heading = headingMag;
  if (presentHeading) {
    in.body.add("continue present heading, cleared " + appr + " runway " + rwy,
                "continue present heading, cleared " + apprSpeech + " runway " + spell(rwy));
    Reply ok, wrongRwy, roger;
    ok.words.add("Present heading, cleared " + appr + " runway " + rwy, "present heading, cleared " + apprSpeech + " runway " + spell(rwy));
    wrongRwy.words.add("Present heading, cleared " + appr + " runway " + other,
                       "present heading, cleared " + apprSpeech + " runway " + spell(other));
    wrongRwy.why = "Wrong runway: the clearance is for runway " + rwy + ".";
    roger.words.add("Roger");
    roger.why = "An approach clearance is read back in full: heading and \"cleared " + appr + " runway " + rwy + "\".";
    for (Reply* rr : {&ok, &wrongRwy, &roger}) rr->words.add(", " + me.text, ", " + me.speech);
    in.replies = {ok, wrongRwy, roger};
    issue(in, ctx, 3.0);
    return;
  }
  const bool slow = s.iasKt > 195.0;
  const std::string speedText = slow ? ", reduce speed 180 knots" : "";
  const std::string speedSpeech = slow ? ", reduce speed one eight zero knots" : "";
  const double current = s.headingTrueDeg - s.magneticVariationDeg;
  const bool left = wrap180(headingMag - current) < 0.0;
  const std::string dir = left ? "left" : "right";
  in.speedKt = slow ? 180 : 0;
  in.body.add("turn " + dir + " heading " + headingText(headingMag) + ", cleared " + appr + " runway " + rwy + speedText,
              "turn " + dir + " heading " + headingSpeech(headingMag) + ", cleared " + apprSpeech + " runway " + spell(rwy) + speedSpeech);
  const std::string turn = left ? "Left" : "Right", wrongTurn = left ? "Right" : "Left";
  auto readback = [&](const std::string& t, const std::string& rw) {
    Words w;
    w.add(t + " heading " + headingText(headingMag) + ", cleared " + appr + " runway " + rw + speedText,
          t + " heading " + headingSpeech(headingMag) + ", cleared " + apprSpeech + " runway " + spell(rw) + speedSpeech);
    return w.add(", " + me.text, ", " + me.speech);
  };
  Reply ok, wrongDir, wrongRwy;
  ok.words = readback(turn, rwy);
  wrongDir.words = readback(wrongTurn, rwy);
  wrongDir.why = "The turn direction is part of the instruction: ATC said turn " + dir + ".";
  wrongRwy.words = readback(turn, other);
  wrongRwy.why = "Wrong runway: the approach clearance is for runway " + rwy + ".";
  in.replies = {ok, wrongDir, wrongRwy};
  if (other == rwy) in.replies.pop_back();
  issue(in, ctx, 1.0);
}

void Atc::issueAltitude(int altFt, const AtcContext& ctx) {
  const bool climb = altFt > ctx.state.altitudeFt;
  const std::string verb = climb ? "climb " : "descend ";
  const std::string Verb = climb ? "Climb " : "Descend ";
  // Below the transition altitude the QNH comes with the altitude.
  const bool qnh = altFt <= kTransitionAltFt;
  const std::string qText = qnh ? ", QNH 1013" : "", qSpeech = qnh ? ", Q N H one zero one three" : "";
  Instruction in;
  in.kind = Kind::Altitude;
  in.station = kRadar;
  in.altFt = altFt;
  in.body.add(verb + altText(altFt) + qText, verb + altSpeech(altFt) + qSpeech);
  const Words me = cs(ctx);
  Reply ok, wrongAlt, wrongRef;
  ok.words.add(Verb + altText(altFt) + qText, verb + altSpeech(altFt) + qSpeech);
  const int off = climb ? altFt + 10000 : altFt - 1000;
  wrongAlt.words.add(Verb + altText(off) + qText, verb + altSpeech(off) + qSpeech);
  wrongAlt.why = "Wrong level: ATC said " + altText(altFt) + ". Flying another cleared level is a level bust.";
  if (qnh) {
    wrongRef.words.add(Verb + altText(altFt) + ", QNH 1003", verb + altSpeech(altFt) + ", Q N H one zero zero three");
    wrongRef.why = "Wrong QNH: 1013. A 10 hPa error puts the aircraft 280 ft off its altitude.";
  } else {
    wrongRef.words.add(Verb + fmt("altitude %d feet", altFt), verb + "altitude " + passingSpeech(altFt));
    wrongRef.why = "Above the transition altitude (5000 ft) levels are flight levels, on the standard setting 1013: "
                   "read back \"" + altText(altFt) + "\".";
  }
  for (Reply* rp : {&ok, &wrongAlt, &wrongRef}) rp->words.add(", " + me.text, ", " + me.speech);
  in.replies = {ok, wrongAlt, wrongRef};
  issue(in, ctx, 1.0);
  if (climb)
    hint_ = "Set " + std::to_string(altFt) + " in the FCU ALT window and pull the knob to climb. Passing 5000 ft, "
            "set the baro to STD.";
}

// Queries a heading or altitude that isn't being flown, once per instruction.
void Atc::monitor(const AtcContext& ctx) {
  if (awaiting_ || !pending_.empty()) return;
  const A320State& s = ctx.state;
  const double t = now(ctx);
  // Time to set the FCU and turn at about 3 degrees per second, plus a margin.
  const double allowed = 25.0 + headingTurnDeg_ / 2.0;
  if (headingMag_ > 0 && !headingQueried_ && !approachCleared_ && t - headingSetAt_ > allowed) {
    const double hdgMag = s.headingTrueDeg - ctx.state.magneticVariationDeg;
    if (std::fabs(wrap180(hdgMag - headingMag_)) > 25.0) {
      headingQueried_ = true;
      Instruction in = headingInstruction(headingMag_, ctx);
      issue(in, ctx, 0.5, "check heading", "check heading");
      hint_ = "ATC expects heading " + headingText(headingMag_) + ". Set it on the FCU (HDG) and pull the knob [Key: U].";
      return;
    }
  }
  // Cleared for the approach, the glideslope takes the aircraft below the cleared altitude.
  if (clearedAltFt_ > 0 && !altQueried_ && !s.onGround && !approachCleared_) {
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
  const std::string depRwy = ctx.world.runways[static_cast<size_t>(depRunway_)].ident;
  const Runway& arr = ctx.world.runways[static_cast<size_t>(runway_)];
  const std::string rwy = arr.ident;
  const std::string depName = kStations[depStation_].spoken, arrName = kStations[arrStation_].spoken;
  const std::string destCity = ctx.world.airportOf(runway_).city;
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
  if (phase_ == A320_ATC_PHASE_CLEARANCE && s.onGround && on(clearanceStation_)) {
    Option o;
    const std::string callName = kStations[clearanceStation_].spoken;
    // Calling Radar from another airport: say which one.
    const std::string at = clearanceStation_ == kRadar ? " at " + ctx.world.airportOf(depRunway_).city : "";
    o.words.add(callName + ",", callName + ",").add(me.text + ", A320" + at + " runway " + depRwy,
                                                    me.speech + ", A three twenty" + at + " runway " + spell(depRwy));
    if (atisHeard_ && atisStation_ >= 0) o.words.add("with information " + letter, "with information " + spell(letter));
    o.words.add(", request IFR clearance to " + destCity, ", request I F R clearance to " + destCity);
    o.request = Request::Clearance;
    out.push_back(o);
  }
  if (phase_ == A320_ATC_PHASE_DEPARTURE && s.onGround && on(depStation_) && !takeoffCleared_) {
    Option o;
    if (kStations[depStation_].traffic)
      o.words.add(depName + ", " + me.text + ", departing runway " + depRwy, depName + ", " + me.speech + ", departing runway " + spell(depRwy));
    else
      o.words.add(me.text + ", ready for departure runway " + depRwy, me.speech + ", ready for departure runway " + spell(depRwy));
    o.request = Request::Ready;
    out.push_back(o);
  }
  if (phase_ == A320_ATC_PHASE_RADAR && !checkedInRadar_ && on(kRadar)) {
    const int passing = static_cast<int>(std::lround(s.altitudeFt / 100.0)) * 100;
    const int hdg = roundHeading(s.headingTrueDeg - ctx.state.magneticVariationDeg);
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
  if (phase_ == A320_ATC_PHASE_TOWER && !checkedInTower_ && on(arrStation_)) {
    Option o;
    if (arr.ils.ident.empty() || kStations[arrStation_].traffic)
      o.words.add(arrName + ", " + me.text + ", on final runway " + rwy, arrName + ", " + me.speech + ", on final runway " + spell(rwy));
    else
      o.words.add(arrName + ", " + me.text + ", established ILS runway " + rwy,
                  arrName + ", " + me.speech + ", established I L S runway " + spell(rwy));
    o.request = Request::CheckInTower;
    out.push_back(o);
  }
  if (phase_ == A320_ATC_PHASE_RADAR && radarContact_ && !approachCleared_ && on(kRadar)) {
    // Another ILS runway at the destination.
    for (size_t i = 0; i < ctx.world.runways.size() && out.size() + 1 < A320_ATC_MAX_OPTIONS; ++i) {
      const Runway& o2 = ctx.world.runways[i];
      if (static_cast<int>(i) == runway_ || o2.airport != arr.airport || o2.ils.ident.empty()) continue;
      const std::string other = o2.ident;
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
      if (kStations[depStation_].traffic) {
        const std::string trafficName = kStations[depStation_].spoken;
        w.add(", readback correct, departure at your discretion, announce it on " + trafficName + ", call me airborne");
      } else {
        w.add(", readback correct, report ready for departure");
      }
      say(in.station, w, 1.2, ctx);
      break;
    }
    case Kind::Takeoff: takeoffCleared_ = true; break;
    case Kind::ContactRadar: break;
    case Kind::Squawk: break;
    case Kind::Heading: {
      headingMag_ = in.heading;
      headingSetAt_ = t;
      headingQueried_ = false;
      const double hdgMag = ctx.state.headingTrueDeg - ctx.state.magneticVariationDeg;
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
  const Runway& arr = ctx.world.runways[static_cast<size_t>(runway_)];
  const std::string rwy = arr.ident;
  const std::string depRwy = ctx.world.runways[static_cast<size_t>(depRunway_)].ident;
  const std::string depOther = otherIdent(ctx.world, depRunway_);
  const Station& dep = kStations[depStation_];
  const Station& arrSt = kStations[arrStation_];
  const Words me = cs(ctx);
  const double delay = 1.5;
  switch (o.request) {
    case Request::Clearance: {
      const std::string city = ctx.world.airportOf(runway_).city;
      const std::string route = "cleared to " + city + " via radar vectors";
      // An AFIS relays the clearance from the unit that issues it.
      const std::string by = dep.afis ? "Tallinn Radar clears you to " + city + " via radar vectors" : route;
      Instruction in;
      in.kind = Kind::Clearance;
      in.station = clearanceStation_;
      in.altFt = kInitialAltFt;
      const std::string code = fmt("%04d", squawk_);
      in.body.add(by + ", after departure runway heading, climb " + altText(kInitialAltFt) + ", squawk " + code,
                  by + ", after departure runway heading, climb " + altSpeech(kInitialAltFt) + ", squawk " + spell(code));
      const std::string Route = "Cleared to " + city + " via radar vectors";
      Reply ok, wrongAlt, wrongCode;
      ok.words.add(Route + ", runway heading, climb " + altText(kInitialAltFt) + ", squawk " + code,
                   route + ", runway heading, climb " + altSpeech(kInitialAltFt) + ", squawk " + spell(code));
      wrongAlt.words.add(Route + ", runway heading, climb " + altText(5000) + ", squawk " + code,
                         route + ", runway heading, climb " + altSpeech(5000) + ", squawk " + spell(code));
      wrongAlt.why = "Wrong altitude: the clearance limit is 4000 ft. Set 4000 in the FCU ALT window.";
      std::string swapped = code;
      std::swap(swapped[1], swapped[2]);
      wrongCode.words.add(Route + ", runway heading, climb " + altText(kInitialAltFt) + ", squawk " + swapped,
                          route + ", runway heading, climb " + altSpeech(kInitialAltFt) + ", squawk " + spell(swapped));
      wrongCode.why = "Wrong squawk: the code is " + code + ".";
      for (Reply* r : {&ok, &wrongAlt, &wrongCode}) r->words.add(", " + me.text, ", " + me.speech);
      in.replies = {ok, wrongAlt, wrongCode};
      if (clearanceStation_ == kRadar) {
        issue(in, ctx, delay, "QNH 1013", "Q N H one zero one three");  // no ATS there to give the runway
      } else if (atisStation_ < 0) {
        issue(in, ctx, delay, "runway in use " + depRwy + ", wind calm, QNH 1013",
              "runway in use " + spell(depRwy) + ", wind calm, Q N H one zero one three");
      } else if (!atisHeard_) {
        const std::string letter(1, atisLetter_);
        issue(in, ctx, delay, "information " + letter + " is current, QNH 1013",
              "information " + spell(letter) + " is current, Q N H one zero one three");
      } else {
        issue(in, ctx, delay);
      }
      if (longRoute_)
        hint_ = "A flight to " + city + ": after takeoff Tallinn Radar vectors you there and climbs you to the cruise "
                "level. Insert the arrival on the MCDU (F-PLN > ARRIVAL) so its ILS is tuned.";
      break;
    }
    case Request::Ready: {
      if (dep.traffic) {
        // A broadcast to whoever is around: nobody answers.
        takeoffCleared_ = ifrCleared_;
        if (!ifrCleared_)
          hint_ = "IFR flights need a clearance before departure: get it from Tallinn Radar " +
                  freqText(kStations[kRadar].khz) + " first.";
        break;
      }
      if (!ifrCleared_) {
        Words w = me;
        w.add(", negative, you have no IFR clearance yet. Request your clearance first.");
        say(depStation_, w, delay, ctx);
        hint_ = std::string("IFR flights need a clearance before departure: request it from ") + dep.spoken + " first.";
        break;
      }
      Instruction in;
      in.kind = Kind::Takeoff;
      in.station = depStation_;
      Reply ok, wrongRwy, roger;
      if (dep.afis) {
        in.body.add("runway " + depRwy + " free, wind calm, take off at your discretion",
                    "runway " + spell(depRwy) + " free, wind calm, take off at your discretion");
        ok.words.add("Taking off runway " + depRwy, "taking off runway " + spell(depRwy));
        wrongRwy.words.add("Cleared for takeoff runway " + depRwy, "cleared for takeoff runway " + spell(depRwy));
        wrongRwy.why = std::string(dep.spoken) + " is an AFIS: it reports the runway free but gives no clearances. "
                       "The decision to take off is yours: read back \"taking off runway " + depRwy + "\".";
        roger.words.add("Roger");
        roger.why = "Say what you will do, with the runway: \"taking off runway " + depRwy + "\".";
      } else {
        in.body.add("wind calm, runway " + depRwy + ", cleared for takeoff", "wind calm, runway " + spell(depRwy) + ", cleared for takeoff");
        ok.words.add("Cleared for takeoff runway " + depRwy, "cleared for takeoff runway " + spell(depRwy));
        wrongRwy.words.add("Cleared for takeoff runway " + depOther, "cleared for takeoff runway " + spell(depOther));
        wrongRwy.why = "You are on runway " + depRwy + ". A wrong runway in a takeoff readback is a classic error ATC must "
                       "catch.";
        roger.words.add("Roger, rolling");
        roger.why = "A takeoff clearance is read back with the runway: \"cleared for takeoff runway " + depRwy + "\".";
      }
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
      if (arrSt.traffic) {
        landingCleared_ = true;  // announced; the decision to land is the crew's
        break;
      }
      Instruction in;
      in.kind = Kind::Landing;
      in.station = arrStation_;
      Reply ok, wrong;
      if (arrSt.afis) {
        in.body.add("runway " + rwy + " free, wind calm, QNH 1013", "runway " + spell(rwy) + " free, wind calm, Q N H one zero one three");
        ok.words.add("Landing runway " + rwy + ", QNH 1013", "landing runway " + spell(rwy) + ", Q N H one zero one three");
        wrong.words.add("Cleared to land runway " + rwy, "cleared to land runway " + spell(rwy));
        wrong.why = std::string(arrSt.spoken) + " is an AFIS: \"runway free\" is information, not a landing clearance. "
                    "Read back what you will do and the QNH: \"landing runway " + rwy + ", QNH 1013\".";
      } else {
        in.body.add("wind calm, runway " + rwy + ", cleared to land", "wind calm, runway " + spell(rwy) + ", cleared to land");
        ok.words.add("Cleared to land runway " + rwy, "cleared to land runway " + spell(rwy));
        wrong.words.add("Roger");
        wrong.why = "Landing clearances are read back with the runway: \"cleared to land runway " + rwy + "\".";
      }
      for (Reply* r : {&ok, &wrong}) r->words.add(", " + me.text, ", " + me.speech);
      in.replies = {ok, wrong};
      const std::string spoken = arrSt.spoken;
      issue(in, ctx, delay, spoken, spoken);
      break;
    }
    case Request::OtherRunway: {
      runway_ = o.runway;
      vectorStage_ = 0;  // new vectors for the other runway
      const std::string other = ctx.world.runways[static_cast<size_t>(runway_)].ident;
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
  s.atcRunwayIndex = atisRunway();
  s.atcMessageSeq = seq_;
}

std::string Atc::takeHint() {
  std::string h;
  h.swap(hint_);
  return h;
}

}  // namespace a320
