#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "a320/Airport.h"
#include "a320/Fms.h"
#include "a320/Geo.h"
#include "a320/a320_api.h"

namespace a320 {

struct AtcContext {
  const World& world;
  const LocalFrame& frame;
  const A320State& state;
  const A320Controls& controls;
  const Fms& fms;
};

// Air traffic control for an IFR flight between the sim's airports, with ICAO phraseology: ATIS,
// IFR clearance and takeoff from Tower (or the AFIS, which relays and informs but does not clear),
// Tallinn Radar vectors to the ILS, then the arrival's Tower or AFIS. The crew answers from a
// menu: readbacks (with wrong ones to learn from) and requests. Instructions not read back are
// repeated; headings and altitudes not flown are queried.
class Atc {
 public:
  // A new flight for the scenario; sets COM 1 and the transponder as the crew would have them.
  // Ground starts depart from depRunway; every flight lands on arrRunway (World::runways).
  void reset(const World& world, A320Scenario scenario, int depRunway, int arrRunway, A320Controls& controls,
             uint32_t seed, int utcMinutes);
  void setEnabled(bool on) { enabled_ = on; }
  bool enabled() const { return enabled_; }

  void update(const AtcContext& ctx);
  void choose(int option, const AtcContext& ctx);
  std::vector<std::string> options(const AtcContext& ctx) const;

  bool message(uint32_t seq, A320AtcMessage& out) const;
  uint32_t lastSeq() const { return seq_; }
  void fillState(A320State& s) const;
  void setWeather(int weather) { weather_ = weather; }
  std::string callsign(const Fms& fms) const;
  std::string stationName(int khz) const;
  // A tutor remark since the last call, or "".
  std::string takeHint();

 private:
  int weather_ = A320_WEATHER_SUNNY;
  enum class Kind {
    None, Clearance, Takeoff, ContactRadar, Squawk, Heading, Altitude, Approach, ContactTower, Landing, Vacate
  };
  struct Words {
    std::string text, speech;
    Words& add(const std::string& t) { return add(t, t); }
    Words& add(const std::string& t, const std::string& s);
  };
  struct Reply {
    Words words;
    std::string why;  // empty for the correct readback
  };
  struct Instruction {
    Kind kind = Kind::None;
    int station = 0;
    Words body;              // without the callsign, for repeats and corrections
    std::vector<Reply> replies;
    int heading = -1, altFt = 0, speedKt = 0, freqKhz = 0;
    double issuedAt = 0.0;
    int repeats = 0;
  };
  enum class Request { Clearance, Ready, CheckInRadar, CheckInTower, OtherRunway, SayAgain };
  struct Option {
    Words words;
    int reply = -1;  // index into the instruction's replies, or -1 for a request
    Request request = Request::Clearance;
    int runway = -1;
  };
  struct Pending {
    double at;
    int station;
    Words words;
  };

  std::vector<Option> buildOptions(const AtcContext& ctx) const;
  void emit(int speaker, const std::string& station, int khz, const Words& w, const AtcContext& ctx);
  void say(int station, const Words& w, double delayS, const AtcContext& ctx);
  void issue(Instruction in, const AtcContext& ctx, double delayS, const std::string& prefix = "",
             const std::string& prefixSpeech = "");
  void readbackDone(const Instruction& in, const AtcContext& ctx);
  void request(const Option& o, const AtcContext& ctx);
  void vectors(const AtcContext& ctx);
  void monitor(const AtcContext& ctx);
  Instruction headingInstruction(int headingMag, const AtcContext& ctx, const std::string& extra = "") const;
  void clearApproach(const AtcContext& ctx, int headingMag, bool presentHeading);
  void issueAltitude(int altFt, const AtcContext& ctx);
  int atisRunway() const;
  Words cs(const AtcContext& ctx) const;
  double now(const AtcContext& ctx) const { return ctx.state.simTimeS; }

  bool enabled_ = true;
  int phase_ = A320_ATC_PHASE_CLEARANCE;
  int depRunway_ = 0;       // runway in use for departure
  int runway_ = 0;          // arrival runway, the one vectored to
  int depStation_ = 1, arrStation_ = 1, atisStation_ = -1, groundStation_ = -1;
  int clearanceStation_ = 1;  // the departure's Tower or AFIS; Tallinn Radar where there is no ATS
  bool longRoute_ = false;  // to another airport: a climb to the cruise level, a descent later
  int squawk_ = 2000;
  char atisLetter_ = 'A';
  int atisTime_ = 0;
  bool atisHeard_ = false;
  double atisTunedS_ = 0.0, nextAtisAt_ = 0.0;
  int lastActiveKhz_ = 0;
  double lastUpdate_ = -1.0;

  bool ifrCleared_ = false, takeoffCleared_ = false, approachCleared_ = false, landingCleared_ = false;
  bool radarContact_ = false, checkedInRadar_ = false, checkedInTower_ = false, squawkAsked_ = false;
  bool approachScenario_ = false;
  int clearedAltFt_ = 0, headingMag_ = -1, speedKt_ = 0;
  double headingSetAt_ = 0.0, altSetAt_ = 0.0, headingTurnDeg_ = 0.0;
  bool headingQueried_ = false, altQueried_ = false;
  int vectorStage_ = 0;     // 0 none, 1 to the turn point, 2 base
  int side_ = -1;           // -1 = left of the final approach course, +1 = right
  bool fromAhead_ = true;   // turn point is a downwind's end (else a straight-in intercept point)
  bool descentIssued_ = false, cruiseIssued_ = false;
  bool noTakeoffHinted_ = false, noLandingHinted_ = false, notOnFreqHinted_ = false;

  bool awaiting_ = false;
  Instruction instr_;
  Instruction last_;
  int instructionCount_ = 0;
  std::deque<Pending> pending_;
  double busyUntil_ = 0.0;

  std::deque<A320AtcMessage> log_;
  uint32_t seq_ = 0;
  std::string hint_;
};

}  // namespace a320
