#pragma once

#include <memory>
#include <string>
#include <vector>

#include "a320/Airport.h"
#include "a320/Audio.h"
#include "a320/Autopilot.h"
#include "a320/FlyByWire.h"
#include "a320/Fms.h"
#include "a320/Geo.h"
#include "a320/Ils.h"
#include "a320/Mcdu.h"
#include "a320/SimClock.h"
#include "a320/Systems.h"
#include "a320/a320_api.h"

namespace JSBSim {
class FGFDMExec;
}

namespace a320 {

class Simulation {
 public:
  explicit Simulation(Airport airport);
  ~Simulation();
  Simulation(const Simulation&) = delete;
  Simulation& operator=(const Simulation&) = delete;

  // Loads the JSBSim A320 model from jsbsimRoot (aircraft/, engine/).
  bool init(const std::string& jsbsimRoot, std::string* error);

  bool reset(A320Scenario scenario, int runwayIndex);
  void setControls(const A320Controls& c);
  void fcuCommand(A320FcuCommand cmd);
  void setFcuTargets(double spdKt, double hdgMagDeg, double altFt, double vsFpm);
  AudioEngine& audio() { return audio_; }
  void update(double realDtS);
  // One fixed step regardless of pause; used by update() and by tests.
  void step();

  const A320State& state() const { return state_; }
  const A320Controls& controls() const { return controls_; }
  SimClock& clock() { return clock_; }
  const Airport& airport() const { return airport_; }
  const LocalFrame& frame() const { return frame_; }
  // The scenario runway's ILS (where the aircraft was placed); the tuned one is tunedIls().
  const Ils& ils() const { return ilsAll_[static_cast<size_t>(runwayIndex_)]; }
  int activeRunway() const { return runwayIndex_; }
  int tunedIls() const { return tunedIls_; }

  void mcduKey(int key);
  void mcduDisplay(A320McduDisplay& out) const;
  Fms& fms() { return fms_; }

 private:
  double trimAirborne();
  void hint(const char* text);
  void hintAboveGlideslope();
  ApInput apInput() const;
  void applyControls();
  void refreshState();
  double prop(const char* name) const;
  void setProp(const char* name, double v);

  Airport airport_;
  LocalFrame frame_;
  std::unique_ptr<JSBSim::FGFDMExec> fdm_;
  std::vector<Ils> ilsAll_;  // one per runway direction
  int runwayIndex_ = 0;
  int tunedIls_ = -1;
  IlsSignal ilsSignal_;
  Fms fms_;
  Mcdu mcdu_;
  double weightLbs_ = 0.0;
  SimClock clock_;
  FlyByWire fbw_;
  FlapsSystem flaps_;
  Autopilot ap_;
  Apu apu_;
  GroundDecel decel_;
  void updateEngines(bool bleedAvailable);
  AudioEngine audio_;
  double throttle_ = 0.0;
  bool athrActive_ = false;
  Callouts callouts_;
  TakeoffCallouts takeoffCallouts_;
  A320Controls controls_{};
  A320State state_{};
  bool wasOnGround_ = true;
  bool wasAlphaFloor_ = false;
  bool aboveGsHinted_ = false, noGsArmHinted_ = false;
  double airborneS_ = 0.0;
  double lastAirborneS_ = 0.0;
  double speedbrakePos_ = 0.0;
};

}  // namespace a320
