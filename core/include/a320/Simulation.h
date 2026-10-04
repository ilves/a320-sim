#pragma once

#include <memory>
#include <string>
#include <vector>

#include "a320/Airport.h"
#include "a320/Atc.h"
#include "a320/Audio.h"
#include "a320/Autopilot.h"
#include "a320/FlyByWire.h"
#include "a320/Fms.h"
#include "a320/Geo.h"
#include "a320/GroundMap.h"
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
  explicit Simulation(World world);
  ~Simulation();
  Simulation(const Simulation&) = delete;
  Simulation& operator=(const Simulation&) = delete;

  // Loads the JSBSim A320 model from jsbsimRoot (aircraft/, engine/).
  bool init(const std::string& jsbsimRoot, std::string* error);

  bool reset(A320Scenario scenario, int runwayIndex);
  // A flight from depRunway to arrRunway: on the ground at the departure, or in the air distanceNm
  // from the arrival (A320_SCENARIO_APPROACH; the finals use their fixed distances).
  bool startFlight(A320Scenario scenario, int depRunway, int arrRunway, double distanceNm,
                   int flightPlan = A320_PLAN_ROUTE);
  // The terrain under the flight model (Content/Terrain); without it the ground is the nearest
  // airport's elevation everywhere.
  bool loadGround(const std::string& terrainDir, std::string* error = nullptr) { return ground_.load(terrainDir, error); }
  // The ground's height above the first airport's field at a point of the flat world.
  double groundAt(double northM, double eastM) const;
  // On an airport's flattened runway area (the runway, 1000 m beyond its ends, 600 m aside).
  bool onAirportGround(double northM, double eastM) const;
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
  const World& world() const { return world_; }
  const LocalFrame& frame() const { return frame_; }
  const LocalFrame& airportFrame(int airport) const { return airportFrames_[static_cast<size_t>(airport)]; }
  // The scenario runway's ILS (where the aircraft was placed); the tuned one is tunedIls().
  const Ils& ils() const { return ilsAll_[static_cast<size_t>(runwayIndex_)]; }
  int activeRunway() const { return runwayIndex_; }
  int tunedIls() const { return tunedIls_; }

  void mcduKey(int key);
  void mcduDisplay(A320McduDisplay& out) const;
  Fms& fms() { return fms_; }
  const Fms& fms() const { return fms_; }

  Atc& atc() { return atc_; }
  const Atc& atc() const { return atc_; }
  std::vector<std::string> atcOptions() const;
  void atcChoose(int option);

 private:
  double trimAirborne();
  void hint(const char* text);
  void hintAboveGlideslope();
  ApInput apInput() const;
  void applyControls();
  void refreshState();
  // The MCDU's data for a new flight, as much as flightPlan (A320FlightPlan) says.
  void loadFlightPlan(int flightPlan, bool onRunway, int depRunway, int arrRunway);
  double prop(const char* name) const;
  void setProp(const char* name, double v);

  World world_;
  LocalFrame frame_;                       // the flat world's
  std::vector<LocalFrame> airportFrames_;  // each airport's own, for its ILS geometry
  int nearestAirport_ = 0;
  double magVar() const { return world_.airports[static_cast<size_t>(nearestAirport_)].magneticVariationDeg; }
  std::unique_ptr<JSBSim::FGFDMExec> fdm_;
  std::vector<Ils> ilsAll_;  // one per runway direction
  int runwayIndex_ = 0;
  int tunedIls_ = -1;
  IlsSignal ilsSignal_;
  Fms fms_;
  Mcdu mcdu_;
  Atc atc_;
  double stepTimeS_ = 0.0;
  void updateAtc();
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

  // A destroyed aircraft (A320Destroyed): the flight model stops and, after a breakup, the two
  // pieces fall on their own: gravity, drag, a tumble, until they reach the ground.
  struct Piece {
    double n = 0.0, e = 0.0, u = 0.0;     // its CG in the flat world (u: height above field)
    double vn = 0.0, ve = 0.0, vu = 0.0;  // m/s
    double hdg = 0.0, pitch = 0.0, bank = 0.0;  // grid heading
    double hdgRate = 0.0, bankRate = 0.0, pitchTarget = 0.0;
    double cgX = 0.0;    // its CG ahead of the aircraft's reference point (m)
    double dragK = 0.0;  // g / terminal speed squared
    bool onGround = false;
  };
  void checkDestroyed();
  void breakUp();
  void crash();
  void updatePieces(double dt);
  void fillDestroyed(A320State& s) const;
  // The cockpit's instruments after a breakup: what the nose section is doing; engines winding down.
  void cockpitFromPiece(double dt);
  int destroyed_ = A320_DESTROYED_NONE;
  uint32_t destroyedSeq_ = 0;
  uint32_t impactSeq_[2] = {0, 0};
  Piece pieces_[2];
  bool impact_ = false;  // this step's ground contact is a crash
  double impactFpm_ = 0.0;
  double groundHeightM_ = 0.0;
  double wreckIasFactor_ = 1.0;  // the flight model's airspeed over the standard atmosphere's, at the breakup
  double groundSetM_ = 1e9;  // what the flight model's ground was last set to

  // Where the scenery is flattened to an airport's level: each runway's length plus 1000 m at
  // both ends, 600 m to the sides (as tools/make_terrain.py).
  struct FlatBox {
    double midN, midE, dirN, dirE, halfLengthM, levelM;
  };
  std::vector<FlatBox> flatBoxes_;
  GroundMap ground_;
};

}  // namespace a320
