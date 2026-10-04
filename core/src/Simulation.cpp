#include "a320/Simulation.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>

#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "initialization/FGTrim.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGTurbine.h"
#include "simgear/misc/sg_path.hxx"

#include "a320/Units.h"

namespace a320 {
namespace {

constexpr double kLbsToKg = 0.45359237;
// CG height above the runway with the gear compressed at rest, so the radio altimeter
// reads 0 on the ground (measured from the model: tests/test_simulation.cpp).
constexpr double kRadioAltOffsetFt = 8.4;
constexpr double kReverserAngleRad = 2.0944;  // cos = -0.5: half of forward thrust reversed
constexpr double kLineupDistanceM = 60.0;
constexpr double kInterceptDeg = 30.0;
// Takeoff THS setting for the model's default CG (real aircraft: from the load sheet).
constexpr double kTakeoffThsDeg = -2.0;
// THS / elevator pitch effectiveness ratio (CmThs -3.5 vs CmDe -1.5 in A320.xml).
constexpr double kElevatorToThs = 1.5 / 3.5;

}  // namespace

Simulation::Simulation(Airport airport)
    : airport_(std::move(airport)), frame_(airport_.reference) {}

Simulation::~Simulation() = default;

double Simulation::prop(const char* name) const { return fdm_->GetPropertyValue(name); }
void Simulation::setProp(const char* name, double v) { fdm_->SetPropertyValue(name, v); }

bool Simulation::init(const std::string& jsbsimRoot, std::string* error) {
  try {
    JSBSim::FGJSBBase::debug_lvl = 0;
    fdm_ = std::make_unique<JSBSim::FGFDMExec>();
    fdm_->SetRootDir(SGPath::fromUtf8(jsbsimRoot));
    fdm_->SetAircraftPath(SGPath("aircraft"));
    fdm_->SetEnginePath(SGPath("engine"));
    fdm_->SetSystemsPath(SGPath("systems"));
    fdm_->Setdt(clock_.stepS());
    if (!fdm_->LoadModel("A320")) {
      if (error) *error = "JSBSim could not load aircraft/A320 from " + jsbsimRoot;
      fdm_.reset();
      return false;
    }
  } catch (const std::exception& e) {
    if (error) *error = std::string("JSBSim: ") + e.what();
    fdm_.reset();
    return false;
  }
  return reset(A320_SCENARIO_RUNWAY, airport_.find("26") ? 1 : 0);
}

bool Simulation::reset(A320Scenario scenario, int runwayIndex) {
  if (!fdm_ || airport_.runways.empty()) return false;
  runwayIndex_ = clamp(runwayIndex, 0, static_cast<int>(airport_.runways.size()) - 1);
  const Runway& rw = airport_.runways[runwayIndex_];
  ils_ = std::make_unique<Ils>(frame_, rw);

  controls_ = A320Controls{};
  controls_.gearDown = 1;
  flaps_ = FlapsSystem{};
  callouts_.reset();
  takeoffCallouts_.reset();
  // Sequence counters stay monotonic so front ends never mistake a reset for a new event.
  const uint32_t calloutSeq = state_.calloutSeq, touchdownSeq = state_.touchdownSeq, hintSeq = state_.hintSeq;
  state_ = A320State{};
  state_.calloutSeq = calloutSeq;
  state_.touchdownSeq = touchdownSeq;
  state_.hintSeq = hintSeq;

  const bool coldDark = scenario == A320_SCENARIO_COLD_DARK;
  const bool onRunway = scenario == A320_SCENARIO_RUNWAY || coldDark;
  const bool intercept = scenario == A320_SCENARIO_APPROACH;
  apu_.setRunning(false);
  decel_.reset();
  // Engines running: masters on, mode NORM; cold and dark: everything off.
  controls_.engMaster[0] = controls_.engMaster[1] = coldDark ? 0 : 1;
  controls_.engMode = A320_ENG_MODE_NORM;
  controls_.lights = coldDark ? 0
                    : A320_LT_BEACON | A320_LT_STROBE | A320_LT_NAV | A320_LT_LANDING | A320_LT_TAKEOFF;
  controls_.signs = A320_SIGN_NO_SMOKING | (coldDark ? 0 : A320_SIGN_SEATBELTS);
  controls_.spoilersArmed = onRunway ? 1 : 0;
  controls_.autobrake = onRunway && !coldDark ? A320_AUTOBRAKE_MAX : A320_AUTOBRAKE_OFF;
  double speedKt = 0.0;
  RunwayPoint start;
  if (onRunway) {
    start = {-ils_->axes().displacementM() + kLineupDistanceM, 0.0, 0.0};
    controls_.parkBrake = 1;
    flaps_.setLever(1, 0.0);  // CONF 1+F for takeoff
  } else if (intercept) {
    // 3 NM left of the extended centreline, 20 NM out, at 3000 ft: a 30 degree intercept
    // that captures the localizer near 15 NM and the glideslope from below near 9 NM.
    start = {-20.0 * kNmToM, -3.0 * kNmToM, 3000.0 * kFtToM - rw.threshold.altM};
    controls_.gearDown = 0;
    speedKt = 220.0;
  } else {
    const double distM = (scenario == A320_SCENARIO_FINAL_10NM ? 10.0 : 4.0) * kNmToM;
    start = ils_->glidepathPoint(-distM);
    flaps_.setLever(scenario == A320_SCENARIO_FINAL_10NM ? 3 : 4, 200.0);
    speedKt = scenario == A320_SCENARIO_FINAL_10NM ? 160.0 : 150.0;
  }
  controls_.flapsLever = flaps_.lever();
  const GeoPos pos = frame_.toGeo(ils_->axes().toEnu(start));
  double thsDeg = 0.0;

  try {
    auto ic = fdm_->GetIC();
    ic->SetTerrainElevationFtIC(airport_.reference.altM * kMToFt);
    ic->SetGeodLatitudeDegIC(pos.latDeg);
    ic->SetLongitudeDegIC(pos.lonDeg);
    const double headingDeg = rw.trueCourseDeg + (intercept ? kInterceptDeg : 0.0);
    ic->SetPsiDegIC(headingDeg);
    ic->SetPhiDegIC(0.0);
    ic->SetThetaDegIC(0.0);
    ic->SetWindNEDFpsIC(0.0, 0.0, 0.0);
    if (onRunway) {
      ic->SetVcalibratedKtsIC(0.0);
      ic->SetAltitudeAGLFtIC(kRadioAltOffsetFt);
    } else {
      ic->SetAltitudeASLFtIC(pos.altM * kMToFt);
      ic->SetVcalibratedKtsIC(speedKt);
      ic->SetFlightPathAngleDegIC(intercept ? 0.0 : -ils_->glideslopeDeg());
    }

    fdm_->ResetToInitialConditions(0);
    // Commands must be set after the reset (it zeroes the FCS); positions are set too so
    // gear and flaps start where they are commanded instead of travelling there.
    setProp("gear/gear-cmd-norm", controls_.gearDown ? 1.0 : 0.0);
    setProp("gear/gear-pos-norm", controls_.gearDown ? 1.0 : 0.0);
    setProp("fcs/flap-cmd-norm", flaps_.flapTargetDeg() / 40.0);
    setProp("fcs/flap-pos-deg", flaps_.flapTargetDeg());
    for (int i = 0; i < 2; ++i) {
      char name[64];
      std::snprintf(name, sizeof(name), "fcs/throttle-cmd-norm[%d]", i);
      setProp(name, onRunway ? 0.0 : 0.5);
      std::snprintf(name, sizeof(name), "propulsion/engine[%d]/reverser-angle-rad", i);
      setProp(name, 0.0);
    }
    if (!coldDark) fdm_->GetPropulsion()->InitRunning(-1);
    thsDeg = onRunway ? kTakeoffThsDeg : trimAirborne();
    if (onRunway) {
      setProp("fcs/ths-pos-rad", thsDeg * kDegToRad);
      try {
        fdm_->DoTrim(JSBSim::tGround);
      } catch (const std::exception&) {
        // Settling on the gear happens in the first second of simulation anyway.
      }
    }
  } catch (const std::exception&) {
    return false;
  }

  controls_.thrustLever = onRunway ? 0.0 : clamp(prop("fcs/throttle-cmd-norm[0]"), 0.0, 1.0);
  throttle_ = controls_.thrustLever;
  {
    const double magVar = airport_.magneticVariationDeg;
    const double courseMag = rw.trueCourseDeg - magVar;
    // FCU preset: climb to 5000 ft at 200 kt after takeoff; on final, approach speed and a
    // 3000 ft missed-approach altitude.
    ap_.reset(onRunway ? 200.0 : speedKt, courseMag + (intercept ? kInterceptDeg : 0.0), onRunway ? 5000.0 : 3000.0);
    if (intercept) {
      // Levers in CL with A/THR flying the speed, as in normal operation.
      ap_.engageCruise();
      controls_.thrustLever = kLeverClimb;
      athrActive_ = true;
    }
  }
  fbw_.reset(onRunway ? PitchLaw::Ground : PitchLaw::Flight, 0.0, thsDeg);
  clock_.resetTime();
  refreshState();
  wasOnGround_ = state_.onGround != 0;
  wasAlphaFloor_ = false;
  airborneS_ = lastAirborneS_ = wasOnGround_ ? 0.0 : 60.0;
  return true;
}

void Simulation::updateEngines(bool bleedAvailable) {
  const A320Controls& c = controls_;
  for (int i = 0; i < 2; ++i) {
    auto engine = std::static_pointer_cast<JSBSim::FGTurbine>(fdm_->GetPropulsion()->GetEngine(i));
    if (!engine) continue;
    const bool running = engine->GetRunning();
    if (!c.engMaster[i]) {
      // Fuel off. With the selector in CRANK the starter still motors the engine dry.
      engine->SetCutoff(true);
      engine->SetStarter(!running && c.engMode == A320_ENG_MODE_CRANK && bleedAvailable);
    } else if (!running) {
      // FADEC automatic start: starter on, fuel and ignition at ~20 % N2 (JSBSim needs >15 %).
      const bool starter = c.engMode == A320_ENG_MODE_IGN_START && bleedAvailable;
      engine->SetStarter(starter);
      engine->SetCutoff(!(starter && engine->GetN2() >= 20.0));
    } else {
      engine->SetCutoff(false);
    }
  }
}

double Simulation::trimAirborne() {
  // JSBSim trims with the elevator (pitch-trim-cmd-norm). Start from a THS guess, then hand
  // the elevator trim over to the THS and trim again so the elevator starts near neutral.
  for (double guessDeg : {-4.0, -8.0, -12.0, 0.0}) {
    double thsDeg = guessDeg;
    bool trimmed = false;
    for (int pass = 0; pass < 3; ++pass) {
      setProp("fcs/ths-pos-rad", thsDeg * kDegToRad);
      setProp("fcs/pitch-trim-cmd-norm", 0.0);
      try {
        fdm_->DoTrim(JSBSim::tLongitudinal);
        trimmed = true;
      } catch (const std::exception&) {
        break;
      }
      const double elevatorDeg = prop("fcs/elevator-pos-deg");
      if (std::fabs(elevatorDeg) < 0.2) break;
      thsDeg = clamp(thsDeg + elevatorDeg * kElevatorToThs, kThsMinDeg, kThsMaxDeg);
    }
    if (trimmed) {
      // The FBW owns the elevator from here; fold any residual trim into the THS.
      thsDeg = clamp(thsDeg + prop("fcs/elevator-pos-deg") * kElevatorToThs, kThsMinDeg, kThsMaxDeg);
      setProp("fcs/pitch-trim-cmd-norm", 0.0);
      setProp("fcs/ths-pos-rad", thsDeg * kDegToRad);
      return thsDeg;
    }
  }
  setProp("fcs/pitch-trim-cmd-norm", 0.0);
  return -4.0;
}

void Simulation::hint(const char* text) {
  if (!text) return;
  std::snprintf(state_.hint, sizeof(state_.hint), "%s", text);
  ++state_.hintSeq;
}

void Simulation::setControls(const A320Controls& c) {
  // Sim tutor: explain switch and lever moves the aircraft refuses or ignores.
  const A320Controls& old = controls_;
  const A320State& s = state_;
  char text[256];
  if (!c.gearDown && old.gearDown && s.onGround) hint("GEAR: the lever can't be raised with weight on the wheels.");
  if (((c.reverse && !old.reverse) || (c.reverse2 && !old.reverse2)) && !s.onGround)
    hint("REVERSE: the reversers only deploy on the ground. In flight the levers give idle thrust.");
  if (c.flapsLever > old.flapsLever && s.iasKt > flapVfeKt(c.flapsLever) + 2.0) {
    std::snprintf(text, sizeof(text), "FLAPS: too fast. The limit (VFE) for this position is %.0f kt, you are at %.0f kt.",
                  flapVfeKt(c.flapsLever), s.iasKt);
    hint(text);
  }
  if (c.apuStart && !old.apuStart && !c.apuMaster) hint("APU START: switch the APU MASTER SW on first.");
  for (int i = 0; i < 2; ++i) {
    if (!c.engMaster[i] || old.engMaster[i] || s.engRunning[i]) continue;
    if (c.engMode != A320_ENG_MODE_IGN_START)
      hint("ENG START: set ENG MODE to IGN/START first, then the ENG MASTER switch.");
    else if (!s.bleedAvailable)
      hint("ENG START: no bleed air for the starter. Start the APU and switch APU BLEED on.");
  }
  controls_ = c;
}

ApInput Simulation::apInput() const {
  ApInput in;
  const A320State& s = state_;
  in.iasKt = s.iasKt;
  in.tasKt = s.tasKt;
  in.groundSpeedKt = s.groundSpeedKt;
  in.altitudeFt = s.altitudeFt;
  in.radioAltFt = s.radioAltFt;
  in.verticalSpeedFpm = s.verticalSpeedFpm;
  in.flightPathDeg = s.flightPathDeg;
  in.headingTrueDeg = s.headingTrueDeg;
  in.trackTrueDeg = s.trackTrueDeg;
  in.bankDeg = s.bankDeg;
  in.onGround = s.onGround != 0;
  in.locValid = s.locValid != 0;
  in.gsValid = s.gsValid != 0;
  in.locDots = s.locDots;
  in.gsDots = s.gsDots;
  in.dmeNm = s.dmeNm;
  in.ilsCourseTrueDeg = s.ilsCourseDeg;
  in.glideslopeDeg = ils_ ? ils_->glideslopeDeg() : 3.0;
  in.locDegPerDot = ils_ ? ils_->locHalfSectorDeg() / 2.0 : 0.8;
  in.magneticVariationDeg = airport_.magneticVariationDeg;
  in.pilotStickPitch = controls_.stickPitch;
  in.pilotStickRoll = controls_.stickRoll;
  in.thrustLever = thrustLevers(controls_).forward();
  in.alphaDeg = s.alphaDeg;
  in.vlsKt = s.vlsKt;
  in.vmaxKt = s.vmaxKt;
  in.currentThrottle = throttle_;
  in.dtS = clock_.stepS();
  return in;
}

void Simulation::fcuCommand(A320FcuCommand cmd) {
  hint(ap_.command(cmd, apInput()));
  refreshState();
}

void Simulation::setFcuTargets(double spdKt, double hdgMagDeg, double altFt, double vsFpm) {
  const double oldSpd = ap_.spdKt();
  ap_.setTargets(spdKt, hdgMagDeg, altFt, vsFpm);
  const double vls = std::ceil(state_.vlsKt);
  if (!state_.onGround && ap_.spdKt() < vls && oldSpd >= vls)
    hint("SPD below VLS (top of the amber band): the autopilot and A/THR will not fly slower than VLS.");
  refreshState();
}

void Simulation::update(double realDtS) {
  if (!fdm_) return;
  const int steps = clock_.advance(realDtS);
  for (int i = 0; i < steps; ++i) step();
  state_.simTimeS = clock_.simTimeS();
  state_.paused = clock_.paused() ? 1 : 0;
  state_.simRate = clock_.rate();
}

void Simulation::step() {
  if (!fdm_) return;
  applyControls();
  fdm_->Run();
  refreshState();
}

void Simulation::applyControls() {
  const A320Controls& c = controls_;
  const bool onGround = state_.onGround != 0;

  apu_.update(c.apuMaster != 0, c.apuStart != 0, clock_.stepS());
  bool anyEngineRunning = false;
  for (int i = 0; i < 2; ++i) anyEngineRunning = anyEngineRunning || state_.engRunning[i];
  updateEngines((c.apuBleed && apu_.avail()) || anyEngineRunning);

  const ApOutput ap = ap_.update(apInput());
  const bool alphaFloor = ap_.athrMode() == A320_ATHR_AFLOOR;
  if (alphaFloor && !wasAlphaFloor_)
    hint("ALPHA FLOOR: the angle of attack came close to the stall, so A/THR set TOGA thrust. Once the speed is back, "
         "press A/THR to end TOGA LK, then set the thrust levers.");
  wasAlphaFloor_ = alphaFloor;
  const double stickPitch = ap.apActive ? ap.stickPitch : c.stickPitch;
  const double stickRoll = ap.apActive ? ap.stickRoll : c.stickRoll;
  const double pedals = clamp(c.pedals + (ap.apActive ? ap.pedals : 0.0), -1.0, 1.0);

  FbwInput in;
  in.stickPitch = clamp(stickPitch, -1.0, 1.0);
  in.stickRoll = clamp(stickRoll, -1.0, 1.0);
  in.pitchDeg = state_.pitchDeg;
  in.flightPathDeg = state_.flightPathDeg;
  in.alphaDeg = state_.alphaDeg;
  in.bankDeg = state_.bankDeg;
  in.pitchRateDegS = prop("velocities/q-rad_sec") * kRadToDeg;
  in.rollRateDegS = prop("velocities/p-rad_sec") * kRadToDeg;
  in.loadFactor = state_.loadFactor;
  in.radioAltFt = state_.radioAltFt;
  in.verticalSpeedFpm = state_.verticalSpeedFpm;
  in.onGround = onGround;
  in.dtS = clock_.stepS();
  const FbwOutput out = fbw_.update(in);
  setProp("fcs/elevator-cmd-norm", out.elevatorCmd);
  setProp("fcs/ths-pos-rad", out.thsDeg * kDegToRad);
  setProp("fcs/aileron-cmd-norm", out.aileronCmd);
  // The model's rudder is positive trailing-edge left (nose left, CmDr < 0) while nosewheel
  // steering is positive right, so the pedal command is negated for the rudder only.
  setProp("fcs/rudder-cmd-norm", -pedals);
  setProp("fcs/steer-cmd-norm", pedals * steeringAuthority(state_.groundSpeedKt));

  // Reversers deploy on the ground only; a lever in reverse in flight gives idle thrust.
  const ThrustLevers levers = thrustLevers(c);
  const bool reverse = (levers.reverse[0] || levers.reverse[1]) && onGround;
  athrActive_ = ap.athrActive && !reverse;
  throttle_ = athrActive_ ? ap.throttle : levers.forward();
  for (int i = 0; i < 2; ++i) {
    const bool engReverse = levers.reverse[i] && onGround;
    double cmd = levers.reverse[i] ? (engReverse ? levers.lever[i] : 0.0) : levers.lever[i];
    // A/THR works below each engine's own lever, so a retarded lever keeps its engine back.
    if (athrActive_ && !levers.reverse[i]) cmd = ap.thrustOverride ? ap.throttle : std::fmin(ap.throttle, levers.lever[i]);
    char name[64];
    std::snprintf(name, sizeof(name), "fcs/throttle-cmd-norm[%d]", i);
    setProp(name, cmd);
    std::snprintf(name, sizeof(name), "propulsion/engine[%d]/reverser-angle-rad", i);
    setProp(name, engReverse ? kReverserAngleRad : 0.0);
  }

  GroundDecelInput gd;
  gd.onGround = onGround;
  gd.armed = c.spoilersArmed != 0;
  gd.autobrake = c.autobrake;
  gd.thrustLever = levers.forward();
  gd.reverse = reverse;
  gd.groundSpeedKt = state_.groundSpeedKt;
  gd.pilotBrake = std::fmax(c.brakeLeft, c.brakeRight);
  gd.dtS = clock_.stepS();
  const double autoBrake = decel_.update(gd);
  const double park = c.parkBrake ? 1.0 : 0.0;
  setProp("fcs/left-brake-cmd-norm", std::fmax(std::fmax(clamp(c.brakeLeft, 0.0, 1.0), park), autoBrake));
  setProp("fcs/right-brake-cmd-norm", std::fmax(std::fmax(clamp(c.brakeRight, 0.0, 1.0), park), autoBrake));

  // Ground interlock: the gear lever cannot be raised with weight on wheels.
  if (c.gearDown || !onGround) setProp("gear/gear-cmd-norm", c.gearDown ? 1.0 : 0.0);

  if (c.flapsLever != flaps_.lever()) flaps_.setLever(c.flapsLever, state_.iasKt);
  flaps_.update(state_.iasKt);
  setProp("fcs/flap-cmd-norm", flaps_.flapTargetDeg() / 40.0);
  setProp("fcs/speedbrake-cmd-norm", decel_.spoilersOut() ? 1.0 : clamp(c.speedbrake, 0.0, 1.0));
}

void Simulation::refreshState() {
  A320State& s = state_;
  s.structSize = sizeof(A320State);
  s.simTimeS = clock_.simTimeS();
  s.paused = clock_.paused() ? 1 : 0;
  s.simRate = clock_.rate();

  s.latDeg = prop("position/lat-geod-deg");
  s.lonDeg = prop("position/long-gc-deg");
  s.altitudeFt = prop("position/h-sl-ft");
  const Enu enu = frame_.toEnu({s.latDeg, s.lonDeg, s.altitudeFt * kFtToM});
  s.northM = enu.n;
  s.eastM = enu.e;
  s.heightAboveFieldM = s.altitudeFt * kFtToM - airport_.reference.altM;

  s.headingTrueDeg = prop("attitude/psi-deg");
  s.pitchDeg = prop("attitude/theta-deg");
  s.bankDeg = prop("attitude/phi-deg");
  const double vn = prop("velocities/v-north-fps"), ve = prop("velocities/v-east-fps");
  if (std::hypot(vn, ve) > 1.0) {
    s.trackTrueDeg = std::atan2(ve, vn) * kRadToDeg;
    if (s.trackTrueDeg < 0.0) s.trackTrueDeg += 360.0;
  } else {
    s.trackTrueDeg = s.headingTrueDeg;
  }
  s.flightPathDeg = prop("flight-path/gamma-deg");
  s.iasKt = std::fmax(prop("velocities/vc-kts"), 0.0);
  s.tasKt = prop("velocities/vtrue-kts");
  s.groundSpeedKt = prop("velocities/vg-fps") * kFpsToKt;
  s.mach = prop("velocities/mach");
  s.verticalSpeedFpm = prop("velocities/h-dot-fps") * 60.0;
  s.radioAltFt = std::fmax(prop("position/h-agl-ft") - kRadioAltOffsetFt, 0.0);
  s.alphaDeg = prop("aero/alpha-deg");
  s.loadFactor = prop("accelerations/Nz");
  s.onGround = (prop("gear/unit[1]/WOW") > 0.5 || prop("gear/unit[2]/WOW") > 0.5) ? 1 : 0;
  s.pitchLaw = static_cast<int>(fbw_.law());

  for (int i = 0; i < 2; ++i) {
    char name[64];
    std::snprintf(name, sizeof(name), "propulsion/engine[%d]/n1", i);
    s.n1[i] = prop(name);
    std::snprintf(name, sizeof(name), "propulsion/engine[%d]/n2", i);
    s.n2[i] = prop(name);
    std::snprintf(name, sizeof(name), "propulsion/engine[%d]/fuel-flow-rate-pps", i);
    s.fuelFlowKgH[i] = prop(name) * kLbsToKg * 3600.0;
  }
  s.fuelKg = prop("propulsion/total-fuel-lbs") * kLbsToKg;
  const double weightLbs = prop("inertia/weight-lbs");
  s.grossWeightKg = weightLbs * kLbsToKg;
  const ThrustLevers levers = thrustLevers(controls_);
  const double forwardLever = levers.forward();
  double reverseAmount = 0.0;
  s.reverse = 0;
  for (int i = 0; i < 2; ++i) {
    // A lever in reverse in flight gives idle, so it is shown at idle.
    s.thrustLeverEng[i] = levers.reverse[i] && !s.onGround ? 0.0 : levers.lever[i];
    s.reverseEng[i] = levers.reverse[i] && s.onGround ? 1 : 0;
    if (s.reverseEng[i]) {
      s.reverse = 1;
      reverseAmount = std::fmax(reverseAmount, levers.lever[i]);
    }
  }
  s.thrustLever = s.reverse ? reverseAmount : forwardLever;
  s.thrustDetent = static_cast<int>(thrustDetent(forwardLever));

  s.flapsLever = flaps_.lever();
  s.onePlusF = flaps_.onePlusF() ? 1 : 0;
  s.flapDeg = prop("fcs/flap-pos-deg");
  s.gearPos = prop("gear/gear-pos-norm");
  s.gearLeverDown = prop("gear/gear-cmd-norm") > 0.5 ? 1 : 0;
  s.speedbrakePos = prop("fcs/speedbrake-pos-norm");
  s.parkBrake = controls_.parkBrake ? 1 : 0;
  s.elevatorNorm = prop("fcs/elevator-pos-norm");
  s.aileronNorm = prop("fcs/left-aileron-pos-norm");
  s.rudderNorm = prop("fcs/rudder-pos-norm");
  s.thsDeg = prop("fcs/ths-pos-rad") * kRadToDeg;

  s.apEngaged = ap_.apEngaged() ? 1 : 0;
  s.ap1Engaged = ap_.ap1Engaged() ? 1 : 0;
  s.ap2Engaged = ap_.ap2Engaged() ? 1 : 0;
  s.athrEngaged = ap_.athrEngaged() ? 1 : 0;
  s.athrActive = athrActive_ ? 1 : 0;
  s.latMode = ap_.lateral();
  s.vertMode = ap_.vertical();
  s.athrMode = ap_.athrMode();
  s.armed = ap_.armed();
  s.fcuSpdKt = ap_.spdKt();
  s.fcuHdgMagDeg = ap_.hdgMagDeg();
  s.fcuAltFt = ap_.altFt();
  s.fcuVsFpm = ap_.vsFpm();
  s.apDisconnectSeq = ap_.disconnectSeq();

  s.apuN = apu_.n();
  s.apuAvail = apu_.avail() ? 1 : 0;
  s.apuStarting = apu_.starting() ? 1 : 0;
  s.apuMaster = controls_.apuMaster;
  s.apuBleed = controls_.apuBleed;
  s.engMode = controls_.engMode;
  for (int i = 0; i < 2; ++i) {
    auto engine = std::static_pointer_cast<JSBSim::FGTurbine>(fdm_->GetPropulsion()->GetEngine(i));
    s.engMaster[i] = controls_.engMaster[i];
    s.engRunning[i] = engine && engine->GetRunning() ? 1 : 0;
    s.engStarting[i] = engine && !engine->GetRunning() && engine->GetStarter() ? 1 : 0;
  }
  s.bleedAvailable = (controls_.apuBleed && apu_.avail()) || s.engRunning[0] || s.engRunning[1] ? 1 : 0;
  s.autobrake = decel_.mode();
  s.autobrakeActive = decel_.autobrakeActive() ? 1 : 0;
  s.autobrakeDecel = decel_.decelReached() ? 1 : 0;
  s.spoilersArmed = controls_.spoilersArmed;
  s.groundSpoilers = decel_.spoilersOut() ? 1 : 0;
  s.lights = controls_.lights;
  s.signs = controls_.signs;

  const bool takeoffPhase = s.onGround || (s.radioAltFt < 1500.0 && forwardLever > kLeverClimb + 0.05);
  const SpeedLimits lim = computeSpeedLimits(s.flapsLever, s.onePlusF != 0, s.flapDeg, weightLbs,
                                             s.gearPos > 0.05, takeoffPhase);
  s.vlsKt = lim.vlsKt;
  s.vsKt = lim.vsKt;
  s.vmaxKt = lim.vmaxKt;
  s.vfeNextKt = lim.vfeNextKt;

  s.ilsRunwayIndex = runwayIndex_;
  s.ilsCourseDeg = airport_.runways[runwayIndex_].trueCourseDeg;
  const IlsSignal sig = ils_->receive(enu);
  s.locValid = sig.locValid ? 1 : 0;
  s.gsValid = sig.gsValid ? 1 : 0;
  s.locDots = sig.locDots;
  s.gsDots = sig.gsDots;
  s.dmeNm = sig.dmeNm;
  const PapiState papi = computePapi(*ils_, enu);
  for (int i = 0; i < 4; ++i) s.papiWhite[i] = papi.white[i] ? 1 : 0;

  WarningInput w;
  w.onGround = s.onGround != 0;
  w.thrustLever = forwardLever;
  w.flapsLever = s.flapsLever;
  w.speedbrake = s.speedbrakePos;
  w.parkBrake = s.parkBrake != 0;
  w.gearDown = s.gearPos > 0.99;
  w.radioAltFt = s.radioAltFt;
  w.iasKt = s.iasKt;
  w.vmaxKt = s.vmaxKt;
  w.alphaDeg = s.alphaDeg;
  w.verticalSpeedFpm = s.verticalSpeedFpm;
  w.gsValid = s.gsValid != 0;
  w.gsDots = s.gsDots;
  s.warnings = computeWarnings(w);

  // Takeoff speeds follow weight and flaps until the takeoff roll starts.
  if (!takeoffCallouts_.rolling() && s.onGround && s.groundSpeedKt < 30.0) {
    const SpeedLimits takeoff = computeSpeedLimits(s.flapsLever, s.onePlusF != 0, s.flapDeg, weightLbs, true, true);
    const TakeoffSpeeds speeds = computeTakeoffSpeeds(takeoff.vsKt);
    s.v1Kt = speeds.v1Kt;
    s.vrKt = speeds.vrKt;
    s.v2Kt = speeds.v2Kt;
  }
  const TakeoffSpeeds speeds{s.v1Kt, s.vrKt, s.v2Kt};
  const char* takeoffCall =
      takeoffCallouts_.update(s.onGround != 0, s.iasKt, forwardLever, s.verticalSpeedFpm, s.radioAltFt, speeds);
  if (takeoffCall) {
    std::snprintf(s.callout, sizeof(s.callout), "%s", takeoffCall);
    ++s.calloutSeq;
  } else if (const char* text = callouts_.update(s.radioAltFt, s.onGround != 0, forwardLever)) {
    std::snprintf(s.callout, sizeof(s.callout), "%s", text);
    ++s.calloutSeq;
  }

  airborneS_ = s.onGround ? 0.0 : airborneS_ + clock_.stepS();
  // A skip of a few feet after touchdown is the same landing, not a new one.
  if (s.onGround && !wasOnGround_ && lastAirborneS_ > 3.0) {
    const RunwayPoint p = ils_->axes().fromEnu(enu);
    s.touchdownFpm = s.verticalSpeedFpm;
    s.touchdownDistanceM = p.x;
    s.touchdownCenterlineM = p.y;
    ++s.touchdownSeq;
  }
  if (!s.onGround) lastAirborneS_ = airborneS_;
  wasOnGround_ = s.onGround != 0;
}

}  // namespace a320
