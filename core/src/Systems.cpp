#include "a320/Systems.h"

#include <cmath>

#include "a320/Units.h"

namespace a320 {

void FlapsSystem::setLever(int position, double iasKt) {
  position = clamp(position, 0, 4);
  if (position == 1) onePlusF_ = lever_ == 0 ? iasKt < 100.0 : lever_ > 1;
  lever_ = position;
}

void FlapsSystem::update(double iasKt) {
  if (lever_ == 1 && onePlusF_ && iasKt > 210.0) onePlusF_ = false;
}

double FlapsSystem::flapTargetDeg() const {
  static const double kFlapDeg[5] = {0.0, 0.0, 15.0, 20.0, 35.0};
  return lever_ == 1 && onePlusF_ ? 10.0 : kFlapDeg[lever_];
}

const char* flapConfigName(int lever, bool onePlusF) {
  static const char* kNames[5] = {"0", "1", "2", "3", "FULL"};
  return lever == 1 && onePlusF ? "1+F" : kNames[clamp(lever, 0, 4)];
}

ThrustDetent thrustDetent(double lever) {
  constexpr double kTol = 0.02;
  if (lever <= kTol) return ThrustDetent::Idle;
  if (std::fabs(lever - kLeverClimb) <= kTol) return ThrustDetent::Climb;
  if (std::fabs(lever - kLeverFlexMct) <= kTol) return ThrustDetent::FlexMct;
  if (lever >= 1.0 - kTol) return ThrustDetent::Toga;
  return ThrustDetent::Manual;
}

ThrustLevers thrustLevers(const A320Controls& c) {
  ThrustLevers t;
  t.lever[0] = clamp(c.thrustLever, 0.0, 1.0);
  t.reverse[0] = c.reverse != 0;
  t.lever[1] = c.splitThrust ? clamp(c.thrustLever2, 0.0, 1.0) : t.lever[0];
  t.reverse[1] = c.splitThrust ? c.reverse2 != 0 : t.reverse[0];
  return t;
}

double ThrustLevers::forward() const {
  return std::fmax(reverse[0] ? 0.0 : lever[0], reverse[1] ? 0.0 : lever[1]);
}

const char* thrustDetentName(ThrustDetent d) {
  switch (d) {
    case ThrustDetent::Idle: return "IDLE";
    case ThrustDetent::Climb: return "CL";
    case ThrustDetent::FlexMct: return "FLX/MCT";
    case ThrustDetent::Toga: return "TOGA";
    case ThrustDetent::Manual: return "MAN";
  }
  return "";
}

double steeringAuthority(double groundSpeedKt) {
  constexpr double kPedalOnly = 6.0 / 75.0;
  return lerp(1.0, kPedalOnly, clamp((groundSpeedKt - 10.0) / 30.0, 0.0, 1.0));
}

double flapVfeKt(int flapsLever) {
  static const double kVfe[5] = {350.0, 230.0, 200.0, 185.0, 177.0};
  return kVfe[clamp(flapsLever, 0, 4)];
}

SpeedLimits computeSpeedLimits(int flapsLever, bool onePlusF, double flapDeg, double weightLbs,
                               bool gearDown, bool takeoffPhase) {
  // Peak of each flap column of the CLalpha table in A320.xml (0, 1, 9, 10, 40 deg).
  static const double kFlapCol[5] = {0.0, 1.0, 9.0, 10.0, 40.0};
  static const double kClMax[5] = {1.50, 1.61, 1.66, 1.90, 2.40};
  constexpr double kVfeOnePlusF = 215.0;
  constexpr double kVle = 280.0;
  constexpr double kWingAreaFt2 = 1317.0;
  constexpr double kRho0 = 0.0023769;  // slug/ft3; IAS ~ EAS at airport altitudes

  double clMax = kClMax[4];
  for (int i = 0; i < 4; ++i) {
    if (flapDeg <= kFlapCol[i + 1]) {
      const double t = (flapDeg - kFlapCol[i]) / (kFlapCol[i + 1] - kFlapCol[i]);
      clMax = lerp(kClMax[i], kClMax[i + 1], clamp(t, 0.0, 1.0));
      break;
    }
  }

  SpeedLimits s;
  const double vsFps = std::sqrt(2.0 * weightLbs / (kRho0 * kWingAreaFt2 * clMax));
  s.vsKt = vsFps * kFpsToKt;
  s.vlsKt = s.vsKt * (takeoffPhase ? 1.13 : 1.23);

  const int lever = clamp(flapsLever, 0, 4);
  s.vmaxKt = lever == 1 && onePlusF ? kVfeOnePlusF : flapVfeKt(lever);
  if (gearDown) s.vmaxKt = std::fmin(s.vmaxKt, kVle);
  s.vfeNextKt = lever < 4 ? flapVfeKt(lever + 1) : 0.0;
  return s;
}

uint32_t computeWarnings(const WarningInput& in) {
  uint32_t w = kWarnNone;
  const bool takeoffThrust = in.onGround && in.thrustLever >= kLeverFlexMct - 0.02;
  if (takeoffThrust && (in.flapsLever == 0 || in.flapsLever == 4)) w |= kWarnConfigFlaps;
  if (takeoffThrust && in.speedbrake > 0.05) w |= kWarnConfigSpdBrk;
  if (takeoffThrust && in.parkBrake) w |= kWarnConfigParkBrk;
  if (!in.onGround && !in.gearDown && in.radioAltFt < 750.0 &&
      (in.flapsLever >= 3 || in.thrustLever < 0.05))
    w |= kWarnGearNotDown;
  if (in.iasKt > in.vmaxKt + 4.0) w |= kWarnOverspeed;
  // Stall warning a little below the model's CLmax angle of attack (~16 degrees).
  if (!in.onGround && in.alphaDeg > 14.0) w |= kWarnStall;
  if (!in.onGround && in.gsValid && in.gsDots > 1.3 && in.radioAltFt < 1000.0 &&
      in.radioAltFt > 30.0)
    w |= kWarnGlideslope;
  // Approximation of GPWS mode 1 (excessive descent rate).
  if (!in.onGround && in.radioAltFt < 2450.0 && in.radioAltFt > 30.0 &&
      -in.verticalSpeedFpm > 1000.0 + in.radioAltFt * 1.2)
    w |= kWarnSinkRate;
  return w;
}

const char* warningText(Warning w) {
  switch (w) {
    case kWarnConfigFlaps: return "CONFIG FLAPS NOT IN T.O CONFIG";
    case kWarnConfigSpdBrk: return "CONFIG SPD BRK NOT RETRACTED";
    case kWarnConfigParkBrk: return "CONFIG PARK BRK ON";
    case kWarnGearNotDown: return "L/G GEAR NOT DOWN";
    case kWarnOverspeed: return "OVERSPEED";
    case kWarnStall: return "STALL";
    case kWarnGlideslope: return "GLIDE SLOPE";
    case kWarnSinkRate: return "SINK RATE";
    default: return "";
  }
}

void Apu::update(bool master, bool startButton, double dtS) {
  if (!master) {
    starting_ = false;
    n_ = std::fmax(n_ - 5.0 * dtS, 0.0);
  } else {
    if (startButton && !lastStart_ && n_ < 95.0) starting_ = true;
    if (starting_) {
      n_ = std::fmin(n_ + 3.5 * dtS, 100.0);
      if (n_ >= 100.0) starting_ = false;
    }
  }
  lastStart_ = startButton;
}

void Apu::setRunning(bool running) {
  n_ = running ? 100.0 : 0.0;
  starting_ = false;
}

void GroundDecel::reset() {
  spoilers_ = active_ = decelLight_ = false;
  mode_ = requested_ = A320_AUTOBRAKE_OFF;
  lastGsKt_ = -1.0;
  decel_ = brake_ = 0.0;
}

double GroundDecel::update(const GroundDecelInput& in) {
  // Measured deceleration from ground speed, smoothed.
  if (lastGsKt_ >= 0.0 && in.dtS > 0.0) {
    const double d = (lastGsKt_ - in.groundSpeedKt) * kKtToMps / in.dtS;
    decel_ += (d - decel_) * 0.05;
  }
  lastGsKt_ = in.groundSpeedKt;

  const bool idle = in.thrustLever < 0.05;
  if (!in.onGround || (!idle && !in.reverse) || (!in.armed && !in.reverse)) spoilers_ = false;
  else if (in.reverse || (in.armed && idle && in.groundSpeedKt > 72.0)) spoilers_ = true;

  if (in.autobrake != requested_) {
    requested_ = in.autobrake;
    mode_ = requested_;
    if (mode_ == A320_AUTOBRAKE_OFF) active_ = false;
  }
  // Disarmed by firm pilot braking or by advancing the thrust levers.
  if (active_ && (in.pilotBrake > 0.6 || (!idle && !in.reverse))) {
    active_ = false;
    mode_ = A320_AUTOBRAKE_OFF;
  }
  if (!active_ && mode_ != A320_AUTOBRAKE_OFF && spoilers_) active_ = true;
  if (!active_) {
    brake_ = 0.0;
    decelLight_ = false;
    return 0.0;
  }

  if (mode_ == A320_AUTOBRAKE_MAX) {
    brake_ = 1.0;
  } else {
    const double target = mode_ == A320_AUTOBRAKE_LO ? 1.7 : 3.0;
    brake_ = clamp(brake_ + 0.25 * (target - decel_) * in.dtS, 0.0, 1.0);
    decelLight_ = decel_ > 0.8 * target;
  }
  if (mode_ == A320_AUTOBRAKE_MAX) decelLight_ = decel_ > 4.0;
  // Hold the aircraft once stopped.
  if (in.groundSpeedKt < 1.0) brake_ = std::fmax(brake_, 0.5);
  return brake_;
}

TakeoffSpeeds computeTakeoffSpeeds(double stallKt) {
  TakeoffSpeeds t;
  t.v2Kt = std::ceil(std::fmax(1.18 * stallKt, 120.0));
  t.vrKt = std::fmax(t.v2Kt - 4.0, 118.0);  // VMCA-type floor
  t.v1Kt = std::fmax(t.vrKt - 2.0, 112.0);  // VMCG-type floor
  return t;
}

const char* TakeoffCallouts::update(bool onGround, double iasKt, double thrustLever, double verticalSpeedFpm,
                                    double radioAltFt, const TakeoffSpeeds& speeds) {
  const double prev = previousIas_;
  previousIas_ = iasKt;
  if (onGround && !rolling_) {
    // A takeoff starts with takeoff thrust (FLX/MCT or TOGA) from low speed.
    if (iasKt < 60.0 && thrustLever >= kLeverFlexMct - 0.02) {
      rolling_ = true;
      next_ = 0;
    }
    return nullptr;
  }
  if (!rolling_) return nullptr;
  if (onGround && thrustLever < kLeverClimb - 0.05) {
    rolling_ = false;  // rejected takeoff
    return nullptr;
  }
  struct Call { double kt; const char* text; };
  const Call calls[] = {{100.0, "ONE HUNDRED KNOTS"}, {speeds.v1Kt, "V ONE"}, {speeds.vrKt, "ROTATE"}};
  if (next_ < 3 && prev < calls[next_].kt && iasKt >= calls[next_].kt) return calls[next_++].text;
  if (next_ >= 1 && next_ <= 3 && !onGround && verticalSpeedFpm > 300.0 && radioAltFt > 30.0) {
    next_ = 4;
    rolling_ = false;
    return "POSITIVE CLIMB";
  }
  return nullptr;
}

const char* Callouts::update(double radioAltFt, bool onGround, double thrustLever) {
  struct Callout { double ft; const char* text; };
  static const Callout kCallouts[] = {
      {2500.0, "TWO THOUSAND FIVE HUNDRED"}, {1000.0, "ONE THOUSAND"}, {500.0, "FIVE HUNDRED"},
      {100.0, "ONE HUNDRED"}, {50.0, "FIFTY"}, {40.0, "FORTY"}, {30.0, "THIRTY"},
      {20.0, "TWENTY"}, {10.0, "TEN"}};

  const double prev = previousFt_;
  previousFt_ = radioAltFt;
  if (prev < 0.0 || onGround || radioAltFt >= prev) return nullptr;
  // Manual landing: "RETARD" replaces "TWENTY" while the thrust levers are above idle.
  if (prev > 20.0 && radioAltFt <= 20.0 && thrustLever > 0.02) return "RETARD";
  for (const Callout& c : kCallouts) {
    if (prev > c.ft && radioAltFt <= c.ft) return c.text;
  }
  return nullptr;
}

}  // namespace a320
