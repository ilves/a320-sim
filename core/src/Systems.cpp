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

SpeedLimits computeSpeedLimits(int flapsLever, bool onePlusF, double flapDeg, double weightLbs,
                               bool gearDown, bool takeoffPhase) {
  // Peak of each flap column of the CLalpha table in A320.xml (0, 1, 9, 10, 40 deg).
  static const double kFlapCol[5] = {0.0, 1.0, 9.0, 10.0, 40.0};
  static const double kClMax[5] = {1.50, 1.61, 1.66, 1.90, 2.40};
  static const double kVfe[5] = {350.0, 230.0, 200.0, 185.0, 177.0};
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
  s.vmaxKt = lever == 1 && onePlusF ? kVfeOnePlusF : kVfe[lever];
  if (gearDown) s.vmaxKt = std::fmin(s.vmaxKt, kVle);
  s.vfeNextKt = lever < 4 ? kVfe[lever + 1] : 0.0;
  if (lever == 0) s.vfeNextKt = kVfe[1];
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
