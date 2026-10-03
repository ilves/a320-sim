#pragma once

#include <cstdint>

#include "a320/a320_api.h"

namespace a320 {

// Flaps lever positions 0, 1, 2, 3, FULL. Position 1 gives CONF 1+F (flaps 10) when
// selected below 100 kt, and 1+F auto-retracts to CONF 1 above 210 kt.
class FlapsSystem {
 public:
  void setLever(int position, double iasKt);
  void update(double iasKt);
  int lever() const { return lever_; }
  double flapTargetDeg() const;
  bool onePlusF() const { return lever_ == 1 && onePlusF_; }

 private:
  int lever_ = 0;
  bool onePlusF_ = false;
};

const char* flapConfigName(int lever, bool onePlusF);

enum class ThrustDetent { Idle, Climb, FlexMct, Toga, Manual };

// Thrust lever position 0..1 with the A320 detents; between detents is manual thrust.
ThrustDetent thrustDetent(double lever);
const char* thrustDetentName(ThrustDetent d);
constexpr double kLeverClimb = 0.75;
constexpr double kLeverFlexMct = 0.88;

// Nosewheel steering authority vs ground speed: full tiller authority while taxiing,
// rudder-pedal authority (about 6 degrees of 75) at takeoff and landing speeds.
double steeringAuthority(double groundSpeedKt);

struct SpeedLimits {
  double vmaxKt = 350.0;   // min(VMO, VFE, VLE)
  double vfeNextKt = 0.0;  // VFE of the next flaps position, 0 when FULL
  double vlsKt = 0.0;      // lowest selectable speed, 1.23 x Vs1g (1.13 x on takeoff)
  double vsKt = 0.0;       // 1 g stall speed
};

// Stall speed from the JSBSim A320 model's CLmax for the actual flap angle and weight.
SpeedLimits computeSpeedLimits(int flapsLever, bool onePlusF, double flapDeg, double weightLbs,
                               bool gearDown, bool takeoffPhase);

enum Warning : uint32_t {
  kWarnNone = 0,
  kWarnConfigFlaps = A320_WARN_CONFIG_FLAPS,
  kWarnConfigSpdBrk = A320_WARN_CONFIG_SPD_BRK,
  kWarnConfigParkBrk = A320_WARN_CONFIG_PARK_BRK,
  kWarnGearNotDown = A320_WARN_GEAR_NOT_DOWN,
  kWarnOverspeed = A320_WARN_OVERSPEED,
  kWarnStall = A320_WARN_STALL,
  kWarnGlideslope = A320_WARN_GLIDESLOPE,
  kWarnSinkRate = A320_WARN_SINK_RATE,
};

struct WarningInput {
  bool onGround = true;
  double thrustLever = 0.0;
  int flapsLever = 0;
  double speedbrake = 0.0;
  bool parkBrake = false;
  bool gearDown = true;
  double radioAltFt = 0.0;
  double iasKt = 0.0;
  double vmaxKt = 350.0;
  double alphaDeg = 0.0;
  double verticalSpeedFpm = 0.0;
  bool gsValid = false;
  double gsDots = 0.0;
};

uint32_t computeWarnings(const WarningInput& in);
const char* warningText(Warning w);

// Radio-altitude callouts announced when descending through each height.
class Callouts {
 public:
  // Returns the callout text for this step, or nullptr.
  const char* update(double radioAltFt, bool onGround, double thrustLever);
  void reset() { previousFt_ = -1.0; }

 private:
  double previousFt_ = -1.0;
};

}  // namespace a320
