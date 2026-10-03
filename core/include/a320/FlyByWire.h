#pragma once

namespace a320 {

enum class PitchLaw { Ground, Flight, Flare };

struct FbwInput {
  double stickPitch = 0.0;  // +1 = full back (nose up)
  double stickRoll = 0.0;   // +1 = full right
  double pitchDeg = 0.0;
  double flightPathDeg = 0.0;
  double alphaDeg = 0.0;
  double bankDeg = 0.0;
  double pitchRateDegS = 0.0;
  double rollRateDegS = 0.0;
  double loadFactor = 1.0;  // Nz, 1 in level flight
  double radioAltFt = 0.0;
  double verticalSpeedFpm = 0.0;
  bool onGround = true;
  double dtS = 1.0 / 120.0;
};

struct FbwOutput {
  double elevatorCmd = 0.0;  // JSBSim convention: negative = trailing edge up (nose up)
  double aileronCmd = 0.0;   // positive = roll right
  double thsDeg = 0.0;       // trimmable horizontal stabiliser, negative = nose up
  PitchLaw law = PitchLaw::Ground;
};

constexpr double kThsMinDeg = -13.5;
constexpr double kThsMaxDeg = 4.0;

// Simplified Airbus Normal Law: stick commands load factor in pitch and roll rate in roll,
// neutral stick holds flight path and bank, with bank/pitch attitude and angle-of-attack
// protections (full back stick holds alpha max instead of stalling). In flight
// law the THS auto-trims so the elevator returns towards neutral; it is frozen on the
// ground and in the flare, as on the real aircraft.
class FlyByWire {
 public:
  FbwOutput update(const FbwInput& in);
  void reset(PitchLaw law, double elevatorCmd, double thsDeg);
  PitchLaw law() const { return law_; }

 private:
  double flightPitch(const FbwInput& in);
  double flarePitch(const FbwInput& in);
  double roll(const FbwInput& in);
  void enter(PitchLaw law, const FbwInput& in);

  PitchLaw law_ = PitchLaw::Ground;
  double elevator_ = 0.0;      // last elevator command, for bumpless transfer
  double thsDeg_ = 0.0;
  double pitchIntegral_ = 0.0;  // acts as auto-trim
  double rollIntegral_ = 0.0;
  double bankHoldDeg_ = 0.0;
  bool bankHeld_ = false;
  double airborneS_ = 0.0;
  double groundS_ = 0.0;
  double flareTimeS_ = 0.0;
  double flarePitchMemDeg_ = 0.0;
};

}  // namespace a320
