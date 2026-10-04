#include "a320/FlyByWire.h"

#include <cmath>

#include "a320/Units.h"

namespace a320 {
namespace {

constexpr double kMaxRollRateDegS = 15.0;
constexpr double kBankNeutralLimitDeg = 33.0;
constexpr double kBankMaxDeg = 67.0;
constexpr double kPitchMaxDeg = 30.0;
constexpr double kPitchMinDeg = -15.0;
constexpr double kStickDeadband = 0.05;
constexpr double kFlareEntryFt = 50.0;
constexpr double kFlareExitFt = 100.0;

double deadband(double v) {
  if (std::fabs(v) < kStickDeadband) return 0.0;
  return (v - std::copysign(kStickDeadband, v)) / (1.0 - kStickDeadband);
}

}  // namespace

void FlyByWire::reset(PitchLaw law, double elevatorCmd, double thsDeg) {
  law_ = law;
  elevator_ = elevatorCmd;
  thsDeg_ = clamp(thsDeg, kThsMinDeg, kThsMaxDeg);
  pitchIntegral_ = elevatorCmd;
  rollIntegral_ = 0.0;
  bankHeld_ = false;
  airborneS_ = 0.0;
  groundS_ = 0.0;
  flareTimeS_ = 0.0;
  lastAlphaDeg_ = -100.0;
}

void FlyByWire::enter(PitchLaw law, const FbwInput& in) {
  law_ = law;
  // Bumpless transfer: the integrators start from the surface position already applied.
  pitchIntegral_ = elevator_;
  if (law == PitchLaw::Flare) {
    flareTimeS_ = 0.0;
    flarePitchMemDeg_ = in.pitchDeg;
  }
}

FbwOutput FlyByWire::update(const FbwInput& in) {
  airborneS_ = in.onGround ? 0.0 : airborneS_ + in.dtS;
  groundS_ = in.onGround ? groundS_ + in.dtS : 0.0;

  switch (law_) {
    case PitchLaw::Ground:
      if (airborneS_ > 1.0) enter(PitchLaw::Flight, in);
      break;
    case PitchLaw::Flight:
      if (groundS_ > 0.5) enter(PitchLaw::Ground, in);
      else if (in.radioAltFt < kFlareEntryFt && in.verticalSpeedFpm < 0.0 && airborneS_ > 10.0)
        enter(PitchLaw::Flare, in);
      break;
    case PitchLaw::Flare:
      if (groundS_ > 0.5) enter(PitchLaw::Ground, in);
      else if (in.radioAltFt > kFlareExitFt) enter(PitchLaw::Flight, in);
      break;
  }

  FbwOutput out;
  out.law = law_;
  switch (law_) {
    case PitchLaw::Ground:
      elevator_ = -deadband(in.stickPitch);
      out.aileronCmd = deadband(in.stickRoll);
      rollIntegral_ = 0.0;
      bankHeld_ = false;
      break;
    case PitchLaw::Flight: {
      elevator_ = flightPitch(in);
      out.aileronCmd = roll(in);
      // Auto-trim: move the THS (max 1 deg/s) to take over the elevator's steady deflection.
      const double elevatorDeg = elevator_ * 25.0;
      // Slow compared with the pitch loop so the two integrators do not fight.
      double trimRate = clamp(0.05 * elevatorDeg, -1.0, 1.0);
      // As on the aircraft, no nose-up auto-trim in alpha protection or at high pitch: trimming
      // into the protection would hold the nose up after the stick is released.
      if (trimRate < 0.0 && (in.alphaDeg > kAlphaProtDeg || in.pitchDeg > kPitchMaxDeg - 5.0)) trimRate = 0.0;
      thsDeg_ = clamp(thsDeg_ + trimRate * in.dtS, kThsMinDeg, kThsMaxDeg);
      break;
    }
    case PitchLaw::Flare:
      elevator_ = flarePitch(in);
      out.aileronCmd = roll(in);
      break;
  }
  out.elevatorCmd = elevator_;
  out.thsDeg = thsDeg_;
  return out;
}

double FlyByWire::flightPitch(const FbwInput& in) {
  const double stick = deadband(in.stickPitch);
  // Full back = 2.5 g, full forward = -1 g (clean configuration limits).
  double deltaN = stick >= 0.0 ? stick * 1.5 : stick * 2.0;

  // Pitch attitude protection fades the order out near the limits.
  if (in.pitchDeg > kPitchMaxDeg - 5.0 && deltaN > 0.0)
    deltaN *= clamp((kPitchMaxDeg - in.pitchDeg) / 5.0, 0.0, 1.0);
  if (in.pitchDeg < kPitchMinDeg + 5.0 && deltaN < 0.0)
    deltaN *= clamp((in.pitchDeg - kPitchMinDeg) / 5.0, 0.0, 1.0);

  // Neutral stick holds the flight path, compensating bank up to 33 degrees.
  const double bankComp = clamp(in.bankDeg, -kBankNeutralLimitDeg, kBankNeutralLimitDeg) * kDegToRad;
  const double nzOneG = std::cos(in.flightPathDeg * kDegToRad) / std::cos(bankComp);
  double nzCmd = nzOneG + deltaN;
  // Alpha protection: above alpha prot the stick commands alpha (neutral = alpha prot,
  // full back = alpha max), so the aircraft gives up flight path rather than stall.
  const double alphaTarget = kAlphaProtDeg + clamp(stick, 0.0, 1.0) * (kAlphaMaxDeg - kAlphaProtDeg);
  // Alpha rate damps the approach to alpha max (the phugoid would otherwise overshoot it).
  const double alphaRate = in.dtS > 0.0 && lastAlphaDeg_ > -90.0 ? (in.alphaDeg - lastAlphaDeg_) / in.dtS : 0.0;
  lastAlphaDeg_ = in.alphaDeg;
  nzCmd = std::fmin(nzCmd, nzOneG + 0.3 * (alphaTarget - in.alphaDeg) - 0.3 * alphaRate);
  // Above the pitch limit the nose is brought back down even with the stick neutral.
  if (in.pitchDeg > kPitchMaxDeg) nzCmd = std::fmin(nzCmd, nzOneG - 0.1 * (in.pitchDeg - kPitchMaxDeg));
  const double err = nzCmd - in.loadFactor;

  // Gains tuned against the JSBSim model (tests/test_flight.cpp); kQ is per deg/s.
  constexpr double kP = 0.5, kI = 0.8, kQ = 0.03;
  pitchIntegral_ = clamp(pitchIntegral_ - kI * err * in.dtS, -1.0, 1.0);
  return clamp(pitchIntegral_ - kP * err + kQ * in.pitchRateDegS, -1.0, 1.0);
}

double FlyByWire::flarePitch(const FbwInput& in) {
  // Real flare law: attitude memorised at 50 ft, then 2 degrees nose down over 8 s from
  // 30 ft, so the pilot has to pull to flare.
  if (in.radioAltFt < 30.0) flareTimeS_ += in.dtS;
  const double target = flarePitchMemDeg_ - 2.0 * clamp(flareTimeS_ / 8.0, 0.0, 1.0) +
                        deadband(in.stickPitch) * 10.0;
  const double err = target - in.pitchDeg;
  constexpr double kP = 0.08, kI = 0.05, kQ = 0.25;
  pitchIntegral_ = clamp(pitchIntegral_ - kI * err * in.dtS, -1.0, 1.0);
  return clamp(pitchIntegral_ - kP * err + kQ * in.pitchRateDegS * kDegToRad * 10.0, -1.0, 1.0);
}

double FlyByWire::roll(const FbwInput& in) {
  const double stick = deadband(in.stickRoll);
  double rateCmd;
  if (stick != 0.0) {
    bankHeld_ = false;
    rateCmd = stick * kMaxRollRateDegS;
    // Bank protection: the roll order fades to zero at 67 degrees.
    if (std::fabs(in.bankDeg) > kBankMaxDeg - 10.0 && rateCmd * in.bankDeg > 0.0) {
      rateCmd *= clamp((kBankMaxDeg - std::fabs(in.bankDeg)) / 10.0, 0.0, 1.0);
      if (std::fabs(in.bankDeg) > kBankMaxDeg)
        rateCmd = -std::copysign((std::fabs(in.bankDeg) - kBankMaxDeg) * 0.8, in.bankDeg);
    }
  } else {
    if (!bankHeld_) {
      bankHeld_ = true;
      bankHoldDeg_ = in.bankDeg;
    }
    // Released beyond 33 degrees the aircraft rolls back to 33.
    bankHoldDeg_ = clamp(bankHoldDeg_, -kBankNeutralLimitDeg, kBankNeutralLimitDeg);
    rateCmd = clamp((bankHoldDeg_ - in.bankDeg) * 0.8, -kMaxRollRateDegS, kMaxRollRateDegS);
  }
  const double err = rateCmd - in.rollRateDegS;
  constexpr double kP = 0.06, kI = 0.04;
  rollIntegral_ = clamp(rollIntegral_ + kI * err * in.dtS, -0.5, 0.5);
  return clamp(kP * err + rollIntegral_, -1.0, 1.0);
}

}  // namespace a320
