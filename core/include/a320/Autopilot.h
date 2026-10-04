#pragma once

#include <cstdint>

#include "a320/a320_api.h"

namespace a320 {

struct ApInput {
  double iasKt = 0.0, tasKt = 0.0, groundSpeedKt = 0.0;
  double altitudeFt = 0.0, radioAltFt = 0.0, verticalSpeedFpm = 0.0, flightPathDeg = 0.0;
  double headingTrueDeg = 0.0, trackTrueDeg = 0.0, bankDeg = 0.0;
  bool onGround = true;
  bool locValid = false, gsValid = false;
  double locDots = 0.0, gsDots = 0.0, dmeNm = 0.0;
  double ilsCourseTrueDeg = 0.0, glideslopeDeg = 3.0, locDegPerDot = 0.8;
  double magneticVariationDeg = 0.0;
  double pilotStickPitch = 0.0, pilotStickRoll = 0.0;  // for instinctive disconnect
  double thrustLever = 0.0;    // pilot's lever; autothrust works below it, up to CL
  double alphaDeg = 0.0;
  double vlsKt = 0.0, vmaxKt = 400.0;  // speed protections for the selected speed
  double currentThrottle = 0.0;  // what the engines are commanded now, for bumpless engagement
  double dtS = 1.0 / 120.0;
};

struct ApOutput {
  bool apActive = false;
  double stickPitch = 0.0, stickRoll = 0.0, pedals = 0.0;
  bool athrActive = false;
  bool thrustOverride = false;  // alpha floor / TOGA LK: thrust not limited by the levers
  double throttle = 0.0;
};

// Simplified A320 autopilot and autothrust: the AP flies through the normal-law FBW with
// sidestick-like orders, as the real flight guidance does through the ELACs.
class Autopilot {
 public:
  void reset(double spdKt, double hdgMagDeg, double altFt);
  // Returns a tutor hint when the press is refused or needs explaining, else nullptr.
  const char* command(A320FcuCommand cmd, const ApInput& in);
  // Scenario start: AP1 in HDG and ALT, A/THR in SPEED.
  void engageCruise();
  void setTargets(double spdKt, double hdgMagDeg, double altFt, double vsFpm);
  ApOutput update(const ApInput& in);

  bool apEngaged() const { return ap1_ || ap2_; }
  bool ap1Engaged() const { return ap1_; }
  bool ap2Engaged() const { return ap2_; }
  bool athrEngaged() const { return athr_; }
  A320LatMode lateral() const { return lat_; }
  A320VertMode vertical() const { return vert_; }
  A320AthrMode athrMode() const { return athrMode_; }
  int armed() const;
  double spdKt() const { return spd_; }
  double hdgMagDeg() const { return hdg_; }
  double altFt() const { return alt_; }
  double vsFpm() const { return vs_; }
  uint32_t disconnectSeq() const { return disconnects_; }

 private:
  void disconnectAp();
  bool approachMode() const;
  const char* engageAp(bool& self, bool& other, const ApInput& in);
  double lateralBank(const ApInput& in);
  double verticalFpa(const ApInput& in);
  void updateModes(const ApInput& in);
  double autothrust(const ApInput& in, bool& active);
  void enterVertical(A320VertMode mode, const ApInput& in);
  // The selected speed kept between VLS and VMAX, as the flight guidance does.
  double protectedSpeed(const ApInput& in) const;

  bool ap1_ = false, ap2_ = false, athr_ = false;
  A320LatMode lat_ = A320_LAT_NONE;
  A320VertMode vert_ = A320_VERT_NONE;
  A320AthrMode athrMode_ = A320_ATHR_OFF;
  bool locArmed_ = false, gsArmed_ = false;
  double spd_ = 160.0, hdg_ = 0.0, alt_ = 3000.0, vs_ = 0.0;
  double fpaIntegral_ = 0.0;
  double gsIntegral_ = 0.0;
  double modeTimeS_ = 0.0;  // time in G/S*
  double speedIntegral_ = 0.0;
  bool athrWasActive_ = false;
  bool aFloor_ = false, togaLock_ = false;
  uint32_t disconnects_ = 0;
};

const char* latModeName(int mode);
const char* vertModeName(int mode);
const char* athrModeName(int mode);

}  // namespace a320
