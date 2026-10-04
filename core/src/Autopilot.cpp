#include "a320/Autopilot.h"

#include <cmath>

#include "a320/FlyByWire.h"
#include "a320/Systems.h"
#include "a320/Units.h"

namespace a320 {
namespace {

double wrap180(double deg) {
  deg = std::fmod(deg, 360.0);
  if (deg > 180.0) deg -= 360.0;
  if (deg < -180.0) deg += 360.0;
  return deg;
}

double wrap360(double deg) {
  deg = std::fmod(deg, 360.0);
  return deg < 0.0 ? deg + 360.0 : deg;
}

// Vertical speed to flight path angle at the given true airspeed.
double fpaForVs(double vsFpm, double tasKt) {
  const double tasFpm = std::fmax(tasKt, 60.0) * kKtToMps * kMToFt * 60.0;
  return std::asin(clamp(vsFpm / tasFpm, -0.5, 0.5)) * kRadToDeg;
}

constexpr double kTakeoverStick = 0.5;
constexpr double kLandModeFt = 400.0;
constexpr double kFlareFt = 40.0;
constexpr double kRetardFt = 20.0;

}  // namespace

void Autopilot::reset(double spdKt, double hdgMagDeg, double altFt) {
  ap1_ = ap2_ = athr_ = false;
  lat_ = A320_LAT_NONE;
  vert_ = A320_VERT_NONE;
  athrMode_ = A320_ATHR_OFF;
  locArmed_ = gsArmed_ = navArmed_ = false;
  spd_ = spdKt;
  hdg_ = wrap360(hdgMagDeg);
  alt_ = altFt;
  vs_ = 0.0;
  fpa_ = 0.0;
  trkFpa_ = false;
  athrWasActive_ = false;
  aFloor_ = togaLock_ = false;
}

int Autopilot::armed() const {
  int bits = 0;
  if (vert_ == A320_VERT_VS || vert_ == A320_VERT_FPA || vert_ == A320_VERT_OP_CLB || vert_ == A320_VERT_OP_DES) bits |= A320_ARMED_ALT;
  if (locArmed_) bits |= A320_ARMED_LOC;
  if (navArmed_) bits |= A320_ARMED_NAV;
  if (gsArmed_) bits |= A320_ARMED_GS;
  return bits;
}

void Autopilot::setTargets(double spdKt, double hdgMagDeg, double altFt, double vsFpm) {
  spd_ = clamp(std::round(spdKt), 100.0, 350.0);
  hdg_ = wrap360(std::round(hdgMagDeg));
  alt_ = clamp(std::round(altFt / 100.0) * 100.0, 100.0, 39000.0);
  vs_ = clamp(std::round(vsFpm / 100.0) * 100.0, -6000.0, 6000.0);
}

void Autopilot::setFpa(double fpaDeg) { fpa_ = clamp(std::round(fpaDeg * 10.0) / 10.0, -9.9, 9.9); }

double Autopilot::presentDirection(const ApInput& in) const {
  return wrap360(std::round((trkFpa_ ? in.trackTrueDeg : in.headingTrueDeg) - in.magneticVariationDeg));
}

void Autopilot::syncVerticalTargets(const ApInput& in) {
  vs_ = clamp(std::round(in.verticalSpeedFpm / 100.0) * 100.0, -6000.0, 6000.0);
  setFpa(in.flightPathDeg);
}

void Autopilot::engageCruise() {
  ap1_ = true;
  athr_ = true;
  lat_ = A320_LAT_HDG;
  vert_ = A320_VERT_ALT;
}

void Autopilot::disconnectAp() {
  if (ap1_ || ap2_) ++disconnects_;
  ap1_ = ap2_ = false;
}

bool Autopilot::approachMode() const {
  return locArmed_ || gsArmed_ || lat_ == A320_LAT_LOC_STAR || lat_ == A320_LAT_LOC || lat_ == A320_LAT_ROLLOUT ||
         vert_ == A320_VERT_GS_STAR || vert_ == A320_VERT_GS || vert_ == A320_VERT_LAND || vert_ == A320_VERT_FLARE;
}

const char* Autopilot::engageAp(bool& self, bool& other, const ApInput& in) {
  if (self) {
    self = false;
    if (!other) ++disconnects_;  // the last autopilot off: cavalry charge
    return nullptr;
  }
  if (in.onGround || in.radioAltFt <= 100.0)
    return "AP: can't engage on the ground. It is available from 100 ft radio altitude after take-off.";
  const char* hint = nullptr;
  if (other && !approachMode()) {
    // Both autopilots only for the approach (CAT 3 DUAL); otherwise the new one takes over.
    other = false;
    hint = "AP: both autopilots engage only with LOC or APPR armed. The other AP disengaged.";
  }
  const bool wasEngaged = other;
  self = true;
  if (!wasEngaged) {
    // With no mode selected the AP engages in HDG and V/S (TRK and FPA) on the present values.
    if (lat_ == A320_LAT_NONE || lat_ == A320_LAT_ROLLOUT) {
      lat_ = selectedLateral();
      hdg_ = presentDirection(in);
    }
    if (vert_ == A320_VERT_NONE || vert_ == A320_VERT_FLARE) {
      syncVerticalTargets(in);
      enterVertical(selectedVertical(), in);
    }
  }
  return hint;
}

void Autopilot::enterVertical(A320VertMode mode, const ApInput& in) {
  if (mode == A320_VERT_GS_STAR) gsIntegral_ = 0.0;
  vert_ = mode;
  fpaIntegral_ = in.flightPathDeg;
  if (mode != A320_VERT_LAND) pathTrim_ = 0.0;  // LAND keeps the glideslope's trim, without a step
}

const char* Autopilot::command(A320FcuCommand cmd, const ApInput& in) {
  const bool landing = vert_ == A320_VERT_LAND || vert_ == A320_VERT_FLARE || lat_ == A320_LAT_ROLLOUT;
  const bool onGs = vert_ == A320_VERT_GS_STAR || vert_ == A320_VERT_GS;
  static const char* kLandLocked =
      "LAND mode (below 400 ft): the autoland can no longer be changed. Disconnect the AP or go around.";
  switch (cmd) {
    case A320_FCU_AP1:
      return engageAp(ap1_, ap2_, in);
    case A320_FCU_AP2:
      return engageAp(ap2_, ap1_, in);
    case A320_FCU_ATHR:
      if (aFloor_ || togaLock_) {
        // The only way out of alpha floor / TOGA LK: disconnect the autothrust.
        aFloor_ = togaLock_ = false;
        athr_ = false;
        return "A/THR disconnected: thrust follows the thrust levers again. Set them where you need thrust.";
      }
      athr_ = !athr_;
      if (athr_ && athrMode_ == A320_ATHR_OFF) spd_ = std::round(in.iasKt);
      if (athr_ && !in.onGround && (in.thrustLever > kLeverClimb + 0.03 || in.thrustLever <= 0.02))
        return "A/THR armed (blue on the FMA): it becomes active when the thrust levers are in the CL detent.";
      return nullptr;
    case A320_FCU_HDG_PUSH:
      if (landing) return kLandLocked;
      if (in.navValid) {
        // NAV: managed lateral guidance along the flight plan, armed on the ground.
        if (lat_ == A320_LAT_LOC || lat_ == A320_LAT_LOC_STAR)
          return "NAV: the localizer is engaged. Push LOC or APPR to leave the approach first.";
        if (in.onGround) navArmed_ = true;
        else lat_ = A320_LAT_NAV;
        return nullptr;
      }
      [[fallthrough]];
    case A320_FCU_HDG_PULL:
      if (landing) return kLandLocked;
      navArmed_ = false;
      lat_ = selectedLateral();
      locArmed_ = gsArmed_ = false;
      if (onGs) {
        syncVerticalTargets(in);
        enterVertical(selectedVertical(), in);
      }
      if (cmd == A320_FCU_HDG_PULL) return nullptr;
      hdg_ = presentDirection(in);
      return trkFpa_ ? "HDG pushed: no flight plan to navigate (NAV), so the autopilot holds the present track, "
                       "wings level. Choose departure and arrival on the FLIGHT menu or the MCDU for a route."
                     : "HDG pushed: no flight plan to navigate (NAV), so the autopilot holds the present heading, "
                       "wings level. Choose departure and arrival on the FLIGHT menu or the MCDU for a route.";
    case A320_FCU_TRK_FPA: {
      trkFpa_ = !trkFpa_;
      // An engaged HDG or V/S becomes TRK or FPA on the present values; a preselected
      // direction keeps pointing the same way over the ground (drift angle added or taken off).
      const double drift = wrap180(in.trackTrueDeg - in.headingTrueDeg);
      if (lat_ == A320_LAT_HDG || lat_ == A320_LAT_TRK) {
        lat_ = selectedLateral();
        hdg_ = presentDirection(in);
      } else if (!in.onGround) {
        hdg_ = wrap360(std::round(hdg_ + (trkFpa_ ? drift : -drift)));
      }
      if (vert_ == A320_VERT_VS || vert_ == A320_VERT_FPA) {
        syncVerticalTargets(in);
        enterVertical(selectedVertical(), in);
      }
      return nullptr;
    }
    case A320_FCU_LOC:
      if (landing) return kLandLocked;
      if (lat_ == A320_LAT_LOC || lat_ == A320_LAT_LOC_STAR) return nullptr;
      locArmed_ = !locArmed_;
      gsArmed_ = false;
      return nullptr;
    case A320_FCU_APPR: {
      if (landing) return kLandLocked;
      const bool armedNow = locArmed_ || gsArmed_ || onGs;
      if (armedNow) {
        locArmed_ = gsArmed_ = false;
        if (onGs) enterVertical(selectedVertical(), in);
        return nullptr;
      }
      if (in.onGround || in.radioAltFt < 400.0) return "APPR: can't be armed below 400 ft radio altitude.";
      locArmed_ = lat_ != A320_LAT_LOC && lat_ != A320_LAT_LOC_STAR;
      gsArmed_ = true;
      if (!in.locValid)
        return "APPR armed (LOC and G/S in blue on the FMA). No ILS signal yet: it captures once the "
               "localizer is alive (press LS to see the scales).";
      return nullptr;
    }
    case A320_FCU_ALT_PULL: {
      if (landing) return kLandLocked;
      if (onGs) return "ALT: no open climb or descent on the glideslope. Push APPR first to leave the approach.";
      const double err = alt_ - in.altitudeFt;
      if (std::fabs(err) < 50.0) enterVertical(A320_VERT_ALT, in);
      else enterVertical(err > 0.0 ? A320_VERT_OP_CLB : A320_VERT_OP_DES, in);
      return nullptr;
    }
    case A320_FCU_ALT_PUSH: {
      if (landing) return kLandLocked;
      if (onGs) return "ALT: no level-off on the glideslope. Push APPR first to leave the approach.";
      alt_ = clamp(std::round(in.altitudeFt / 100.0) * 100.0, 100.0, 39000.0);
      enterVertical(std::fabs(alt_ - in.altitudeFt) < 20.0 ? A320_VERT_ALT : A320_VERT_ALT_STAR, in);
      hint_ = "ALT pushed: managed climb and descent are not simulated, so the autopilot levels off at the present "
              "altitude, " + std::to_string(static_cast<int>(alt_)) + " ft. Set a new altitude and pull the knob to go on.";
      return hint_.c_str();
    }
    case A320_FCU_VS_PULL:
      if (landing) return kLandLocked;
      // An armed G/S stays armed: a V/S descent onto the beam is the standard capture from above.
      // Leaving an engaged G/S does not re-arm it.
      syncVerticalTargets(in);
      enterVertical(selectedVertical(), in);
      return nullptr;
    case A320_FCU_VS_PUSH:
      if (landing) return kLandLocked;
      vs_ = 0.0;
      fpa_ = 0.0;
      enterVertical(selectedVertical(), in);
      return nullptr;
  }
  return nullptr;
}

void Autopilot::updateModes(const ApInput& in) {
  // NAV engages at 30 ft after takeoff; without a leg to fly it reverts to holding the heading.
  if (navArmed_ && in.navValid && !in.onGround && in.radioAltFt > 30.0) {
    lat_ = A320_LAT_NAV;
    navArmed_ = false;
  }
  if (!in.navValid) navArmed_ = false;
  if (lat_ == A320_LAT_NAV && !in.navValid) {
    lat_ = selectedLateral();
    hdg_ = presentDirection(in);
  }
  // Localizer capture: within 1.8 dots and converging, then track once nearly centred.
  if (locArmed_ && in.locValid && std::fabs(in.locDots) < 1.8) {
    lat_ = A320_LAT_LOC_STAR;
    locArmed_ = false;
  }
  if (lat_ == A320_LAT_LOC_STAR && std::fabs(in.locDots) < 0.2 &&
      std::fabs(wrap180(in.trackTrueDeg - in.ilsCourseTrueDeg)) < 10.0)
    lat_ = A320_LAT_LOC;

  // Glideslope capture needs the localizer first, as on the real aircraft.
  const bool onLoc = lat_ == A320_LAT_LOC || lat_ == A320_LAT_LOC_STAR;
  if (gsArmed_ && onLoc && in.gsValid && std::fabs(in.gsDots) < 0.4) {
    enterVertical(A320_VERT_GS_STAR, in);
    gsArmed_ = false;
  }
  // Capture ends once the beam is held: nearly centred, or after the transition has settled.
  modeTimeS_ = vert_ == A320_VERT_GS_STAR ? modeTimeS_ + in.dtS : 0.0;
  if (vert_ == A320_VERT_GS_STAR && (std::fabs(in.gsDots) < 0.1 || modeTimeS_ > 15.0)) vert_ = A320_VERT_GS;
  if ((vert_ == A320_VERT_GS || vert_ == A320_VERT_GS_STAR) && lat_ == A320_LAT_LOC && in.radioAltFt < kLandModeFt)
    enterVertical(A320_VERT_LAND, in);
  if (vert_ == A320_VERT_LAND && in.radioAltFt < kFlareFt) enterVertical(A320_VERT_FLARE, in);
  if ((vert_ == A320_VERT_FLARE || vert_ == A320_VERT_LAND) && in.onGround) {
    lat_ = A320_LAT_ROLLOUT;
    vert_ = A320_VERT_NONE;
  }

  // Altitude capture is always armed in the climb/descent modes.
  const double altErr = alt_ - in.altitudeFt;
  const bool towards = (vert_ == A320_VERT_OP_CLB && altErr > 0.0) || (vert_ == A320_VERT_OP_DES && altErr < 0.0) ||
                       ((vert_ == A320_VERT_VS || vert_ == A320_VERT_FPA) && altErr * in.verticalSpeedFpm > 0.0);
  if (towards && std::fabs(altErr) < std::fmax(100.0, std::fabs(in.verticalSpeedFpm) * 0.15))
    enterVertical(A320_VERT_ALT_STAR, in);
  if (vert_ == A320_VERT_ALT_STAR && std::fabs(altErr) < 20.0) enterVertical(A320_VERT_ALT, in);
}

double Autopilot::lateralBank(const ApInput& in) {
  switch (lat_) {
    case A320_LAT_HDG: {
      const double hdgTrue = hdg_ + in.magneticVariationDeg;
      return clamp(2.5 * wrap180(hdgTrue - in.headingTrueDeg), -25.0, 25.0);
    }
    case A320_LAT_NAV:
      return clamp(2.5 * wrap180(in.navTrackTrueDeg - in.trackTrueDeg), -25.0, 25.0);
    case A320_LAT_TRK: {
      // The track over the ground: the wind correction angle comes by itself.
      const double trkTrue = hdg_ + in.magneticVariationDeg;
      return clamp(2.5 * wrap180(trkTrue - in.trackTrueDeg), -25.0, 25.0);
    }
    case A320_LAT_LOC_STAR:
    case A320_LAT_LOC: {
      // Aim the track at the localizer: up to 25 degrees of intercept, proportional to dots.
      const double trackCmd = in.ilsCourseTrueDeg + clamp(12.0 * in.locDots, -25.0, 25.0);
      const double maxBank = lat_ == A320_LAT_LOC ? 15.0 : 25.0;
      return clamp(2.5 * wrap180(trackCmd - in.trackTrueDeg), -maxBank, maxBank);
    }
    default:
      return 0.0;
  }
}

double Autopilot::verticalFpa(const ApInput& in) {
  const double altErr = alt_ - in.altitudeFt;
  switch (vert_) {
    case A320_VERT_ALT:
      return fpaForVs(clamp(altErr * 4.0, -1000.0, 1000.0), in.tasKt);
    case A320_VERT_ALT_STAR:
      return fpaForVs(clamp(altErr * 4.0, -std::fmax(std::fabs(in.verticalSpeedFpm), 300.0),
                            std::fmax(std::fabs(in.verticalSpeedFpm), 300.0)), in.tasKt);
    case A320_VERT_VS:
      return fpaForVs(vs_, in.tasKt);
    case A320_VERT_FPA:
      return fpa_;
    case A320_VERT_OP_CLB:
    case A320_VERT_OP_DES: {
      // Speed on pitch: faster than target -> pitch up.
      const double lo = vert_ == A320_VERT_OP_CLB ? 0.5 : -8.0;
      const double hi = vert_ == A320_VERT_OP_CLB ? 15.0 : 0.0;
      const double err = in.iasKt - protectedSpeed(in);
      fpaIntegral_ = clamp(fpaIntegral_ + 0.05 * err * in.dtS, lo, hi);
      return clamp(fpaIntegral_ + 0.3 * err, lo, hi);
    }
    case A320_VERT_GS_STAR:
    case A320_VERT_GS:
    case A320_VERT_LAND: {
      // Close to the antenna one dot is only a few feet, so LAND fades the beam corrections
      // out below 300 ft and flies the nominal path into the flare instead of chasing it.
      const double beamWeight = vert_ == A320_VERT_LAND ? clamp((in.radioAltFt - 80.0) / 220.0, 0.0, 1.0) : 1.0;
      if (vert_ == A320_VERT_GS || vert_ == A320_VERT_GS_STAR)
        gsIntegral_ = clamp(gsIntegral_ + 0.02 * in.gsDots * in.dtS, -0.5, 0.5);  // slow trim only
      return -in.glideslopeDeg + beamWeight * (clamp(1.3 * in.gsDots, -2.0, 2.0) + gsIntegral_);
    }
    default:
      return in.flightPathDeg;
  }
}

double Autopilot::protectedSpeed(const ApInput& in) const {
  const double lo = in.vlsKt, hi = std::fmax(in.vmaxKt - 3.0, lo);
  return lo > 0.0 ? clamp(spd_, lo, hi) : spd_;
}

double Autopilot::autothrust(const ApInput& in, bool& active) {
  if (aFloor_ || togaLock_) {
    athrMode_ = aFloor_ ? A320_ATHR_AFLOOR : A320_ATHR_TOGA_LK;
    active = true;
    athrWasActive_ = false;
    return 1.0;
  }
  // A/THR works with the levers between idle and CL; at TOGA/FLX the pilot has manual thrust.
  active = athr_ && in.thrustLever > 0.02 && in.thrustLever <= kLeverClimb + 0.03 && !in.onGround;
  if (vert_ == A320_VERT_FLARE && in.radioAltFt < kRetardFt) athrMode_ = A320_ATHR_RETARD;
  else if (vert_ == A320_VERT_OP_CLB) athrMode_ = A320_ATHR_THR_CLB;
  else if (vert_ == A320_VERT_OP_DES) athrMode_ = A320_ATHR_THR_IDLE;
  else athrMode_ = athr_ ? A320_ATHR_SPEED : A320_ATHR_OFF;
  if (!athr_) athrMode_ = A320_ATHR_OFF;
  if (!active) {
    athrWasActive_ = false;
    return in.thrustLever;
  }

  const double limit = std::fmin(in.thrustLever, kLeverClimb);
  const double err = protectedSpeed(in) - in.iasKt;
  if (!athrWasActive_) speedIntegral_ = in.currentThrottle - 0.03 * err;  // bumpless
  athrWasActive_ = true;
  switch (athrMode_) {
    case A320_ATHR_THR_CLB:
      return limit;
    case A320_ATHR_THR_IDLE:
    case A320_ATHR_RETARD:
      speedIntegral_ = 0.0;
      return 0.0;
    default:
      // Gentle on purpose: a fast thrust loop couples with the pitch loop on the glideslope.
      speedIntegral_ = clamp(speedIntegral_ + 0.004 * err * in.dtS, 0.0, limit);
      return clamp(speedIntegral_ + 0.03 * err, 0.0, limit);
  }
}

ApOutput Autopilot::update(const ApInput& in) {
  ApOutput out;
  // Instinctive disconnect: a firm sidestick input takes over from the autopilot.
  if (apEngaged() && (std::fabs(in.pilotStickPitch) > kTakeoverStick || std::fabs(in.pilotStickRoll) > kTakeoverStick))
    disconnectAp();
  // Touchdown with the levers at idle disconnects autothrust.
  if (athr_ && in.onGround && in.thrustLever < 0.02) athr_ = false;
  // Alpha floor: TOGA thrust from the autothrust (even if it was off) when the angle of attack
  // gets close to the stall, or with full back stick in alpha protection. Not near the ground.
  const bool floorCondition = !in.onGround && in.radioAltFt > 100.0 &&
                              (in.alphaDeg > kAlphaFloorDeg || (in.alphaDeg > kAlphaProtDeg && in.pilotStickPitch > 0.9));
  if (floorCondition) {
    aFloor_ = athr_ = true;
    togaLock_ = false;
  } else if (aFloor_ && in.alphaDeg < kAlphaFloorDeg - 0.5) {
    aFloor_ = false;
    togaLock_ = true;
  }
  if (in.onGround) aFloor_ = togaLock_ = false;

  updateModes(in);
  // Leaving the approach with both autopilots engaged keeps only one.
  if (ap1_ && ap2_ && !approachMode()) ap2_ = false;
  out.throttle = autothrust(in, out.athrActive);
  out.thrustOverride = aFloor_ || togaLock_;
  if (!apEngaged()) return out;

  out.apActive = true;
  if (vert_ == A320_VERT_FLARE) {
    // Autoland flare: ease back for about +3 degrees, more if still sinking fast.
    out.stickPitch = 0.3 + clamp(-0.0004 * (in.verticalSpeedFpm + 300.0), 0.0, 0.2);
  } else if (lat_ == A320_LAT_ROLLOUT) {
    out.stickPitch = 0.0;
  } else {
    // Flight path -> sidestick (load factor) order, kept below the FBW pitch loop's bandwidth:
    // higher gains or limits limit-cycle after a large upset such as a V/S 0 selection.
    const double fpaErr = verticalFpa(in) - in.flightPathDeg;
    // The FBW alone leaves a standing error of a few tenths of a degree (more as the speed
    // changes): the path modes trim it out. LAND keeps its own law into the flare.
    if (vert_ == A320_VERT_VS || vert_ == A320_VERT_FPA || vert_ == A320_VERT_GS_STAR || vert_ == A320_VERT_GS)
      pathTrim_ = clamp(pathTrim_ + 0.25 * clamp(fpaErr, -1.0, 1.0) * in.dtS, -1.5, 1.5);
    out.stickPitch = clamp(0.12 * (fpaErr + pathTrim_), -0.3, 0.3);
  }

  if (lat_ == A320_LAT_ROLLOUT || vert_ == A320_VERT_FLARE) {
    // Track the centreline on the localizer: dots -> metres at the current range.
    const double yM = -in.locDots * in.locDegPerDot * kDegToRad * in.locRangeNm * kNmToM;
    out.pedals = clamp(-0.03 * yM - 0.2 * wrap180(in.headingTrueDeg - in.ilsCourseTrueDeg), -1.0, 1.0);
    if (vert_ == A320_VERT_FLARE) out.pedals = clamp(out.pedals, -0.3, 0.3);
    out.stickRoll = lat_ == A320_LAT_ROLLOUT ? 0.0 : clamp(0.06 * (lateralBank(in) - in.bankDeg), -0.6, 0.6);
  } else {
    out.stickRoll = clamp(0.06 * (lateralBank(in) - in.bankDeg), -0.6, 0.6);
  }
  return out;
}

const char* latModeName(int mode) {
  switch (mode) {
    case A320_LAT_HDG: return "HDG";
    case A320_LAT_LOC_STAR: return "LOC*";
    case A320_LAT_LOC: return "LOC";
    case A320_LAT_ROLLOUT: return "ROLL OUT";
    case A320_LAT_TRK: return "TRK";
    case A320_LAT_NAV: return "NAV";
    default: return "";
  }
}

const char* vertModeName(int mode) {
  switch (mode) {
    case A320_VERT_ALT: return "ALT";
    case A320_VERT_ALT_STAR: return "ALT*";
    case A320_VERT_VS: return "V/S";
    case A320_VERT_OP_CLB: return "OP CLB";
    case A320_VERT_OP_DES: return "OP DES";
    case A320_VERT_GS: return "G/S";
    case A320_VERT_GS_STAR: return "G/S*";
    case A320_VERT_LAND: return "LAND";
    case A320_VERT_FLARE: return "FLARE";
    case A320_VERT_FPA: return "FPA";
    default: return "";
  }
}

const char* athrModeName(int mode) {
  switch (mode) {
    case A320_ATHR_SPEED: return "SPEED";
    case A320_ATHR_THR_CLB: return "THR CLB";
    case A320_ATHR_THR_IDLE: return "THR IDLE";
    case A320_ATHR_RETARD: return "RETARD";
    case A320_ATHR_AFLOOR: return "A.FLOOR";
    case A320_ATHR_TOGA_LK: return "TOGA LK";
    default: return "";
  }
}

}  // namespace a320
