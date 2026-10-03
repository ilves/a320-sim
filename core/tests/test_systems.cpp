#include "Check.h"
#include "a320/FlyByWire.h"
#include "a320/SimClock.h"
#include "a320/Systems.h"

using namespace a320;

TEST(clock_fixed_steps_pause_and_rate) {
  SimClock c(0.01, 0.25);
  CHECK(c.advance(0.035) == 3);
  CHECK(c.advance(0.005) == 1);  // carries the 5 ms remainder
  c.setPaused(true);
  CHECK(c.advance(1.0) == 0);
  CHECK_NEAR(c.simTimeS(), 0.04, 1e-9);
  c.setPaused(false);
  c.setRate(2.0);
  CHECK(c.advance(0.05) == 10);
  // A one-second hitch is capped instead of replayed.
  c.setRate(1.0);
  const int capped = c.advance(1.0);
  CHECK(capped >= 24 && capped <= 25);
}

TEST(flaps_one_plus_f) {
  FlapsSystem f;
  f.setLever(1, 0.0);
  CHECK(f.onePlusF());
  CHECK_NEAR(f.flapTargetDeg(), 10.0, 1e-9);
  f.update(215.0);  // auto-retract above 210 kt
  CHECK(!f.onePlusF());
  CHECK_NEAR(f.flapTargetDeg(), 0.0, 1e-9);

  FlapsSystem g;
  g.setLever(1, 250.0);  // selected in flight from 0: CONF 1, flaps stay up
  CHECK(!g.onePlusF());
  g.setLever(4, 150.0);
  CHECK_NEAR(g.flapTargetDeg(), 35.0, 1e-9);
  CHECK(std::string(flapConfigName(4, false)) == "FULL");
}

TEST(thrust_detents) {
  CHECK(thrustDetent(0.0) == ThrustDetent::Idle);
  CHECK(thrustDetent(0.75) == ThrustDetent::Climb);
  CHECK(thrustDetent(0.88) == ThrustDetent::FlexMct);
  CHECK(thrustDetent(1.0) == ThrustDetent::Toga);
  CHECK(thrustDetent(0.5) == ThrustDetent::Manual);
}

TEST(speed_limits) {
  // 64 t landing weight, CONF FULL: VLS in the 140s with this model's CLmax.
  const SpeedLimits full = computeSpeedLimits(4, false, 35.0, 141000.0, true, false);
  CHECK(full.vlsKt > 135.0 && full.vlsKt < 155.0);
  CHECK_NEAR(full.vmaxKt, 177.0, 1e-9);
  CHECK_NEAR(full.vlsKt / full.vsKt, 1.23, 1e-9);
  const SpeedLimits clean = computeSpeedLimits(0, false, 0.0, 141000.0, false, false);
  CHECK(clean.vlsKt > full.vlsKt);
  CHECK_NEAR(clean.vmaxKt, 350.0, 1e-9);
  // Gear down caps VMAX at VLE.
  CHECK_NEAR(computeSpeedLimits(0, false, 0.0, 141000.0, true, false).vmaxKt, 280.0, 1e-9);
}

TEST(warnings) {
  WarningInput t;
  t.onGround = true;
  t.thrustLever = 1.0;
  t.flapsLever = 0;
  t.parkBrake = true;
  const uint32_t w = computeWarnings(t);
  CHECK(w & kWarnConfigFlaps);
  CHECK(w & kWarnConfigParkBrk);

  WarningInput a;
  a.onGround = false;
  a.gearDown = false;
  a.radioAltFt = 600.0;
  a.flapsLever = 3;
  a.thrustLever = 0.5;
  a.iasKt = 160.0;
  CHECK(computeWarnings(a) & kWarnGearNotDown);
  a.gearDown = true;
  a.gsValid = true;
  a.gsDots = 1.6;
  CHECK(computeWarnings(a) == kWarnGlideslope);
}

TEST(callouts_and_retard) {
  Callouts c;
  CHECK(c.update(2600.0, false, 0.5) == nullptr);
  CHECK(std::string(c.update(2490.0, false, 0.5)) == "TWO THOUSAND FIVE HUNDRED");
  CHECK(c.update(2480.0, false, 0.5) == nullptr);
  c.update(60.0, false, 0.5);
  CHECK(std::string(c.update(49.0, false, 0.5)) == "FIFTY");
  c.update(25.0, false, 0.5);
  CHECK(std::string(c.update(19.0, false, 0.5)) == "RETARD");
  // Climbing never calls out.
  CHECK(c.update(30.0, false, 0.5) == nullptr);
}

TEST(fbw_ground_law_is_direct) {
  FlyByWire f;
  FbwInput in;
  in.onGround = true;
  in.stickPitch = 1.0;
  const FbwOutput out = f.update(in);
  CHECK(out.law == PitchLaw::Ground);
  CHECK_NEAR(out.elevatorCmd, -1.0, 1e-9);
}
