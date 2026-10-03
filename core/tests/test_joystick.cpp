#include "Check.h"
#include "a320/JoystickMapping.h"

using namespace a320::joy;

TEST(joystick_axis_math) {
  CHECK_NEAR(normalize(0, 0, 65535), -1.0, 1e-9);
  CHECK_NEAR(normalize(65535, 0, 65535), 1.0, 1e-9);
  CHECK_NEAR(normalize(32767, 0, 65535), 0.0, 1e-4);
  CHECK_NEAR(normalize(5, 10, 10), 0.0, 1e-9);
  CHECK_NEAR(applyDeadzone(0.04, 0.06), 0.0, 1e-9);
  CHECK_NEAR(applyDeadzone(1.0, 0.06), 1.0, 1e-9);
  CHECK_NEAR(applyDeadzone(-0.53, 0.06), -0.5, 1e-9);
  // Throttle: full forward (inverted to +1) is TOGA; detents snap.
  const ThrottleCal linear;
  CHECK_NEAR(throttleFromAxis(1.0, linear).lever, 1.0, 1e-9);
  CHECK_NEAR(throttleFromAxis(-1.0, linear).lever, 0.0, 1e-9);
  CHECK_NEAR(throttleFromAxis(0.51, linear).lever, 0.75, 1e-9);
  CHECK_NEAR(throttleFromAxis(0.2, linear).lever, 0.6, 1e-9);
  CHECK(!throttleFromAxis(-1.0, linear).reverse);
  CHECK_NEAR(brakeAmount(-1.0), 0.0, 1e-9);
  CHECK_NEAR(brakeAmount(1.0), 1.0, 1e-9);
}

TEST(joystick_bindings_and_learn) {
  std::vector<AxisValues> devices(2);
  devices[0] = {0.1, -0.8, 0.3, 0.0, 0.0, 0.0};
  devices[1] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.9};
  const Config c = defaults();
  CHECK_NEAR(axisValue(c.bind[kPitch], devices), -0.8, 1e-9);
  CHECK_NEAR(axisValue(c.bind[kThrottle], devices), -0.3, 1e-9);  // inverted
  CHECK_NEAR(axisValue(c.bind[kBrakeLeft], devices), 0.0, 1e-9);  // unbound
  Binding missing{5, 0, false, {}};
  CHECK_NEAR(axisValue(missing, devices), 0.0, 1e-9);

  std::vector<AxisValues> later = devices;
  later[1][4] = 0.95;  // rudder pedals' U axis pushed
  int dev = -1, axis = -1;
  CHECK(detectAxis(devices, later, dev, axis));
  CHECK(dev == 1 && axis == 4);
  CHECK(!detectAxis(devices, devices, dev, axis));
}

TEST(joystick_config_roundtrip) {
  Config c = defaults();
  c.bind[kRudder] = {1, 4, true, {}};
  c.deadzone = 0.1;
  c.buttons[7] = "VIEW";
  Config back;
  CHECK(fromText(toText(c), back));
  CHECK(back.bind[kRudder].device == 1 && back.bind[kRudder].axis == 4 && back.bind[kRudder].invert);
  CHECK(back.bind[kPitch].axis == 1);
  CHECK_NEAR(back.deadzone, 0.1, 1e-9);
  CHECK(back.buttons[0] == "AP_DISCONNECT");
  CHECK(back.buttons[7] == "VIEW");
  Config spaced;
  CHECK(fromText("  rudder = 1:4:1 \r\nbutton1 = GEAR\n", spaced));
  CHECK(spaced.bind[kRudder].device == 1 && spaced.bind[kRudder].invert);
  CHECK(spaced.buttons[0] == "GEAR");
  Config untouched = defaults();
  CHECK(!fromText("# nothing here\nfoo=bar\n", untouched));
  CHECK(untouched.bind[kPitch].axis == 1);
}

TEST(throttle_detent_calibration) {
  // A quadrant whose axis runs backwards: REV -> TOGA reads 0.9 -> -0.9.
  const double raw[kCalSteps] = {0.5, -0.1, -0.4, -0.9, 0.9};
  ThrottleCal cal;
  bool flip = false;
  std::string error;
  CHECK(buildCalibration(raw, true, cal, flip, error));
  CHECK(flip);
  CHECK(cal.hasReverse);
  // Values after the (now toggled) invert are the negated raw readings.
  CHECK_NEAR(throttleFromAxis(-0.5, cal).lever, 0.0, 1e-9);
  CHECK_NEAR(throttleFromAxis(0.1, cal).lever, 0.75, 1e-9);
  CHECK_NEAR(throttleFromAxis(0.4, cal).lever, 0.88, 1e-9);
  CHECK_NEAR(throttleFromAxis(0.9, cal).lever, 1.0, 1e-9);
  CHECK_NEAR(throttleFromAxis(-0.1, cal).lever, 0.5, 0.02);  // between IDLE and CL: manual
  CHECK_NEAR(throttleFromAxis(0.1 + 0.005, cal).lever, 0.75, 1e-9);  // snaps into CL
  CHECK(std::string(leverDetentName(throttleFromAxis(0.105, cal).lever)) == "CL");
  // Reverse: a dead band just behind IDLE, then up to full reverse.
  CHECK(!throttleFromAxis(-0.55, cal).reverse);
  const LeverPosition rev = throttleFromAxis(-0.9, cal);
  CHECK(rev.reverse && rev.lever == 1.0);
  const LeverPosition revIdle = throttleFromAxis(-0.57, cal);
  CHECK(revIdle.reverse && revIdle.lever < 0.1);

  const double outOfOrder[kCalSteps] = {-1.0, 0.5, 0.3, 1.0, 0.0};
  CHECK(!buildCalibration(outOfOrder, false, cal, flip, error));
  CHECK(!error.empty());
  const double noRoomForReverse[kCalSteps] = {-1.0, 0.5, 0.76, 1.0, -0.98};
  CHECK(!buildCalibration(noRoomForReverse, true, cal, flip, error));
  CHECK(buildCalibration(noRoomForReverse, false, cal, flip, error));
  CHECK(!flip && !cal.hasReverse);
}

TEST(joystick_devices_by_name) {
  Config c = defaults();
  CHECK(resolveDevices(c, {"Logitech Extreme 3D"}));  // names adopted
  CHECK(c.bind[kPitch].deviceName == "Logitech Extreme 3D");
  CHECK(!resolveDevices(c, {"Logitech Extreme 3D"}));
  // Another device plugged in ahead of the stick: bindings follow the stick.
  CHECK(resolveDevices(c, {"Pedals", "Logitech Extreme 3D"}));
  CHECK(c.bind[kPitch].device == 1 && c.bind[kRoll].device == 1);
  // Stick unplugged: unbound until it is back.
  CHECK(resolveDevices(c, {"Pedals"}));
  CHECK(c.bind[kPitch].device == -1 && c.bind[kPitch].deviceName == "Logitech Extreme 3D");
  CHECK(resolveDevices(c, {"Logitech Extreme 3D"}));
  CHECK(c.bind[kPitch].device == 0);
}

TEST(throttle_quadrant_auto_assign) {
  std::array<bool, kAxes> stickAxes = {true, true, true, true, false, false};
  std::array<bool, kAxes> quadAxes = {true, true, false, false, false, false};
  // The quadrant enumerates first, so the defaults point the stick functions at it.
  Config c = defaults();
  const std::vector<std::string> names = {"TCA Q-Eng 1&2", "TCA Sidestick Airbus"};
  CHECK(!autoAssignThrottle(c, {"TCA Sidestick Airbus"}, {stickAxes}));  // no quadrant yet
  CHECK(c.autoThrottle);
  CHECK(autoAssignThrottle(c, names, {quadAxes, stickAxes}));
  CHECK(c.bind[kThrottle].device == 0 && c.bind[kThrottle].axis == 0);
  CHECK(c.bind[kThrottle2].device == 0 && c.bind[kThrottle2].axis == 1);
  CHECK(c.bind[kPitch].device == 1 && c.bind[kRoll].device == 1 && c.bind[kRudder].device == 1);
  CHECK(!c.autoThrottle);
  CHECK(!autoAssignThrottle(c, names, {quadAxes, stickAxes}));  // only once
  CHECK(isThrottleDevice("Saitek Pro Flight Throttle Quadrant") && !isThrottleDevice("Logitech Extreme 3D"));
}

TEST(joystick_config_throttle_roundtrip) {
  Config c = defaults();
  c.bind[kThrottle2] = {1, 1, true, "TCA Q-Eng 1&2"};
  c.cal[1] = ThrottleCal{-0.8, 0.1, 0.4, 0.9, true, -1.0};
  c.throttleButtons[2] = "ENG1_MASTER";
  c.autoThrottle = false;
  Config back;
  CHECK(fromText(toText(c), back));
  CHECK(back.bind[kThrottle2].device == 1 && back.bind[kThrottle2].invert && back.bind[kThrottle2].deviceName == "TCA Q-Eng 1&2");
  CHECK(back.cal[1].hasReverse && back.cal[1].climb == 0.1 && back.cal[1].reverseMax == -1.0);
  CHECK(!back.cal[0].hasReverse && back.cal[0].toga == 1.0);
  CHECK(back.throttleButtons[2] == "ENG1_MASTER");
  CHECK(!back.autoThrottle);
  // A settings file from before THRUST 2 existed.
  Config old;
  CHECK(fromText("pitch=0:1:0\nthrottle=0:2:1\n", old));
  CHECK(old.bind[kThrottle2].device == -1 && old.autoThrottle);
}
