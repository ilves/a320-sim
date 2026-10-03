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
  CHECK_NEAR(throttleLever(1.0), 1.0, 1e-9);
  CHECK_NEAR(throttleLever(-1.0), 0.0, 1e-9);
  CHECK_NEAR(throttleLever(0.51), 0.75, 1e-9);
  CHECK_NEAR(throttleLever(0.2), 0.6, 1e-9);
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
  Binding missing{5, 0, false};
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
  c.bind[kRudder] = {1, 4, true};
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
