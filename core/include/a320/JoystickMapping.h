/* Joystick axis/button mapping (header-only; used by the Unreal front end and the tests). */
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

namespace a320 {
namespace joy {

enum Function { kPitch = 0, kRoll, kRudder, kThrottle, kBrakeLeft, kBrakeRight, kFunctionCount };
constexpr int kAxes = 6;      // X Y Z R U V, as Windows reports them
constexpr int kButtons = 32;
using AxisValues = std::array<double, kAxes>;  // -1..1 per axis of one device

struct Binding {
  int device = -1;  // -1 = not bound
  int axis = 0;
  bool invert = false;
};

struct Config {
  Binding bind[kFunctionCount];
  double deadzone = 0.06;
  std::string buttons[kButtons];  // command name per button (1-based in the file), empty = none
};

inline const char* functionName(int f) {
  static const char* kNames[kFunctionCount] = {"PITCH", "ROLL", "RUDDER", "THROTTLE", "BRAKE L", "BRAKE R"};
  return f >= 0 && f < kFunctionCount ? kNames[f] : "";
}

inline const char* axisName(int a) {
  static const char* kNames[kAxes] = {"X", "Y", "Z", "R", "U", "V"};
  return a >= 0 && a < kAxes ? kNames[a] : "?";
}

// Typical stick: X roll, Y pitch (forward = negative = nose down), Z throttle slider (forward is
// the low end, hence inverted), R twist rudder.
inline Config defaults() {
  Config c;
  c.bind[kPitch] = {0, 1, false};
  c.bind[kRoll] = {0, 0, false};
  c.bind[kRudder] = {0, 3, false};
  c.bind[kThrottle] = {0, 2, true};
  c.buttons[0] = "AP_DISCONNECT";
  c.buttons[1] = "BRAKES";
  c.buttons[2] = "FLAPS_UP";
  c.buttons[3] = "FLAPS_DOWN";
  c.buttons[4] = "GEAR";
  c.buttons[5] = "REVERSE";
  return c;
}

inline double normalize(uint32_t raw, uint32_t lo, uint32_t hi) {
  if (hi <= lo) return 0.0;
  const double v = 2.0 * (static_cast<double>(raw) - lo) / (static_cast<double>(hi) - lo) - 1.0;
  return v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v);
}

inline double applyDeadzone(double v, double dz) {
  if (std::fabs(v) < dz) return 0.0;
  return (v - std::copysign(dz, v)) / (1.0 - dz);
}

// Value of a bound function, -1..1 (inverted if configured); 0 when unbound or absent.
inline double axisValue(const Binding& b, const std::vector<AxisValues>& devices) {
  if (b.device < 0 || b.device >= static_cast<int>(devices.size()) || b.axis < 0 || b.axis >= kAxes) return 0.0;
  const double v = devices[static_cast<size_t>(b.device)][static_cast<size_t>(b.axis)];
  return b.invert ? -v : v;
}

// Throttle axis (-1..1) to thrust lever (0..1), snapping into the A320 detents.
inline double throttleLever(double v) {
  double lever = (v + 1.0) / 2.0;
  for (double detent : {0.0, 0.75, 0.88, 1.0}) {
    if (std::fabs(lever - detent) < 0.03) lever = detent;
  }
  return lever;
}

// Toe brake axis (-1 released .. 1 pressed) to 0..1.
inline double brakeAmount(double v) { return v <= -0.95 ? 0.0 : (v + 1.0) / 2.0; }

// Learn mode: the axis that moved furthest from where it was when learning started.
inline bool detectAxis(const std::vector<AxisValues>& baseline, const std::vector<AxisValues>& now, int& device, int& axis) {
  double best = 0.5;
  bool found = false;
  for (size_t d = 0; d < baseline.size() && d < now.size(); ++d) {
    for (int a = 0; a < kAxes; ++a) {
      const double moved = std::fabs(now[d][static_cast<size_t>(a)] - baseline[d][static_cast<size_t>(a)]);
      if (moved > best) {
        best = moved;
        device = static_cast<int>(d);
        axis = a;
        found = true;
      }
    }
  }
  return found;
}

inline std::string toText(const Config& c) {
  std::ostringstream out;
  out << "# A320 Sim joystick settings: <function>=<device>:<axis 0-5 = X Y Z R U V>:<invert 0/1>\n";
  static const char* kKeys[kFunctionCount] = {"pitch", "roll", "rudder", "throttle", "brakeLeft", "brakeRight"};
  for (int f = 0; f < kFunctionCount; ++f)
    out << kKeys[f] << "=" << c.bind[f].device << ":" << c.bind[f].axis << ":" << (c.bind[f].invert ? 1 : 0) << "\n";
  out << "deadzone=" << c.deadzone << "\n";
  out << "# buttons: AP_DISCONNECT BRAKES FLAPS_UP FLAPS_DOWN GEAR REVERSE SPEEDBRAKE VIEW PAUSE TOGA IDLE\n";
  for (int b = 0; b < kButtons; ++b)
    if (!c.buttons[b].empty()) out << "button" << (b + 1) << "=" << c.buttons[b] << "\n";
  return out.str();
}

inline bool fromText(const std::string& text, Config& c) {
  static const char* kKeys[kFunctionCount] = {"pitch", "roll", "rudder", "throttle", "brakeLeft", "brakeRight"};
  Config parsed;
  for (std::string& b : parsed.buttons) b.clear();
  bool any = false;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    const size_t eq = line.find('=');
    if (eq == std::string::npos) continue;
    const std::string key = line.substr(0, eq), value = line.substr(eq + 1);
    for (int f = 0; f < kFunctionCount; ++f) {
      if (key != kKeys[f]) continue;
      // "<device>:<axis>:<invert>"
      const char* p = value.c_str();
      char* end = nullptr;
      const long dev = std::strtol(p, &end, 10);
      if (*end != ':') continue;
      const long axis = std::strtol(end + 1, &end, 10);
      if (*end != ':') continue;
      const long inv = std::strtol(end + 1, &end, 10);
      parsed.bind[f] = {static_cast<int>(dev), axis < 0 || axis >= kAxes ? 0 : static_cast<int>(axis), inv != 0};
      any = true;
    }
    if (key == "deadzone") parsed.deadzone = std::fmin(std::fmax(std::strtod(value.c_str(), nullptr), 0.0), 0.5);
    if (key.compare(0, 6, "button") == 0) {
      const long button = std::strtol(key.c_str() + 6, nullptr, 10);
      if (button >= 1 && button <= kButtons) parsed.buttons[button - 1] = value;
    }
  }
  if (any) c = parsed;
  return any;
}

}  // namespace joy
}  // namespace a320
