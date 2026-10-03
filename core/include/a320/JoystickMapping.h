/* Joystick axis/button mapping (header-only; used by the Unreal front end and the tests). */
#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

namespace a320 {
namespace joy {

// kThrottle is thrust lever 1, or both levers while kThrottle2 is unbound.
enum Function { kPitch = 0, kRoll, kRudder, kThrottle, kBrakeLeft, kBrakeRight, kThrottle2, kFunctionCount };
constexpr int kAxes = 6;      // X Y Z R U V, as Windows reports them
constexpr int kButtons = 32;
using AxisValues = std::array<double, kAxes>;  // -1..1 per axis of one device

struct Binding {
  int device = -1;  // -1 = not bound, or its named device is not plugged in
  int axis = 0;
  bool invert = false;
  // Windows numbers devices in plug-in order, which can change; the name keeps a binding on
  // its device (see resolveDevices).
  std::string deviceName;
};

// Axis positions (after invert, -1..1) of a thrust lever's hardware detents, recorded by the
// calibration in the joystick panel. The defaults spread the axis linearly, as on a slider.
struct ThrottleCal {
  double idle = -1.0, climb = 0.5, flex = 0.76, toga = 1.0;
  bool hasReverse = false;
  double reverseMax = -1.0;  // full reverse, behind IDLE
};

struct Config {
  Binding bind[kFunctionCount];
  double deadzone = 0.06;
  std::string buttons[kButtons];          // main stick (the PITCH device): command per button
  std::string throttleButtons[kButtons];  // the THRUST 1 device, when it is a separate one
  ThrottleCal cal[2];                     // thrust levers 1 and 2
  bool autoThrottle = true;               // bind a throttle quadrant the first time one is seen
};

inline const char* functionName(int f) {
  static const char* kNames[kFunctionCount] = {"PITCH", "ROLL", "RUDDER", "THRUST 1", "BRAKE L", "BRAKE R", "THRUST 2"};
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
  c.bind[kPitch] = {0, 1, false, {}};
  c.bind[kRoll] = {0, 0, false, {}};
  c.bind[kRudder] = {0, 3, false, {}};
  c.bind[kThrottle] = {0, 2, true, {}};
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

struct LeverPosition {
  double lever = 0.0;  // 0..1 forward (IDLE 0, CL 0.75, FLX/MCT 0.88, TOGA 1), or the reverse amount
  bool reverse = false;
};

// Throttle axis (-1..1, after invert) to a thrust lever through its detent calibration,
// snapping into the A320 detents.
inline LeverPosition throttleFromAxis(double v, const ThrottleCal& c) {
  LeverPosition out;
  if (c.hasReverse && c.reverseMax < c.idle) {
    // A dead band behind IDLE, so resting in the idle detent never deploys the reversers.
    const double start = c.idle - std::fmax(0.03, 0.15 * (c.idle - c.reverseMax));
    if (v <= start) {
      const double span = start - c.reverseMax;
      double amount = span > 1e-6 ? (start - v) / span : 1.0;
      amount = amount > 0.95 ? 1.0 : (amount < 0.05 ? 0.0 : amount);
      out.lever = amount;
      out.reverse = true;
      return out;
    }
  }
  const double xs[4] = {c.idle, c.climb, c.flex, c.toga};
  const double ys[4] = {0.0, 0.75, 0.88, 1.0};
  double lever = v <= xs[0] ? 0.0 : 1.0;
  for (int i = 0; i < 3; ++i) {
    if (v > xs[i] && v <= xs[i + 1]) {
      lever = ys[i] + (v - xs[i]) / (xs[i + 1] - xs[i]) * (ys[i + 1] - ys[i]);
      break;
    }
  }
  for (double detent : ys) {
    if (std::fabs(lever - detent) < 0.03) lever = detent;
  }
  out.lever = lever;
  return out;
}

// Detent a lever from throttleFromAxis sits in (it snaps exactly onto them), for display.
inline const char* leverDetentName(double lever) {
  if (lever == 0.0) return "IDLE";
  if (lever == 0.75) return "CL";
  if (lever == 0.88) return "FLX/MCT";
  if (lever == 1.0) return "TOGA";
  return "MAN";
}

// Calibration wizard steps: the lever is put in each detent in turn.
enum CalStep { kCalIdle = 0, kCalClimb, kCalFlex, kCalToga, kCalReverse, kCalSteps };

inline const char* calStepName(int step) {
  static const char* kNames[kCalSteps] = {"IDLE", "CL", "FLX/MCT", "TOGA", "REV MAX (full reverse)"};
  return step >= 0 && step < kCalSteps ? kNames[step] : "";
}

// Detent calibration from the axis positions recorded at each step. An axis that runs
// backwards (TOGA below IDLE) is mirrored, and flip asks the caller to toggle the invert.
inline bool buildCalibration(const double raw[kCalSteps], bool withReverse, ThrottleCal& cal, bool& flip, std::string& error) {
  flip = raw[kCalToga] < raw[kCalIdle];
  double v[kCalSteps];
  for (int i = 0; i < kCalSteps; ++i) v[i] = flip ? -raw[i] : raw[i];
  constexpr double kGap = 0.02;
  if (!(v[kCalClimb] > v[kCalIdle] + kGap && v[kCalFlex] > v[kCalClimb] + kGap && v[kCalToga] > v[kCalFlex] + kGap)) {
    error = "The detents must come in the order IDLE, CL, FLX/MCT, TOGA along the lever. Try again.";
    return false;
  }
  if (withReverse && !(v[kCalReverse] < v[kCalIdle] - 0.1)) {
    error = "Full reverse must be clearly behind IDLE. Try again, or SKIP the reverse step.";
    return false;
  }
  cal.idle = v[kCalIdle];
  cal.climb = v[kCalClimb];
  cal.flex = v[kCalFlex];
  cal.toga = v[kCalToga];
  cal.hasReverse = withReverse;
  cal.reverseMax = withReverse ? v[kCalReverse] : -1.0;
  return true;
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

inline std::string lowercase(std::string t) {
  std::transform(t.begin(), t.end(), t.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return t;
}

// Throttle quadrants by product name, e.g. "TCA Q-Eng 1&2", "Saitek Pro Flight Throttle Quadrant".
inline bool isThrottleDevice(const std::string& name) {
  const std::string n = lowercase(name);
  for (const char* key : {"throttle", "quadrant", "q-eng", "tq6"})
    if (n.find(key) != std::string::npos) return true;
  return false;
}

inline bool isTwinLeverDevice(const std::string& name) {
  const std::string n = lowercase(name);
  for (const char* key : {"quadrant", "q-eng", "tq6"})
    if (n.find(key) != std::string::npos) return true;
  return false;
}

// Points each binding at its device by name; a binding without a name takes its current
// device's. Returns true if the config changed (and should be saved).
inline bool resolveDevices(Config& c, const std::vector<std::string>& names) {
  const int count = static_cast<int>(names.size());
  bool changed = false;
  for (Binding& b : c.bind) {
    if (b.deviceName.empty()) {
      if (b.device >= 0 && b.device < count) {
        b.deviceName = names[static_cast<size_t>(b.device)];
        changed = true;
      }
      continue;
    }
    int found = b.device >= 0 && b.device < count && names[static_cast<size_t>(b.device)] == b.deviceName ? b.device : -1;
    for (int d = 0; d < count && found < 0; ++d)
      if (names[static_cast<size_t>(d)] == b.deviceName) found = d;
    if (found != b.device) {
      b.device = found;
      changed = true;
    }
  }
  return changed;
}

// The first time a throttle quadrant is plugged in: THRUST 1 (and THRUST 2 on a two-lever
// quadrant) go to its levers, and stick functions that defaulted to it move to the stick.
// hasAxis[d][a]: which axes device d has. Returns true if the config changed.
inline bool autoAssignThrottle(Config& c, const std::vector<std::string>& names, const std::vector<std::array<bool, kAxes>>& hasAxis) {
  if (!c.autoThrottle) return false;
  int quadrant = -1, stick = -1;
  for (int d = 0; d < static_cast<int>(names.size()); ++d) {
    if (isThrottleDevice(names[static_cast<size_t>(d)])) {
      if (quadrant < 0) quadrant = d;
    } else if (stick < 0) {
      stick = d;
    }
  }
  if (quadrant < 0) return false;
  c.autoThrottle = false;
  const std::string& name = names[static_cast<size_t>(quadrant)];
  const std::array<bool, kAxes>& axes = hasAxis[static_cast<size_t>(quadrant)];
  const bool twin = isTwinLeverDevice(name) && axes[1];
  const bool alreadySet = c.bind[kThrottle].device == quadrant && (!twin || c.bind[kThrottle2].device == quadrant);
  if (!alreadySet) {
    c.bind[kThrottle] = {quadrant, twin || !axes[2] ? 0 : 2, false, name};
    if (twin) c.bind[kThrottle2] = {quadrant, 1, false, name};
    c.cal[0] = c.cal[1] = ThrottleCal{};
  }
  for (int f : {kPitch, kRoll, kRudder}) {
    Binding& b = c.bind[f];
    if (b.device != quadrant) continue;
    if (stick >= 0) {
      b.device = stick;
      b.deviceName = names[static_cast<size_t>(stick)];
    } else {
      b = Binding{};
    }
  }
  return true;
}

inline std::string calText(const ThrottleCal& cal) {
  std::ostringstream out;
  out << cal.idle << ":" << cal.climb << ":" << cal.flex << ":" << cal.toga << ":";
  if (cal.hasReverse) out << cal.reverseMax; else out << "none";
  return out.str();
}

inline bool parseCal(const std::string& value, ThrottleCal& cal) {
  double v[4];
  const char* p = value.c_str();
  for (int i = 0; i < 4; ++i) {
    char* end = nullptr;
    v[i] = std::strtod(p, &end);
    if (end == p || *end != ':') return false;
    p = end + 1;
  }
  if (!(v[0] < v[1] && v[1] < v[2] && v[2] < v[3])) return false;
  char* end = nullptr;
  const double rev = std::strtod(p, &end);
  cal = ThrottleCal{v[0], v[1], v[2], v[3], end != p && rev < v[0], end != p && rev < v[0] ? rev : -1.0};
  return true;
}

inline constexpr const char* kFunctionKeys[kFunctionCount] = {"pitch", "roll", "rudder", "throttle", "brakeLeft", "brakeRight", "throttle2"};

inline std::string toText(const Config& c) {
  std::ostringstream out;
  out << "# A320 Sim joystick settings: <function>=<device>:<axis 0-5 = X Y Z R U V>:<invert 0/1>:<device name>\n";
  for (int f = 0; f < kFunctionCount; ++f) {
    out << kFunctionKeys[f] << "=" << c.bind[f].device << ":" << c.bind[f].axis << ":" << (c.bind[f].invert ? 1 : 0);
    if (!c.bind[f].deviceName.empty()) out << ":" << c.bind[f].deviceName;
    out << "\n";
  }
  out << "deadzone=" << c.deadzone << "\n";
  out << "# Thrust lever detents (set by CALIBRATE THRUST): <idle>:<cl>:<flx/mct>:<toga>:<full reverse or none>\n";
  out << "throttle1Detents=" << calText(c.cal[0]) << "\n";
  out << "throttle2Detents=" << calText(c.cal[1]) << "\n";
  out << "autoThrottle=" << (c.autoThrottle ? 1 : 0) << "\n";
  out << "# Buttons: AP_DISCONNECT ATHR_DISCONNECT BRAKES FLAPS_UP FLAPS_DOWN GEAR REVERSE SPEEDBRAKE VIEW PAUSE\n";
  out << "#   TOGA IDLE AP1 ATHR; switches (on while held): ENG1_MASTER ENG2_MASTER ENG_MODE_CRANK ENG_MODE_IGN\n";
  out << "# buttonN: the stick (PITCH device); throttleButtonN: the throttle (THRUST 1 device), if separate.\n";
  for (int b = 0; b < kButtons; ++b)
    if (!c.buttons[b].empty()) out << "button" << (b + 1) << "=" << c.buttons[b] << "\n";
  for (int b = 0; b < kButtons; ++b)
    if (!c.throttleButtons[b].empty()) out << "throttleButton" << (b + 1) << "=" << c.throttleButtons[b] << "\n";
  return out.str();
}

inline bool fromText(const std::string& text, Config& c) {
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
    auto trim = [](std::string t) {
      const size_t b = t.find_first_not_of(" \t");
      const size_t e = t.find_last_not_of(" \t");
      return b == std::string::npos ? std::string() : t.substr(b, e - b + 1);
    };
    const std::string key = trim(line.substr(0, eq)), value = trim(line.substr(eq + 1));
    for (int f = 0; f < kFunctionCount; ++f) {
      if (key != kFunctionKeys[f]) continue;
      // "<device>:<axis>:<invert>[:<device name>]"
      const char* p = value.c_str();
      char* end = nullptr;
      const long dev = std::strtol(p, &end, 10);
      if (*end != ':') continue;
      const long axis = std::strtol(end + 1, &end, 10);
      if (*end != ':') continue;
      const long inv = std::strtol(end + 1, &end, 10);
      const std::string name = *end == ':' ? trim(std::string(end + 1)) : std::string();
      parsed.bind[f] = {static_cast<int>(dev), axis < 0 || axis >= kAxes ? 0 : static_cast<int>(axis), inv != 0, name};
      any = true;
    }
    if (key == "throttle1Detents") parseCal(value, parsed.cal[0]);
    if (key == "throttle2Detents") parseCal(value, parsed.cal[1]);
    if (key == "autoThrottle") parsed.autoThrottle = std::strtol(value.c_str(), nullptr, 10) != 0;
    if (key.compare(0, 14, "throttleButton") == 0) {
      const long button = std::strtol(key.c_str() + 14, nullptr, 10);
      if (button >= 1 && button <= kButtons) parsed.throttleButtons[button - 1] = value;
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
