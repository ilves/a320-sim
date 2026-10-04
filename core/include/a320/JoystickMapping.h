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
enum Function { kPitch = 0, kRoll, kRudder, kThrottle, kBrakeLeft, kBrakeRight, kThrottle2, kFlaps, kSpeedbrake, kFunctionCount };
// DirectInput axes, ordered so 0-3 match the old Windows joystick API (X Y Z R).
constexpr int kAxes = 8;      // X Y Z RZ RX RY SL0 SL1
constexpr int kButtons = 128;
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

// Flaps lever detents 0, 1, 2, 3, FULL (axis after invert); the lever snaps to the nearest.
struct FlapsCal {
  double pos[5] = {-1.0, -0.5, 0.0, 0.5, 1.0};
};

// Speedbrake lever: RET to FULL, and optionally an ARM position beyond RET (lever pulled up).
struct SpeedbrakeCal {
  double ret = -1.0, full = 1.0;
  bool hasArm = false;
  double arm = -1.0;
};

// A hardware button or switch and the cockpit command it drives. device is a product name, or
// "@stick" / "@throttle" for whatever device has PITCH / THRUST 1 (the older settings files).
struct ButtonBind {
  std::string device;
  int button = 0;  // 0-based
  std::string command;
};

struct Config {
  Binding bind[kFunctionCount];
  double deadzone = 0.06;
  std::vector<ButtonBind> binds;
  ThrottleCal cal[2];          // thrust levers 1 and 2
  FlapsCal flapsCal;
  SpeedbrakeCal speedbrakeCal;
  bool autoThrottle = true;    // bind a throttle quadrant the first time one is seen
};

// What a command does with its button: once per press (encoder clicks count), on while held
// (a switch), or set on press (one position of a rotary selector).
enum class Action { Press, Held, Select };

struct CommandInfo {
  const char* name;   // in the settings file
  const char* label;  // in the setup panel
  Action action;
};

inline const std::vector<CommandInfo>& commandCatalog() {
  static const std::vector<CommandInfo> kCommands = {
      {"AP1", "FCU AP1", Action::Press},
      {"AP2", "FCU AP2", Action::Press},
      {"ATHR", "FCU A/THR", Action::Press},
      {"LOC", "FCU LOC", Action::Press},
      {"APPR", "FCU APPR", Action::Press},
      {"SPD_INC", "SPD knob +", Action::Press},
      {"SPD_DEC", "SPD knob -", Action::Press},
      {"HDG_INC", "HDG knob +", Action::Press},
      {"HDG_DEC", "HDG knob -", Action::Press},
      {"HDG_PULL", "HDG knob pull", Action::Press},
      {"HDG_PUSH", "HDG knob push (hold)", Action::Press},
      {"ALT_INC", "ALT knob +", Action::Press},
      {"ALT_DEC", "ALT knob -", Action::Press},
      {"ALT_PULL", "ALT knob pull", Action::Press},
      {"ALT_PUSH", "ALT knob push (level)", Action::Press},
      {"ALT_1000", "ALT 100/1000 at 1000", Action::Held},
      {"VS_INC", "V/S knob +", Action::Press},
      {"VS_DEC", "V/S knob -", Action::Press},
      {"VS_PULL", "V/S knob pull", Action::Press},
      {"VS_PUSH", "V/S knob push (0)", Action::Press},
      {"TRK_FPA", "HDG-V/S / TRK-FPA", Action::Press},
      {"LS", "EFIS LS", Action::Press},
      {"ND_MODE", "ND mode (next)", Action::Press},
      {"ND_ARC", "ND mode ARC", Action::Select},
      {"ND_NAV", "ND mode NAV", Action::Select},
      {"ND_LS", "ND mode LS", Action::Select},
      {"ND_RANGE_INC", "ND range +", Action::Press},
      {"ND_RANGE_DEC", "ND range -", Action::Press},
      {"ND_RANGE_10", "ND range 10", Action::Select},
      {"ND_RANGE_20", "ND range 20", Action::Select},
      {"ND_RANGE_40", "ND range 40", Action::Select},
      {"ND_RANGE_80", "ND range 80", Action::Select},
      {"ND_RANGE_160", "ND range 160", Action::Select},
      {"ND_RANGE_320", "ND range 320", Action::Select},
      {"AP_DISCONNECT", "AP disconnect", Action::Press},
      {"ATHR_DISCONNECT", "A/THR disconnect", Action::Press},
      {"TOGA", "Thrust TOGA", Action::Press},
      {"IDLE", "Thrust IDLE", Action::Press},
      {"REVERSE", "Reverse on/off", Action::Press},
      {"FLAPS_UP", "Flaps up one", Action::Press},
      {"FLAPS_DOWN", "Flaps down one", Action::Press},
      {"SPEEDBRAKE", "Speedbrake on/off", Action::Press},
      {"SPOILERS_ARM", "Spoilers ARM switch", Action::Held},
      {"GEAR", "Gear toggle", Action::Press},
      {"GEAR_UP", "Gear lever UP", Action::Select},
      {"GEAR_DOWN", "Gear lever DOWN", Action::Select},
      // A gear lever with one contact: down while it is closed (or up, for the opposite one).
      {"GEAR_DOWN_SW", "Gear DOWN switch", Action::Held},
      {"GEAR_UP_SW", "Gear UP switch", Action::Held},
      {"BRAKES", "Brakes (hold)", Action::Held},
      {"PARK_BRAKE", "Parking brake switch", Action::Held},
      {"AUTOBRK_OFF", "Autobrake OFF/DISARM", Action::Select},
      {"AUTOBRK_LO", "Autobrake LO", Action::Select},
      {"AUTOBRK_MED", "Autobrake MED", Action::Select},
      {"AUTOBRK_MAX", "Autobrake MAX/HI", Action::Select},
      {"ENG1_MASTER", "ENG 1 master switch", Action::Held},
      {"ENG2_MASTER", "ENG 2 master switch", Action::Held},
      {"ENG_MODE_CRANK", "ENG MODE at CRANK", Action::Held},
      {"ENG_MODE_IGN", "ENG MODE at IGN/START", Action::Held},
      {"APU_MASTER", "APU MASTER", Action::Press},
      {"APU_START", "APU START", Action::Press},
      {"MASTER_WARN", "Master warning", Action::Press},
      {"VIEW", "View", Action::Press},
      {"PAUSE", "Pause", Action::Press},
  };
  return kCommands;
}

inline const CommandInfo* findCommand(const std::string& name) {
  for (const CommandInfo& c : commandCatalog())
    if (name == c.name) return &c;
  return nullptr;
}

// One command per hardware button: assigning replaces what the button did before.
inline void setBind(Config& c, const std::string& device, int button, const std::string& command) {
  c.binds.erase(std::remove_if(c.binds.begin(), c.binds.end(),
                               [&](const ButtonBind& b) { return b.device == device && b.button == button; }),
                c.binds.end());
  c.binds.push_back({device, button, command});
}

inline void clearBinds(Config& c, const std::string& command) {
  c.binds.erase(std::remove_if(c.binds.begin(), c.binds.end(), [&](const ButtonBind& b) { return b.command == command; }),
                c.binds.end());
}

inline const char* functionName(int f) {
  static const char* kNames[kFunctionCount] = {"PITCH", "ROLL", "RUDDER", "THRUST 1", "BRAKE L", "BRAKE R", "THRUST 2",
                                               "FLAPS", "SPEEDBRAKE"};
  return f >= 0 && f < kFunctionCount ? kNames[f] : "";
}

inline const char* axisName(int a) {
  static const char* kNames[kAxes] = {"X", "Y", "Z", "RZ", "RX", "RY", "SL0", "SL1"};
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
  const char* kStickButtons[] = {"AP_DISCONNECT", "BRAKES", "FLAPS_UP", "FLAPS_DOWN", "GEAR", "REVERSE"};
  for (int b = 0; b < 6; ++b) c.binds.push_back({"@stick", b, kStickButtons[b]});
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

// Flaps axis to the nearest lever position 0..4 (0, 1, 2, 3, FULL).
inline int flapsFromAxis(double v, const FlapsCal& c) {
  int best = 0;
  for (int i = 1; i < 5; ++i)
    if (std::fabs(v - c.pos[i]) < std::fabs(v - c.pos[best])) best = i;
  return best;
}

struct SpeedbrakePosition {
  double amount = 0.0;  // 0 = RET .. 1 = FULL
  bool armed = false;   // lever in the ARM position
};

inline SpeedbrakePosition speedbrakeFromAxis(double v, const SpeedbrakeCal& c) {
  SpeedbrakePosition out;
  const double span = c.full - c.ret;
  if (std::fabs(span) < 1e-6) return out;
  const double t = (v - c.ret) / span;  // 0 at RET, 1 at FULL, negative towards ARM
  if (c.hasArm && t < 0.0) {
    const double armT = (c.arm - c.ret) / span;
    out.armed = armT < 0.0 && t < armT / 2.0;
    return out;
  }
  out.amount = t < 0.05 ? 0.0 : (t > 0.95 ? 1.0 : t);
  return out;
}

// Calibration wizards: thrust levers, flaps lever and speedbrake lever, one detent per step.
enum CalTarget { kCalThrust = 0, kCalFlaps, kCalSpeedbrake, kCalTargets };

inline int calStepCount(int target) { return target == kCalSpeedbrake ? 3 : 5; }

inline const char* calStepLabel(int target, int step) {
  static const char* kThrust[5] = {"IDLE", "CL", "FLX/MCT", "TOGA", "REV MAX (full reverse)"};
  static const char* kFlaps[5] = {"0", "1", "2", "3", "FULL"};
  static const char* kSpeedbrake[3] = {"RET (retracted)", "FULL", "ARM (lever pulled up at RET)"};
  if (step < 0 || step >= calStepCount(target)) return "";
  return target == kCalThrust ? kThrust[step] : (target == kCalFlaps ? kFlaps[step] : kSpeedbrake[step]);
}

// The last step of the thrust (reverse) and speedbrake (ARM) wizards is optional.
inline bool calStepOptional(int target, int step) { return target != kCalFlaps && step == calStepCount(target) - 1; }

inline bool buildFlapsCal(const double raw[5], FlapsCal& cal, bool& flip, std::string& error) {
  flip = raw[4] < raw[0];
  double v[5];
  for (int i = 0; i < 5; ++i) v[i] = flip ? -raw[i] : raw[i];
  for (int i = 0; i < 4; ++i) {
    if (!(v[i + 1] > v[i] + 0.04)) {
      error = "The flaps positions must come in the order 0, 1, 2, 3, FULL along the lever. Try again.";
      return false;
    }
  }
  for (int i = 0; i < 5; ++i) cal.pos[i] = v[i];
  return true;
}

inline bool buildSpeedbrakeCal(double ret, double full, bool withArm, double arm, SpeedbrakeCal& cal, bool& flip,
                               std::string& error) {
  flip = full < ret;
  const double r = flip ? -ret : ret, f = flip ? -full : full, a = flip ? -arm : arm;
  if (f - r < 0.3) {
    error = "FULL must be well away from RET along the lever. Try again.";
    return false;
  }
  if (withArm && !(a < r - 0.05)) {
    error = "ARM must be on the other side of RET from FULL. Try again, or press NO ARM.";
    return false;
  }
  cal = SpeedbrakeCal{r, f, withArm, withArm ? a : r};
  return true;
}

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
    const std::string& name = names[static_cast<size_t>(d)];
    if (isThrottleDevice(name)) {
      // A quadrant with add-on modules shows up as several devices; the levers are on "1&2".
      if (quadrant < 0 || name.find("1&2") != std::string::npos) quadrant = d;
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

inline constexpr const char* kFunctionKeys[kFunctionCount] = {"pitch", "roll", "rudder", "throttle", "brakeLeft",
                                                              "brakeRight", "throttle2", "flaps", "speedbrake"};

inline std::string flapsCalText(const FlapsCal& cal) {
  std::ostringstream out;
  for (int i = 0; i < 5; ++i) out << (i ? ":" : "") << cal.pos[i];
  return out.str();
}

inline bool parseNumbers(const std::string& value, double* out, int count) {
  const char* p = value.c_str();
  for (int i = 0; i < count; ++i) {
    char* end = nullptr;
    out[i] = std::strtod(p, &end);
    if (end == p || (i + 1 < count && *end != ':')) return false;
    p = end + 1;
  }
  return true;
}

inline std::string toText(const Config& c) {
  std::ostringstream out;
  out << "# A320 Sim joystick settings: <function>=<device>:<axis 0-7 = X Y Z RZ RX RY SL0 SL1>:<invert 0/1>:<device name>\n";
  for (int f = 0; f < kFunctionCount; ++f) {
    out << kFunctionKeys[f] << "=" << c.bind[f].device << ":" << c.bind[f].axis << ":" << (c.bind[f].invert ? 1 : 0);
    if (!c.bind[f].deviceName.empty()) out << ":" << c.bind[f].deviceName;
    out << "\n";
  }
  out << "deadzone=" << c.deadzone << "\n";
  out << "# Detents, set by the CAL buttons in the setup panel (F2).\n";
  out << "# thrust: <idle>:<cl>:<flx/mct>:<toga>:<full reverse or none>; flaps: 0:1:2:3:FULL; speedbrake: RET:FULL:<ARM or none>\n";
  out << "throttle1Detents=" << calText(c.cal[0]) << "\n";
  out << "throttle2Detents=" << calText(c.cal[1]) << "\n";
  out << "flapsDetents=" << flapsCalText(c.flapsCal) << "\n";
  out << "speedbrakeDetents=" << c.speedbrakeCal.ret << ":" << c.speedbrakeCal.full << ":";
  if (c.speedbrakeCal.hasArm) out << c.speedbrakeCal.arm; else out << "none";
  out << "\n";
  out << "autoThrottle=" << (c.autoThrottle ? 1 : 0) << "\n";
  out << "# Buttons (easiest set with SET on the BUTTONS page of the setup panel):\n";
  out << "#   bind=<button number, from 1>|<command>|<device name, or @stick / @throttle>\n";
  out << "# Commands:";
  int n = 0;
  for (const CommandInfo& cmd : commandCatalog()) out << (n++ % 8 == 0 ? "\n#  " : "") << " " << cmd.name;
  out << "\n";
  for (const ButtonBind& b : c.binds) out << "bind=" << (b.button + 1) << "|" << b.command << "|" << b.device << "\n";
  return out.str();
}

inline bool fromText(const std::string& text, Config& c) {
  Config parsed;
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
    if (key == "flapsDetents") {
      double v[5];
      if (parseNumbers(value, v, 5) && v[0] < v[1] && v[1] < v[2] && v[2] < v[3] && v[3] < v[4])
        for (int i = 0; i < 5; ++i) parsed.flapsCal.pos[i] = v[i];
    }
    if (key == "speedbrakeDetents") {
      double v[2];
      if (parseNumbers(value + ":", v, 2) && v[1] > v[0]) {
        const size_t armAt = value.rfind(':');
        char* end = nullptr;
        const char* armText = value.c_str() + armAt + 1;
        const double arm = std::strtod(armText, &end);
        const bool hasArm = end != armText && arm < v[0];
        parsed.speedbrakeCal = SpeedbrakeCal{v[0], v[1], hasArm, hasArm ? arm : v[0]};
      }
    }
    if (key == "autoThrottle") parsed.autoThrottle = std::strtol(value.c_str(), nullptr, 10) != 0;
    if (key == "deadzone") parsed.deadzone = std::fmin(std::fmax(std::strtod(value.c_str(), nullptr), 0.0), 0.5);
    if (key == "bind") {
      // "<button>|<command>|<device>"
      const size_t a = value.find('|'), b = a == std::string::npos ? a : value.find('|', a + 1);
      if (b == std::string::npos) continue;
      const long button = std::strtol(value.substr(0, a).c_str(), nullptr, 10);
      const std::string command = trim(value.substr(a + 1, b - a - 1)), device = trim(value.substr(b + 1));
      if (button >= 1 && button <= kButtons && !command.empty() && !device.empty())
        parsed.binds.push_back({device, static_cast<int>(button - 1), command});
    }
    // Older files: buttonN (the stick) and throttleButtonN (the throttle).
    const bool stickKey = key.compare(0, 6, "button") == 0, throttleKey = key.compare(0, 14, "throttleButton") == 0;
    if ((stickKey || throttleKey) && !value.empty()) {
      const long button = std::strtol(key.c_str() + (stickKey ? 6 : 14), nullptr, 10);
      if (button >= 1 && button <= kButtons) parsed.binds.push_back({stickKey ? "@stick" : "@throttle", static_cast<int>(button - 1), value});
    }
  }
  if (any) c = parsed;
  return any;
}

}  // namespace joy
}  // namespace a320
