/* WingFlex FCU Cube and EFIS Cube USB HID reports (header-only; used by the Unreal front end
 * and the tests). The layouts follow MobiFlight's open-source drivers (MIT,
 * MobiFlight/Joysticks/WingFlex/FcuCubeReport.cs and EfisCubeReport.cs). Payloads are 64
 * bytes, without the leading HID report ID. */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace a320 {
namespace wingflex {

constexpr uint16_t kFcuVendorId = 0xA316, kFcuProductId = 0xC787;
constexpr uint16_t kEfisVendorId = 0xA516, kEfisProductId = 0xC987;
constexpr size_t kPayloadSize = 64;
using Payload = std::array<uint8_t, kPayloadSize>;

enum FcuButton {
  kSpdMach = 0, kSpdPush, kSpdPull, kHdgTrk, kHdgPush, kHdgPull, kAlt100, kAlt1000, kAltPush, kAltPull,
  kVsPush, kVsPull, kMetricAlt, kAp1, kAp2, kAthr, kLoc, kExped, kAppr, kFcuButtons
};
enum FcuKnob { kSpdKnob = 0, kHdgKnob, kAltKnob, kVsKnob, kFcuKnobs };

struct FcuInput {
  bool button[kFcuButtons] = {};
  int8_t knob[kFcuKnobs] = {};  // clicks turned since the last report, + = clockwise
  uint8_t backlight = 0xFF, lcd = 0xFF;  // the panel's own brightness knobs
};

struct FcuOutput {
  bool loc = false, ap1 = false, ap2 = false, athr = false, exped = false, appr = false;
  bool spdManaged = false, spdDashed = false, hdgManaged = false, hdgDashed = false, altManaged = false, vsDashed = false;
  bool mach = false, trkFpa = false;
  uint8_t backlight = 0xC0, lcd = 0xC0;
  uint16_t spd = 0, hdg = 0, alt = 0;
  int16_t vs = 0;
};

enum EfisButton {
  kMasterWarn = 0, kMasterCaution, kChrono, kStickPriority, kFd, kLs, kCstr, kWpt, kVorD, kNdb, kArpt, kInHg, kHpa,
  kNdLs, kNdVor, kNdNav, kNdArc, kNdPlan, kRange10, kRange20, kRange40, kRange80, kRange160, kRange320,
  kAdf1, kOff1, kVor1, kAdf2, kOff2, kVor2, kBaroPush, kBaroPull, kEfisButtons
};

struct EfisInput {
  bool button[kEfisButtons] = {};
  int8_t baroKnob = 0;
};

struct EfisOutput {
  bool masterWarn = false, masterCaution = false, fd = false, ls = false, cstr = false, wpt = false, vorD = false,
       ndb = false, arpt = false, qfe = false, qnh = false, dot = false, std = true;
  uint8_t backlight = 0xC0, lcd = 0xC0;
  uint16_t baro = 1013;
};

inline bool bit(const uint8_t* p, int index) { return (p[6 + index / 8] >> (index % 8)) & 1; }

inline void setBit(Payload& p, int index, bool on) {
  const uint8_t mask = static_cast<uint8_t>(1u << (index % 8));
  uint8_t& b = p[static_cast<size_t>(6 + index / 8)];
  b = on ? static_cast<uint8_t>(b | mask) : static_cast<uint8_t>(b & ~mask);
}

inline void put16(Payload& p, size_t at, uint16_t v) {
  p[at] = static_cast<uint8_t>(v >> 8);
  p[at + 1] = static_cast<uint8_t>(v & 0xFF);
}

// Input report payload (after the report ID). False if it is not an FCU report.
inline bool parseFcu(const uint8_t* p, size_t size, FcuInput& out) {
  if (size < 17 || p[0] != 0xF2 || p[1] != 0xE1 || p[2] != 0x03) return false;
  for (int i = 0; i < kFcuButtons; ++i) out.button[i] = bit(p, i);
  for (int k = 0; k < kFcuKnobs; ++k) out.knob[k] = static_cast<int8_t>(p[11 + k]);
  out.backlight = p[15];
  out.lcd = p[16];
  return true;
}

inline Payload buildFcu(const FcuOutput& o) {
  Payload p{};
  const uint8_t head[] = {0xF2, 0xE1, 0x03, 0x02, 0x01, 0x02};
  for (size_t i = 0; i < sizeof(head); ++i) p[i] = head[i];
  const bool bits[] = {o.loc, o.ap1, o.ap2, o.athr, o.exped, o.appr, o.spdManaged, o.spdDashed,
                       o.hdgManaged, o.hdgDashed, o.altManaged, o.vsDashed, o.mach, o.trkFpa, false, false,
                       true /* power */};
  for (int i = 0; i < static_cast<int>(sizeof(bits)); ++i) setBit(p, i, bits[i]);
  p[9] = 0x02;
  p[10] = 0x02;
  p[11] = o.backlight;
  p[12] = o.lcd;
  p[13] = 0x00;
  p[14] = 0x08;
  put16(p, 15, o.spd);
  put16(p, 17, o.hdg);
  put16(p, 19, o.alt);
  put16(p, 21, static_cast<uint16_t>(o.vs));  // two's complement
  return p;
}

inline bool parseEfis(const uint8_t* p, size_t size, EfisInput& out) {
  if (size < 13 || p[0] != 0xF2 || p[1] != 0xE1 || p[2] != 0x05) return false;
  for (int i = 0; i < kEfisButtons; ++i) out.button[i] = bit(p, i);
  out.baroKnob = static_cast<int8_t>(p[12]);
  return true;
}

inline Payload buildEfis(const EfisOutput& o) {
  Payload p{};
  const uint8_t head[] = {0xF2, 0xE1, 0x05, 0x03, 0x01, 0x03};
  for (size_t i = 0; i < sizeof(head); ++i) p[i] = head[i];
  const bool bits[] = {o.masterWarn, o.masterCaution, false, false, o.fd, o.ls, o.cstr, o.wpt,
                       o.vorD, o.ndb, o.arpt, true /* lights driven by the host */, o.qfe, o.qnh, o.dot, o.std,
                       true /* power */};
  for (int i = 0; i < static_cast<int>(sizeof(bits)); ++i) setBit(p, i, bits[i]);
  p[9] = 0x02;
  p[10] = 0x02;
  p[11] = o.backlight;
  p[12] = o.lcd;
  p[13] = 0x03;
  p[14] = 0x02;
  put16(p, 15, o.baro);
  return p;
}

}  // namespace wingflex
}  // namespace a320
