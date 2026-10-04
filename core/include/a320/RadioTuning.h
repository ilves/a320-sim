/* VHF COM tuning on the radio management panel (header-only; used by the Unreal front end and
 * the tests). European airspace uses 8.33 kHz channels: each 25 kHz block has four channel names
 * ending in 0, 5, 10 and 15 kHz (135.900, 135.905, 135.910, 135.915, 135.925...). */
#pragma once

namespace a320 {
namespace radio {

constexpr int kMinKhz = 118000;
constexpr int kMaxKhz = 136990;

inline bool isChannel(int khz) {
  const int inBlock = ((khz % 25) + 25) % 25;
  return khz >= kMinKhz && khz <= kMaxKhz && inBlock <= 15 && inBlock % 5 == 0;
}

// Inner knob: the next channel name up or down, wrapping within the MHz like the real knob.
inline int stepKhz(int khz, int dir) {
  const int mhz = khz / 1000;
  int k = khz % 1000;
  for (int i = 0; i < 1000; ++i) {
    k = ((k + (dir > 0 ? 5 : -5)) % 1000 + 1000) % 1000;
    if (isChannel(mhz * 1000 + k)) return mhz * 1000 + k;
  }
  return khz;
}

// Outer knob: whole MHz, 118 to 136, keeping the kHz.
inline int stepMhz(int khz, int dir) {
  int mhz = khz / 1000 + (dir > 0 ? 1 : -1);
  if (mhz > kMaxKhz / 1000) mhz = kMinKhz / 1000;
  if (mhz < kMinKhz / 1000) mhz = kMaxKhz / 1000;
  return mhz * 1000 + khz % 1000;
}

// Transponder entry: Mode A codes have four octal digits (0-7).
inline bool isSquawk(int code) {
  if (code < 0 || code > 7777) return false;
  for (int c = code; c > 0; c /= 10)
    if (c % 10 > 7) return false;
  return true;
}

}  // namespace radio
}  // namespace a320
