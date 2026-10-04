#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <vector>

#include "Check.h"
#include "a320/Audio.h"

using namespace a320;

namespace {

constexpr int kRate = 22050;

double rms(const std::vector<int16_t>& v) {
  double sum = 0.0;
  for (int16_t x : v) sum += double(x) * x;
  return std::sqrt(sum / v.size()) / 32768.0;
}

std::vector<int16_t> renderSeconds(AudioEngine& a, const A320State& s, double seconds) {
  std::vector<int16_t> out(static_cast<size_t>(seconds * kRate));
  for (size_t i = 0; i < out.size(); i += 512) a.render(out.data() + i, int(std::min<size_t>(512, out.size() - i)), s);
  return out;
}

A320State flying(double n1) {
  A320State s{};
  s.n1[0] = s.n1[1] = n1;
  s.n2[0] = s.n2[1] = 60.0 + n1 * 0.4;
  s.iasKt = 150.0;
  s.gearPos = 1.0;
  return s;
}

}  // namespace

TEST(audio_wav_files_parse) {
  std::ifstream file(std::string(A320_SOUNDS_DIR) + "/fifty.wav", std::ios::binary);
  CHECK(file.good());
  const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  WavData wav;
  CHECK(parseWav(bytes, wav));
  CHECK(wav.channels == 1);
  CHECK(wav.sampleRate == 22050);
  CHECK(wav.samples.size() > 5000);
  const std::vector<uint8_t> junk = {'R', 'I', 'F', 'F', 0, 0, 0, 0};
  CHECK(!parseWav(junk, wav));
}

TEST(audio_loads_all_callouts) {
  AudioEngine a;
  CHECK(a.init(kRate, A320_SOUNDS_DIR));
  for (const char* name : {"fifty", "retard", "stall", "glide_slope", "two_thousand_five_hundred", "chime", "cavalry"})
    CHECK(a.hasClip(name));
  CHECK(!AudioEngine().init(100, A320_SOUNDS_DIR));
}

TEST(audio_engine_follows_thrust_and_pause) {
  AudioEngine idle, toga;
  idle.init(kRate, A320_SOUNDS_DIR);
  toga.init(kRate, A320_SOUNDS_DIR);
  renderSeconds(idle, flying(30.0), 1.0);
  renderSeconds(toga, flying(98.0), 1.0);
  const double rIdle = rms(renderSeconds(idle, flying(30.0), 1.0));
  const double rToga = rms(renderSeconds(toga, flying(98.0), 1.0));
  std::printf("  rms idle %.3f, TOGA %.3f\n", rIdle, rToga);
  CHECK(rIdle > 0.01);
  CHECK(rToga > rIdle * 1.5);
  A320State paused = flying(98.0);
  paused.paused = 1;
  renderSeconds(toga, paused, 1.0);
  CHECK(rms(renderSeconds(toga, paused, 0.5)) < 0.002);
}

TEST(audio_callouts_and_warnings) {
  AudioEngine a;
  a.init(kRate, A320_SOUNDS_DIR);
  A320State s{};
  renderSeconds(a, s, 0.1);  // sync counters
  s.calloutSeq = 1;
  std::strcpy(s.callout, "FIFTY");
  std::vector<int16_t> buf(512);
  a.render(buf.data(), 512, s);
  CHECK(a.activeVoices() == 1);
  CHECK(rms(renderSeconds(a, s, 0.4)) > 0.02);  // silent sim, so this is the voice

  // Master warning: the chime repeats until acknowledged.
  AudioEngine w;
  w.init(kRate, A320_SOUNDS_DIR);
  A320State warn{};
  warn.warnings = A320_WARN_GEAR_NOT_DOWN;
  CHECK(rms(renderSeconds(w, warn, 2.0)) > 0.02);
  w.event(A320_SOUND_ACK_WARNING);
  renderSeconds(w, warn, 1.0);  // let the last chime ring out
  CHECK(rms(renderSeconds(w, warn, 2.0)) < 0.001);
}

// V1 and ROTATE come half a second apart: the second waits for the first instead of talking over it.
TEST(audio_callouts_do_not_overlap) {
  AudioEngine a;
  a.init(kRate, A320_SOUNDS_DIR);
  A320State s{};
  renderSeconds(a, s, 0.1);
  std::vector<int16_t> buf(512);
  s.calloutSeq = 1;
  std::strcpy(s.callout, "V ONE");
  a.render(buf.data(), 512, s);
  CHECK(a.activeVoices() == 1);
  renderSeconds(a, s, 0.3);
  s.calloutSeq = 2;
  std::strcpy(s.callout, "ROTATE");
  a.render(buf.data(), 512, s);
  CHECK(a.activeVoices() == 1);  // still V ONE
  renderSeconds(a, s, 0.5);      // V ONE (0.66 s) has ended, ROTATE speaks
  CHECK(a.activeVoices() == 1);
  CHECK(rms(renderSeconds(a, s, 0.3)) > 0.02);
  renderSeconds(a, s, 1.0);
  CHECK(a.activeVoices() == 0);
}
