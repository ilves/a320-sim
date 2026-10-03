#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "a320/a320_api.h"

namespace a320 {

struct WavData {
  int sampleRate = 0;
  int channels = 0;
  std::vector<int16_t> samples;  // interleaved
};

// 16-bit PCM WAV only (what tools/make_callouts.py writes).
bool parseWav(const std::vector<uint8_t>& bytes, WavData& out);

// The cockpit soundscape as one mono stream: engines (N1/N2 driven), airflow, gear and runway
// rumble, speedbrake buffet, touchdown, gear clunks, chimes, the AP-disconnect cavalry charge
// and the spoken callouts. Events are detected from successive sim states.
class AudioEngine {
 public:
  bool init(int sampleRate, const std::string& soundsDir);
  bool ready() const { return rate_ > 0; }
  void setVolume(double volume);
  void event(A320SoundEvent e);
  void render(int16_t* out, int frames, const A320State& s);

  int clipCount() const { return static_cast<int>(clips_.size()); }
  bool hasClip(const std::string& name) const { return clips_.count(name) != 0; }
  size_t activeVoices() const { return voices_.size(); }

 private:
  struct Voice {
    const std::vector<float>* clip;
    size_t pos;
    float gain;
  };
  struct Params {
    float roarAmp = 0, roarA = 0, whineFreq = 0, whineAmp = 0, coreFreq = 0, coreAmp = 0;
    float windAmp = 0, windA = 0, rumbleAmp = 0, buffetAmp = 0;
  };

  void play(const std::string& name, float gain = 1.0f);
  void detectEvents(const A320State& s, double blockS);
  Params targetParams(const A320State& s) const;
  float noise();
  void synthesizeClips();

  int rate_ = 0;
  float volume_ = 0.8f;
  std::map<std::string, std::vector<float>> clips_;
  std::vector<Voice> voices_;

  Params p_;
  double whinePhase_ = 0.0, corePhase_ = 0.0;
  float roar1_ = 0, roar2_ = 0, wind1_ = 0, wind2_ = 0, rumble1_ = 0, rumble2_ = 0, buffet_ = 0;
  uint32_t rng_ = 0x12345678u;

  bool synced_ = false;
  uint32_t calloutSeq_ = 0, touchdownSeq_ = 0, apSeq_ = 0;
  double gearPos_ = 1.0;
  uint32_t acked_ = 0;
  double chimeTimer_ = 0.0, voiceTimer_ = 0.0;
  bool clickPending_ = false;
};

}  // namespace a320
