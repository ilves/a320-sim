#include "a320/Audio.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>

#include "a320/Units.h"

namespace a320 {
namespace {

constexpr float kTwoPi = 6.28318530718f;

// Spoken callouts shipped as WAV files (file name = callout text, lower case, '_' for spaces).
const char* const kVoiceClips[] = {
    "two_thousand_five_hundred", "one_thousand", "five_hundred", "one_hundred", "fifty", "forty",
    "thirty", "twenty", "ten", "retard", "stall", "glide_slope", "sink_rate",
    "one_hundred_knots", "v_one", "rotate", "positive_climb", "hundred_above", "minimum"};

uint32_t readU32(const uint8_t* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (uint32_t(p[3]) << 24); }
uint16_t readU16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

std::string clipName(const char* callout) {
  std::string name;
  for (const char* c = callout; *c; ++c) name += *c == ' ' ? '_' : static_cast<char>(std::tolower(*c));
  return name;
}

float onePoleCoef(float cutoffHz, int rate) {
  return 1.0f - std::exp(-kTwoPi * cutoffHz / static_cast<float>(rate));
}

}  // namespace

bool parseWav(const std::vector<uint8_t>& bytes, WavData& out) {
  if (bytes.size() < 12 || std::memcmp(bytes.data(), "RIFF", 4) != 0 || std::memcmp(bytes.data() + 8, "WAVE", 4) != 0)
    return false;
  int format = 0, bits = 0;
  size_t pos = 12;
  out = WavData{};
  while (pos + 8 <= bytes.size()) {
    const uint8_t* chunk = bytes.data() + pos;
    const uint32_t size = readU32(chunk + 4);
    const size_t body = pos + 8;
    if (body + size > bytes.size()) return false;
    if (std::memcmp(chunk, "fmt ", 4) == 0 && size >= 16) {
      format = readU16(bytes.data() + body);
      out.channels = readU16(bytes.data() + body + 2);
      out.sampleRate = static_cast<int>(readU32(bytes.data() + body + 4));
      bits = readU16(bytes.data() + body + 14);
    } else if (std::memcmp(chunk, "data", 4) == 0) {
      if (format != 1 || bits != 16 || out.channels < 1) return false;
      out.samples.resize(size / 2);
      for (size_t i = 0; i < out.samples.size(); ++i)
        out.samples[i] = static_cast<int16_t>(readU16(bytes.data() + body + i * 2));
      return out.sampleRate > 0;
    }
    pos = body + size + (size & 1);
  }
  return false;
}

bool AudioEngine::init(int sampleRate, const std::string& soundsDir) {
  if (sampleRate < 8000 || sampleRate > 192000) return false;
  rate_ = sampleRate;
  clips_.clear();
  voices_.clear();
  synced_ = false;
  synthesizeClips();
  for (const char* name : kVoiceClips) {
    std::ifstream file(soundsDir + "/" + name + ".wav", std::ios::binary);
    if (!file) continue;
    const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    WavData wav;
    if (!parseWav(bytes, wav)) continue;
    // Mono, resampled to the engine rate by linear interpolation.
    const size_t frames = wav.samples.size() / static_cast<size_t>(wav.channels);
    const double step = static_cast<double>(wav.sampleRate) / rate_;
    std::vector<float>& clip = clips_[name];
    for (double t = 0.0; t < frames - 1; t += step) {
      const size_t i = static_cast<size_t>(t);
      const double f = t - i;
      float a = 0, b = 0;
      for (int c = 0; c < wav.channels; ++c) {
        a += wav.samples[i * wav.channels + c];
        b += wav.samples[(i + 1) * wav.channels + c];
      }
      clip.push_back(static_cast<float>((a + (b - a) * f) / (32768.0 * wav.channels)));
    }
  }
  return true;
}

void AudioEngine::synthesizeClips() {
  const float r = static_cast<float>(rate_);
  auto make = [&](const char* name, float seconds, auto&& sample) {
    std::vector<float>& clip = clips_[name];
    clip.resize(static_cast<size_t>(seconds * r));
    for (size_t i = 0; i < clip.size(); ++i) clip[i] = sample(i / r);
  };
  // Single chime (master caution; repeated for the master warning).
  make("chime", 0.7f, [](float t) {
    const float env = std::min(t / 0.005f, 1.0f) * std::exp(-t * 6.0f);
    return env * (0.5f * std::sin(kTwoPi * 1046.5f * t) + 0.3f * std::sin(kTwoPi * 1568.0f * t) +
                  0.15f * std::sin(kTwoPi * 2093.0f * t));
  });
  // Cavalry charge (autopilot disconnect): a bright rising arpeggio, played twice.
  make("cavalry", 1.4f, [](float t) {
    static const float kNotes[] = {523.25f, 659.25f, 783.99f, 1046.5f, 783.99f, 1046.5f};
    const float noteLen = 0.115f;
    const int idx = static_cast<int>(t / noteLen) % 6;
    const float nt = std::fmod(t, noteLen);
    const float f = kNotes[idx];
    const float env = std::min(nt / 0.004f, 1.0f) * std::exp(-nt * 14.0f);
    return 0.45f * env * (std::sin(kTwoPi * f * t) + 0.33f * std::sin(3 * kTwoPi * f * t) + 0.2f * std::sin(5 * kTwoPi * f * t));
  });
  uint32_t seed = 99;
  auto rnd = [&seed]() {
    seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
    return (seed & 0xFFFF) / 32768.0f - 1.0f;
  };
  make("click", 0.012f, [&](float t) { return 0.5f * rnd() * std::exp(-t * 700.0f) + 0.3f * std::sin(kTwoPi * 2500.0f * t) * std::exp(-t * 900.0f); });
  float lp = 0.0f;
  make("thump", 0.4f, [&](float t) {
    lp += 0.05f * (rnd() - lp);
    return 0.9f * std::sin(kTwoPi * 48.0f * t) * std::exp(-t * 12.0f) + 2.5f * lp * std::exp(-t * 25.0f);
  });
  float lo = 0.0f, hi = 0.0f;
  make("squeal", 0.35f, [&](float t) {
    const float n = rnd();
    lo += 0.35f * (n - lo);
    hi += 0.12f * (n - hi);
    return 0.5f * (lo - hi) * std::exp(-t * 8.0f);
  });
  // Explosion: a deep boom with a crackling, slowly fading tail.
  float boomLp = 0.0f, crackLp = 0.0f;
  make("explosion", 3.0f, [&](float t) {
    boomLp += 0.02f * (rnd() - boomLp);
    crackLp += 0.3f * (rnd() - crackLp);
    const float boom = 6.0f * boomLp * std::exp(-t * 1.6f) + 0.7f * std::sin(kTwoPi * (38.0f - 8.0f * t) * t) * std::exp(-t * 3.0f);
    const float crackle = crackLp * (rnd() > 0.96f ? 1.0f : 0.15f) * std::exp(-t * 1.2f);
    return std::min(t / 0.003f, 1.0f) * (boom + 0.6f * crackle);
  });
  // Breakup: a bang, then metal tearing (a rough, falling tone over noise).
  float tearLp = 0.0f;
  make("breakup", 2.2f, [&](float t) {
    tearLp += 0.08f * (rnd() - tearLp);
    const float bang = 4.0f * tearLp * std::exp(-t * 6.0f);
    const float f = 140.0f - 40.0f * t;
    const float tear = (std::sin(kTwoPi * f * t) > 0.0f ? 0.25f : -0.25f) * (0.6f + 0.4f * rnd()) * std::exp(-t * 1.5f);
    return std::min(t / 0.002f, 1.0f) * (bang + tear + 0.4f * tearLp * std::exp(-t * 0.8f));
  });
  // Cabin chime ("ding-dong") when the seat belt or no smoking signs change.
  make("ding", 1.2f, [](float t) {
    const float second = t > 0.35f ? 1.0f : 0.0f;
    const float t2 = t - 0.35f;
    const float a = std::min(t / 0.004f, 1.0f) * std::exp(-t * 3.5f) * std::sin(kTwoPi * 880.0f * t);
    const float b = second * std::exp(-t2 * 3.5f) * std::sin(kTwoPi * 698.5f * t2);
    return 0.35f * (a + b);
  });
  make("clunk", 0.2f, [&](float t) { return 0.6f * std::sin(kTwoPi * 90.0f * t) * std::exp(-t * 25.0f) + 0.3f * rnd() * std::exp(-t * 60.0f); });
}

void AudioEngine::setVolume(double volume) { volume_ = static_cast<float>(clamp(volume, 0.0, 1.0)); }

void AudioEngine::event(A320SoundEvent e) {
  if (e == A320_SOUND_CLICK) clickPending_ = true;
  else if (e == A320_SOUND_ACK_WARNING) acked_ = ~0u;  // narrowed to the active bits on the next render
}

void AudioEngine::play(const std::string& name, float gain) {
  const auto it = clips_.find(name);
  if (it == clips_.end() || it->second.empty()) return;
  voices_.push_back({&it->second, 0, gain});
}

float AudioEngine::noise() {
  rng_ ^= rng_ << 13;
  rng_ ^= rng_ >> 17;
  rng_ ^= rng_ << 5;
  return (rng_ & 0xFFFFFF) / 8388608.0f - 1.0f;
}

AudioEngine::Params AudioEngine::targetParams(const A320State& s) const {
  Params p;
  if (s.paused) return p;  // everything fades out while paused
  if (s.destroyed != A320_DESTROYED_NONE) {
    // The engines are gone. Falling: the wind; on the ground: the fire's deep roar.
    const bool burning = s.destroyed == A320_DESTROYED_CRASH || s.sections[0].onGround || s.sections[1].onGround;
    const double v = std::sqrt(s.sections[0].velNorthMps * s.sections[0].velNorthMps +
                               s.sections[0].velEastMps * s.sections[0].velEastMps +
                               s.sections[0].velUpMps * s.sections[0].velUpMps);
    const float kt = static_cast<float>(v * 1.944);
    p.windAmp = s.sections[0].onGround ? 0.0f : 0.22f * std::pow(clamp(kt / 320.0f, 0.0f, 1.0f), 2.0f);
    p.windA = onePoleCoef(250.0f + kt * 5.0f, rate_);
    p.rumbleAmp = burning ? 0.25f : 0.0f;
    p.buffetAmp = burning ? 0.06f : 0.0f;  // the crackle
    return p;
  }
  const float n1 = static_cast<float>((s.n1[0] + s.n1[1]) / 200.0);
  const float n2 = static_cast<float>((s.n2[0] + s.n2[1]) / 200.0);
  const float spool = clamp((n1 - 0.2f) / 0.8f, 0.0f, 1.0f);
  p.roarAmp = (n1 > 0.05f ? 0.05f + 0.32f * std::pow(spool, 1.5f) : 0.0f) * (s.reverse ? 1.5f : 1.0f);
  p.roarA = onePoleCoef(120.0f + 700.0f * n1, rate_);
  // Fan blade-passing tone (CFM56: ~36 blades at ~5200 rpm) and the core whine.
  p.whineFreq = 3100.0f * n1;
  p.whineAmp = 0.03f * clamp(n1 / 0.3f, 0.0f, 1.0f);
  p.coreFreq = 1700.0f * n2;
  p.coreAmp = 0.012f * clamp(n2 / 0.6f, 0.0f, 1.0f);
  const float ias = static_cast<float>(s.iasKt);
  p.windAmp = 0.22f * std::pow(clamp(ias / 320.0f, 0.0f, 1.0f), 2.0f);
  p.windA = onePoleCoef(250.0f + ias * 5.0f, rate_);
  if (s.onGround) {
    p.rumbleAmp = 0.2f * clamp(static_cast<float>(s.groundSpeedKt) / 140.0f, 0.0f, 1.0f);
  } else {
    p.rumbleAmp = 0.22f * static_cast<float>(s.gearPos) * std::pow(clamp(ias / 220.0f, 0.0f, 1.0f), 2.0f);
  }
  p.buffetAmp = 0.25f * static_cast<float>(s.speedbrakePos) * std::pow(clamp(ias / 250.0f, 0.0f, 1.0f), 2.0f);
  // APU: a high turbine whine that rises with its speed.
  p.apuFreq = 2600.0f * static_cast<float>(s.apuN / 100.0);
  p.apuAmp = 0.025f * static_cast<float>(clamp(s.apuN / 60.0, 0.0, 1.0));
  return p;
}

void AudioEngine::detectEvents(const A320State& s, double blockS) {
  if (!synced_) {
    // First block after start or reset: adopt the counters without replaying old events.
    calloutSeq_ = s.calloutSeq;
    calloutQueue_.clear();
    touchdownSeq_ = s.touchdownSeq;
    apSeq_ = s.apDisconnectSeq;
    destroyedSeq_ = s.destroyedSeq;
    impactSeq_[0] = s.sections[0].impactSeq;
    impactSeq_[1] = s.sections[1].impactSeq;
    gearPos_ = s.gearPos;
    signs_ = s.signs;
    synced_ = true;
  }
  if (clickPending_) {
    play("click", 0.8f);
    clickPending_ = false;
  }
  if (s.destroyedSeq != destroyedSeq_) {
    destroyedSeq_ = s.destroyedSeq;
    if (s.destroyed == A320_DESTROYED_BREAKUP) play("breakup", 1.0f);
    else if (s.destroyed == A320_DESTROYED_CRASH) play("explosion", 1.0f);
  }
  for (int i = 0; i < 2; ++i) {
    if (s.sections[i].impactSeq != impactSeq_[i]) {
      impactSeq_[i] = s.sections[i].impactSeq;
      play("explosion", 0.8f);
    }
  }
  if (s.calloutSeq != calloutSeq_) {
    calloutSeq_ = s.calloutSeq;
    if (calloutQueue_.size() < 3) calloutQueue_.push_back(clipName(s.callout));
  }
  calloutBusyS_ -= blockS;
  while (calloutBusyS_ <= 0.0 && !calloutQueue_.empty()) {
    const auto it = clips_.find(calloutQueue_.front());
    calloutQueue_.pop_front();
    if (it == clips_.end() || it->second.empty()) continue;
    voices_.push_back({&it->second, 0, 1.0f});
    calloutBusyS_ = static_cast<double>(it->second.size()) / rate_ + 0.05;
  }
  if (s.touchdownSeq != touchdownSeq_) {
    touchdownSeq_ = s.touchdownSeq;
    play("thump", clamp(static_cast<float>(-s.touchdownFpm) / 500.0f, 0.3f, 1.2f));
    if (s.groundSpeedKt > 50.0) play("squeal", 0.8f);
  }
  if (s.apDisconnectSeq != apSeq_) {
    apSeq_ = s.apDisconnectSeq;
    play("cavalry");
  }
  const bool gearSettled = s.gearPos <= 0.001 || s.gearPos >= 0.999;
  const bool gearWasMoving = gearPos_ > 0.001 && gearPos_ < 0.999;
  if (gearSettled && gearWasMoving) play("clunk", 0.9f);
  gearPos_ = s.gearPos;
  if (s.signs != signs_) {
    signs_ = s.signs;
    play("ding", 0.8f);
  }

  if (s.paused) return;
  // Master warning: continuous repetitive chime until acknowledged. Stall and GPWS speak instead.
  const uint32_t red = s.warnings & ~uint32_t(A320_WARN_CAUTION_MASK) & ~uint32_t(A320_WARN_STALL);
  acked_ &= red;
  chimeTimer_ -= blockS;
  if ((red & ~acked_) && chimeTimer_ <= 0.0) {
    play("chime", 0.7f);
    chimeTimer_ = 0.9;
  }
  voiceTimer_ -= blockS;
  if (voiceTimer_ <= 0.0) {
    if (s.warnings & A320_WARN_STALL) {
      play("stall");
      voiceTimer_ = 1.6;
    } else if (s.warnings & A320_WARN_SINK_RATE) {
      play("sink_rate");
      voiceTimer_ = 2.5;
    } else if (s.warnings & A320_WARN_GLIDESLOPE) {
      play("glide_slope", 0.8f);
      voiceTimer_ = 3.0;
    }
  }
}

void AudioEngine::radioClip(const int16_t* samples, int frames, int sampleRate, int khz) {
  if (!ready() || !samples || frames <= 0 || sampleRate <= 0) return;
  RadioClip clip{{}, khz, 0};
  // Squelch opening, the voice, then the squelch tail: noise bursts framing the transmission.
  const size_t open = static_cast<size_t>(0.05 * rate_), tail = static_cast<size_t>(0.09 * rate_);
  const double step = static_cast<double>(sampleRate) / rate_;
  clip.samples.reserve(open + tail + static_cast<size_t>(frames / step) + 1);
  for (size_t i = 0; i < open; ++i) clip.samples.push_back(0.18f * noise());
  for (double t = 0.0; t < frames - 1; t += step) {
    const size_t i = static_cast<size_t>(t);
    const double f = t - static_cast<double>(i);
    clip.samples.push_back(static_cast<float>((samples[i] + (samples[i + 1] - samples[i]) * f) / 32768.0));
  }
  for (size_t i = 0; i < tail; ++i) clip.samples.push_back(0.25f * noise() * (1.0f - static_cast<float>(i) / tail));
  radio_.push_back(std::move(clip));
}

// The next radio sample: band-limited like a VHF set (about 350-2800 Hz), slightly overdriven,
// with a little hiss. Clips for another frequency are dropped (the pilot tuned away).
float AudioEngine::radioSample(int activeKhz) {
  while (!radio_.empty() && (radio_.front().khz != activeKhz || radio_.front().pos >= radio_.front().samples.size()))
    radio_.pop_front();
  if (radio_.empty()) return 0.0f;
  RadioClip& clip = radio_.front();
  const float x = clip.samples[clip.pos++] + 0.015f * noise();
  const float hpA = 1.0f - onePoleCoef(350.0f, rate_);
  radioHp_ = hpA * (radioHp_ + x - radioHpIn_);
  radioHpIn_ = x;
  const float lpA = onePoleCoef(2800.0f, rate_);
  radioLp1_ += lpA * (radioHp_ - radioLp1_);
  radioLp2_ += lpA * (radioLp1_ - radioLp2_);
  return 0.8f * std::tanh(2.2f * radioLp2_);
}

void AudioEngine::render(int16_t* out, int frames, const A320State& s) {
  if (!ready() || frames <= 0) {
    if (out && frames > 0) std::fill(out, out + frames, int16_t(0));
    return;
  }
  detectEvents(s, static_cast<double>(frames) / rate_);
  const Params target = targetParams(s);
  // Parameters glide towards their targets (~50 ms) so nothing clicks or zippers.
  const float glide = onePoleCoef(3.0f, rate_);
  const float rumbleA = onePoleCoef(70.0f, rate_);
  const float buffetA = onePoleCoef(35.0f, rate_);
  const float invRate = 1.0f / rate_;
  // Rain: louder with speed; nothing in a wreck or while paused.
  const float rainTarget = s.weather == A320_WEATHER_RAIN && !s.paused && s.destroyed == A320_DESTROYED_NONE
                               ? 0.04f + 0.08f * clamp(static_cast<float>(s.iasKt) / 250.0f, 0.0f, 1.0f)
                               : 0.0f;
  const float rainHpA = onePoleCoef(2500.0f, rate_);
  const float dropDecay = 1.0f - onePoleCoef(60.0f, rate_);

  for (int i = 0; i < frames; ++i) {
    p_.roarAmp += glide * (target.roarAmp - p_.roarAmp);
    p_.roarA += glide * (target.roarA - p_.roarA);
    p_.whineFreq += glide * (target.whineFreq - p_.whineFreq);
    p_.whineAmp += glide * (target.whineAmp - p_.whineAmp);
    p_.coreFreq += glide * (target.coreFreq - p_.coreFreq);
    p_.coreAmp += glide * (target.coreAmp - p_.coreAmp);
    p_.windAmp += glide * (target.windAmp - p_.windAmp);
    p_.windA += glide * (target.windA - p_.windA);
    p_.rumbleAmp += glide * (target.rumbleAmp - p_.rumbleAmp);
    p_.buffetAmp += glide * (target.buffetAmp - p_.buffetAmp);
    p_.apuFreq += glide * (target.apuFreq - p_.apuFreq);
    p_.apuAmp += glide * (target.apuAmp - p_.apuAmp);

    const float n = noise();
    roar1_ += p_.roarA * (n - roar1_);
    roar2_ += p_.roarA * (roar1_ - roar2_);
    wind1_ += p_.windA * (noise() - wind1_);
    wind2_ += p_.windA * (wind1_ - wind2_);
    rumble1_ += rumbleA * (noise() - rumble1_);
    rumble2_ += rumbleA * (rumble1_ - rumble2_);
    buffet_ += buffetA * (noise() - buffet_);
    rainAmp_ += glide * (rainTarget - rainAmp_);
    const float rn = noise();
    rainLp_ += rainHpA * (rn - rainLp_);
    // Single drops: a random tick now and then, decaying in a few milliseconds.
    if (rainAmp_ > 0.001f && (noise() + 1.0f) * 0.5f < 300.0f * invRate) rainDrop_ = noise();
    rainDrop_ *= dropDecay;

    whinePhase_ += p_.whineFreq * invRate;
    corePhase_ += p_.coreFreq * invRate;
    apuPhase_ += p_.apuFreq * invRate;
    apuPhase_ -= std::floor(apuPhase_);
    whinePhase_ -= std::floor(whinePhase_);
    corePhase_ -= std::floor(corePhase_);

    // Filtered noise loses level as the cutoff drops; the factors keep the mix roughly even.
    float mix = 3.0f * p_.roarAmp * roar2_ + 2.0f * p_.windAmp * wind2_ + 6.0f * p_.rumbleAmp * rumble2_ +
                5.0f * p_.buffetAmp * buffet_ +
                p_.whineAmp * std::sin(kTwoPi * static_cast<float>(whinePhase_)) +
                p_.coreAmp * std::sin(kTwoPi * static_cast<float>(corePhase_)) +
                p_.apuAmp * std::sin(kTwoPi * static_cast<float>(apuPhase_)) +
                rainAmp_ * (1.5f * (rn - rainLp_) + 3.0f * rainDrop_);
    for (Voice& v : voices_) {
      if (v.pos < v.clip->size()) mix += v.gain * (*v.clip)[v.pos++];
    }
    mix += radioSample(s.com1ActiveKhz);
    const float y = std::tanh(mix * volume_);
    out[i] = static_cast<int16_t>(clamp(y, -1.0f, 1.0f) * 32000.0f);
  }
  voices_.erase(std::remove_if(voices_.begin(), voices_.end(), [](const Voice& v) { return v.pos >= v.clip->size(); }),
                voices_.end());
}

}  // namespace a320
