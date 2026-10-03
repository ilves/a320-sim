#!/usr/bin/env python3
"""Generates the spoken cockpit callouts in unreal/A320Sim/Content/Sounds with eSpeak NG.

A flat, synthetic male voice band-limited like a cockpit loudspeaker - close in character to
the Airbus auto-callouts. Requires: espeak-ng on PATH, Python 3 (standard library only).
"""
import array
import math
import os
import subprocess
import sys
import tempfile
import wave

CALLOUTS = {
    "two_thousand_five_hundred": "two thousand five hundred",
    "one_thousand": "one thousand",
    "five_hundred": "five hundred",
    "one_hundred": "one hundred",
    "fifty": "fifty",
    "forty": "forty",
    "thirty": "thirty",
    "twenty": "twenty",
    "ten": "ten",
    "retard": "retard",
    "stall": "stall, stall",
    "glide_slope": "glide slope",
    "sink_rate": "sink rate",
}
RATE = 22050
OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "unreal", "A320Sim", "Content", "Sounds")


def one_pole(samples, cutoff, rate, highpass=False):
    a = 1.0 - math.exp(-2.0 * math.pi * cutoff / rate)
    y, out = 0.0, []
    for x in samples:
        y += a * (x - y)
        out.append(x - y if highpass else y)
    return out


def speak(text):
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
        path = tmp.name
    subprocess.run(["espeak-ng", "-v", "en-us+m3", "-s", "145", "-p", "38", "-a", "160", "-w", path, text], check=True)
    with wave.open(path, "rb") as w:
        rate, data = w.getframerate(), w.readframes(w.getnframes())
    os.unlink(path)
    pcm = array.array("h", data)
    return [s / 32768.0 for s in pcm], rate


def resample(samples, src, dst):
    step = src / dst
    out, t = [], 0.0
    while t < len(samples) - 1:
        i = int(t)
        f = t - i
        out.append(samples[i] * (1 - f) + samples[i + 1] * f)
        t += step
    return out


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for name, text in CALLOUTS.items():
        samples, rate = speak(text)
        samples = resample(samples, rate, RATE)
        # Cockpit speaker: 300 Hz - 3.4 kHz, then normalise to -3 dBFS and trim silence.
        samples = one_pole(one_pole(samples, 300.0, RATE, highpass=True), 3400.0, RATE)
        peak = max(abs(s) for s in samples) or 1.0
        samples = [s * 0.7 / peak for s in samples]
        loud = [i for i, s in enumerate(samples) if abs(s) > 0.02]
        samples = samples[max(loud[0] - 200, 0): loud[-1] + 2000] if loud else samples
        pcm = array.array("h", (int(max(-1.0, min(1.0, s)) * 32767) for s in samples))
        path = os.path.join(OUT_DIR, name + ".wav")
        with wave.open(path, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(RATE)
            w.writeframes(pcm.tobytes())
        print(f"{path}: {len(samples) / RATE:.2f} s")


if __name__ == "__main__":
    sys.exit(main())
