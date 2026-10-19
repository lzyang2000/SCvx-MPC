#!/usr/bin/env python3
"""Score the Three.js exports using their recorded simulation controls.

Requires Python + NumPy, ffmpeg and ffprobe. No downloaded audio assets.
Example: python roadster/scripts/score-video.py --source-version v11 --version v12
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import wave

import numpy as np

RATE = 48000
FPS = 30
LEAD = 0.7


def run(args):
    return subprocess.run(args, check=True, capture_output=True, text=True)


def wav(path, samples):
    with wave.open(str(path), "wb") as stream:
        stream.setnchannels(2)
        stream.setsampwidth(2)
        stream.setframerate(RATE)
        stream.writeframes((np.clip(samples, -1, 1) * 32767).astype("<i2").tobytes())


def probe(path):
    return json.loads(run(["ffprobe", "-v", "error", "-show_streams", "-of", "json", str(path)]).stdout)["streams"]


class Score:
    def __init__(self, seconds):
        self.n = round(seconds * RATE)
        self.t = np.arange(self.n) / RATE
        self.grid = np.arange(0, seconds + 0.01, 0.005)
        self.rng = np.random.default_rng(20261019)
        self.stems = {k: np.zeros((self.n, 2), np.float32) for k in ["music", "electric-motor", "cold-gas", "rocket-engine", "foley"]}
        self.cues = []

    def expand(self, values, attack=0.015, release=0.07):
        values = np.array(values, dtype=float, copy=True)
        state = 0.0
        for i, value in enumerate(values):
            tau = attack if value > state else release
            state += (value - state) * (1 - np.exp(-0.005 / tau))
            values[i] = state
        return np.interp(self.t, self.grid, values)

    def noise(self, low, high):
        f = np.fft.rfftfreq(self.n, 1 / RATE)
        spectrum = np.fft.rfft(self.rng.standard_normal(self.n))
        highpass = (f / low) ** 2 / (1 + (f / low) ** 2)
        lowpass = 1 / np.sqrt(1 + (f / high) ** 8)
        response = highpass * lowpass / np.sqrt(np.maximum(f, low))
        signal = np.fft.irfft(spectrum * response, n=self.n)
        return (signal / np.sqrt(np.mean(signal ** 2))).astype(np.float32)

    def stereo(self, mono, width=0.18):
        # Short, quiet cross-channel reflections add space without a long tail.
        delay = round(0.011 * RATE)
        reflection = np.zeros_like(mono)
        reflection[delay:] = mono[:-delay]
        return np.column_stack((mono + width * reflection, mono - width * reflection)).astype(np.float32)

    def hit(self, stem, at, length, frequency, gain):
        start = round(at * RATE)
        count = min(round(length * RATE), self.n - start)
        if start < 0 or count <= 0:
            return
        t = np.arange(count) / RATE
        env = (1 - np.exp(-t / 0.002)) * np.exp(-t / (length / 5))
        pitch = frequency * (1 + 0.8 * np.exp(-t / 0.025))
        body = np.sin(2 * np.pi * np.cumsum(pitch) / RATE)
        grit = self.rng.standard_normal(count)
        grit = np.convolve(grit, np.ones(9) / 9, mode="same")
        signal = gain * env * (0.85 * body + 0.15 * grit)
        self.stems[stem][start:start + count] += self.stereo(signal, 0.08)

    def music(self):
        music = self.stems["music"]
        # Original D-minor score: slow pads, a soft sequencer and a restrained beat.
        chords = [[50, 57, 60, 65], [46, 53, 58, 62], [48, 53, 57, 60], [48, 55, 60, 64], [50, 57, 60, 65]]
        for chord_index, notes in enumerate(chords):
            start = chord_index * 10
            for i, note in enumerate(notes):
                frequency = 440 * 2 ** ((note - 69) / 12)
                local = self.t - start
                envelope = np.clip(local / 1.6, 0, 1) * np.clip((11.8 - local) / 2.3, 0, 1)
                pan = [-0.65, 0.45, -0.3, 0.65][i]
                for channel, detune in enumerate([-0.0012, 0.0012]):
                    phase = 2 * np.pi * frequency * (1 + detune) * self.t
                    tone = np.sin(phase) + 0.16 * np.sin(phase * 2) + 0.035 * np.sin(phase * 3)
                    music[:, channel] += (0.016 * envelope * tone * (1 + pan * (1 if channel else -1))).astype(np.float32)
            bass_freq = 440 * 2 ** (((notes[0] - 12) - 69) / 12)
            bass = np.sin(2 * np.pi * bass_freq * self.t)
            env = np.clip((self.t - start) / 0.5, 0, 1) * np.clip((start + 10.1 - self.t) / 0.7, 0, 1)
            music += (0.018 * bass * env)[:, None].astype(np.float32)
        beat = 60 / 96
        for index, at in enumerate(np.arange(0.7, self.t[-1] - 0.8, beat / 2)):
            chord = chords[min(int(at / 10), len(chords) - 1)]
            note = chord[[0, 2, 1, 3, 2, 1, 3, 1][index % 8]] + 12
            start = round(at * RATE)
            count = min(round(0.85 * RATE), self.n - start)
            local = np.arange(count) / RATE
            freq = 440 * 2 ** ((note - 69) / 12)
            env = (1 - np.exp(-local / 0.012)) * np.exp(-local / 0.15)
            signal = 0.011 * env * (np.sin(2 * np.pi * freq * local) + 0.14 * np.sin(4 * np.pi * freq * local))
            pan = 0.35 * (-1) ** index
            music[start:start + count] += np.column_stack((signal * (1 - pan), signal * (1 + pan))).astype(np.float32)
            if index % 2 == 0:
                self.hit("music", at, 0.28, 48, 0.027)
            if index % 4 == 2:
                count = min(round(0.11 * RATE), self.n - start)
                local = np.arange(count) / RATE
                noise = self.rng.standard_normal(count)
                snap = 0.008 * noise * np.exp(-local / 0.026) * np.minimum(1, local / 0.003)
                music[start:start + count] += snap[:, None].astype(np.float32)
        fade = np.clip(self.t / 0.9, 0, 1) * np.clip((self.t[-1] - self.t) / 1.6, 0, 1)
        music *= fade[:, None]

    def car(self, frames, offset, shot, summary):
        times = np.array([f["t"] for f in frames])
        local = self.grid - offset - LEAD
        active = (local >= 0) & (local <= shot["segments"][-1]["to"])
        speed = np.interp(local, times, [np.linalg.norm(f["v"]) for f in frames])
        ground = np.interp(local, times, [int(f["phase"] in ["APPROACH", "RAMP", "CONTACT", "SETTLED"]) for f in frames])
        acceleration = np.abs(np.gradient(speed, 0.005))
        motor_env = self.expand(active * ground * np.minimum(1, speed / 3) * (0.35 + 0.6 * np.minimum(acceleration / 5, 1)))
        motor_speed = np.interp(self.t, self.grid, speed)
        phase = 2 * np.pi * np.cumsum(75 + 19 * motor_speed) / RATE
        inverter_phase = 2 * np.pi * np.cumsum(1700 + 45 * motor_speed) / RATE
        motor = 0.07 * motor_env * (0.60 * np.sin(phase) + 0.25 * np.sin(3 * phase + 0.12 * np.sin(phase * 0.07)) + 0.13 * np.sin(6 * phase) + 0.08 * np.sin(inverter_phase))
        road = self.noise(90, 1400) * 0.022 * motor_env * np.minimum(1, motor_speed / 12)
        fan = np.interp(local, times, [f.get("fan_g", 0) for f in frames])
        fan_env = self.expand(fan * active)
        suction = 0.025 * fan_env * (self.noise(70, 650) + 0.18 * np.sin(2 * np.pi * 205 * self.t))
        self.stems["electric-motor"] += self.stereo(motor + road + suction)
        jets = np.array([f["jets"] for f in frames])
        for side, indices in enumerate([[0, 2, 4], [1, 3, 5]]):
            thrust = np.interp(local, times, np.sum(jets[:, indices], axis=1))
            env = self.expand(active * np.sqrt(np.maximum(thrust, 0) / 25500), 0.009, 0.035)
            hiss = self.noise(650, 11000)
            body = self.noise(110, 1100)
            signal = 0.075 * env * (0.78 * hiss + 0.22 * body)
            stereo = np.column_stack((signal * (1.08 if side == 0 else 0.75), signal * (0.75 if side == 0 else 1.08)))
            self.stems["cold-gas"] += stereo.astype(np.float32)
        touchdown = offset + LEAD + summary["touchdown"]["t"]
        self.hit("foley", touchdown, 0.34, 72, 0.07)
        self.cues.append({"event": shot["name"] + " touchdown", "video_time": touchdown})
        airborne = next(f["t"] for f in frames if sum(f["jets"]) > 100)
        self.cues.append({"event": shot["name"] + " cold-gas onset", "video_time": offset + LEAD + airborne})

    def rocket(self, frames, offset, shot, summary):
        beginning = shot["segments"][0]["from"]
        speed = shot["segments"][0]["speed"]
        local = beginning + (self.grid - offset - LEAD) * speed
        times = [f["t"] for f in frames]
        thrust = np.interp(local, times, [np.linalg.norm(f["control"][:3]) for f in frames])
        # Preserve the sound's pitch at 7x; only its envelope follows the video clock.
        active = (self.grid >= offset + LEAD) & (local < summary["touchdown"]["t"])
        env = self.expand(active * (np.maximum(thrust, 0) / (3 * 845000)) ** 0.55, 0.07, 0.11)
        rumble = self.noise(24, 210)
        roar = self.noise(90, 3600)
        crackle = self.noise(600, 8500)
        flutter = 0.8 + 0.2 * np.tanh(self.noise(3, 25))
        combustion = np.sin(2 * np.pi * 47 * self.t + 0.8 * np.sin(2 * np.pi * 2.3 * self.t))
        core = 0.12 * env * (0.52 * rumble + 0.56 * roar + 0.14 * crackle * flutter + 0.18 * combustion)
        self.stems["rocket-engine"] += self.stereo(np.tanh(core * 1.8) / 1.8, 0.08)
        ignition = offset + LEAD
        self.hit("rocket-engine", ignition, 0.45, 35, 0.1)
        # A brief diffuse exhaust tail follows cutoff; there is no extra ignition.
        cutoff = offset + LEAD + (summary["touchdown"]["t"] - beginning) / speed
        self.hit("foley", cutoff, 0.5, 48, 0.045)
        self.cues.extend([{"event": "landing ignition", "video_time": ignition}, {"event": "touchdown / engine cutoff", "video_time": cutoff}])
        for event, key in [("hover", "hover_started_at"), ("slow descent", "slow_descent_started_at")]:
            self.cues.append({"event": event, "video_time": offset + LEAD + (summary[key] - beginning) / speed})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-version", default="v11")
    parser.add_argument("--version", default="v12")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    out = repo / "output/video"
    source = out / f"threejs-{args.source_version}-render-source"
    shots = json.loads((source / "capture-shots.json").read_text())
    duration = sum(s["frames"] for s in shots) / FPS
    audio = out / f"audio-{args.version}"
    audio.mkdir(parents=True, exist_ok=True)
    score = Score(duration)
    score.music()
    offset = 0.0
    for shot in shots:
        record = json.loads((repo / f"roadster/web/public/{shot['name']}-nominal.json").read_text())
        if shot["name"] == "rocket":
            score.rocket(record["frames"], offset, shot, record["summary"])
        else:
            score.car(record["frames"], offset, shot, record["summary"])
        offset += shot["frames"] / FPS
    effects = sum(stem for name, stem in score.stems.items() if name != "music")
    # Gently duck the score while a jet or engine is prominent.
    energy = np.sqrt(np.mean(effects ** 2, axis=1))
    coarse = np.interp(score.grid, score.t, energy)
    duck = score.expand(np.clip(coarse * 5, 0, 0.45), 0.08, 0.35)
    score.stems["music"] *= (1 - duck[:, None])
    mix = effects + score.stems["music"]
    for name, samples in score.stems.items():
        wav(audio / f"{name}.wav", samples)
    wav(audio / "mix-unmastered.wav", mix)
    base = ["ffmpeg", "-hide_banner", "-nostats", "-i", str(audio / "mix-unmastered.wav")]
    analysis = run(base + ["-af", "loudnorm=I=-16:TP=-1.5:LRA=10:print_format=json", "-f", "null", "-"]).stderr
    measured = json.JSONDecoder().raw_decode(analysis[analysis.rfind("{"):])[0]
    mastering = ("loudnorm=I=-16:TP=-1.5:LRA=10:linear=true:"
                 f"measured_I={measured['input_i']}:measured_TP={measured['input_tp']}:"
                 f"measured_LRA={measured['input_lra']}:measured_thresh={measured['input_thresh']}:"
                 f"offset={measured['target_offset']}")
    master = audio / "soundtrack.wav"
    run(base[:1] + ["-y"] + base[1:] + ["-af", mastering, "-ar", str(RATE), "-c:a", "pcm_s16le", str(master)])
    timing = json.loads((source / "timing.json").read_text())
    clip_start = round(timing["landing_start"] * FPS) / FPS
    validation = []
    for language in ["", "-zh"]:
        for prefix, start, seconds in [("scvx-mpc-demos-threejs", 0, duration), ("rocket-landing", clip_start, duration - clip_start)]:
            original = out / f"{prefix}-{args.source_version}{language}.mp4"
            destination = out / f"{prefix}-{args.version}{language}.mp4"
            run(["ffmpeg", "-y", "-v", "error", "-i", str(original), "-ss", f"{start:.9f}", "-i", str(master), "-map", "0:v:0", "-map", "1:a:0", "-c:v", "copy", "-c:a", "aac", "-b:a", "192k", "-ar", str(RATE), "-t", f"{seconds:.9f}", "-movflags", "+faststart", str(destination)])
            streams = probe(destination)
            video = next(s for s in streams if s["codec_type"] == "video")
            sound = next(s for s in streams if s["codec_type"] == "audio")
            assert abs(float(video["duration"]) - seconds) < 1 / FPS
            assert abs(float(sound["duration"]) - seconds) < 0.03
            assert sound["channels"] == 2 and sound["sample_rate"] == str(RATE)
            run(["ffmpeg", "-v", "error", "-i", str(destination), "-f", "null", "-"])
            hashes = [run(["ffmpeg", "-v", "error", "-i", str(p), "-map", "0:v:0", "-c", "copy", "-f", "hash", "-hash", "sha256", "-"]).stdout.strip() for p in [original, destination]]
            assert hashes[0] == hashes[1], "The audio mux must preserve every original video packet"
            result = {"file": destination.name, "decoded": True, "video_unchanged": True, "video_frames": int(video["nb_frames"]), "duration": seconds, "audio": {"codec": sound["codec_name"], "rate": sound["sample_rate"], "channels": 2}}
            (out / f"{destination.stem}-validation.json").write_text(json.dumps(result, indent=2) + "\n")
            validation.append(result)
            print(json.dumps(result), flush=True)
    mastered_analysis = run(["ffmpeg", "-hide_banner", "-nostats", "-i", str(master), "-af", "loudnorm=I=-16:TP=-1.5:LRA=10:print_format=json", "-f", "null", "-"]).stderr
    levels = json.JSONDecoder().raw_decode(mastered_analysis[mastered_analysis.rfind("{"):])[0]
    assert float(levels["input_tp"]) <= -1.4
    assert -17 <= float(levels["input_i"]) <= -15
    cues = {"sample_rate": RATE, "duration": duration, "music": "Original procedural D-minor electronic instrumental, 96 BPM", "effects": "Synthesized from recorded car speed, nozzle thrust and rocket thrust; pitch stays natural at 7x", "master_lufs": levels["input_i"], "master_true_peak_dbtp": levels["input_tp"], "cues": sorted(score.cues, key=lambda c: c["video_time"]), "exports": validation}
    (audio / "cues-and-validation.json").write_text(json.dumps(cues, indent=2) + "\n")
    shutil.copy2(__file__, audio / "score-video.py")
    print("Audio mastering:", levels["input_i"], "LUFS;", levels["input_tp"], "dBTP", flush=True)


if __name__ == "__main__":
    main()
