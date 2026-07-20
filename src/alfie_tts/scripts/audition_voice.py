#!/usr/bin/env python3
"""Audition LibriTTS-R speakers / pitch settings for Alfie's voice.

Renders a test phrase across candidate speaker IDs and playback-rate (pitch)
settings, writing one WAV per combo to an output dir so you can listen and pick.
Optionally plays each one aloud as it renders.

Usage:
    python3 audition_voice.py                      # default candidates -> ./voice_audition/
    python3 audition_voice.py --play               # also play each aloud
    python3 audition_voice.py --speakers 76 515 903 --rates 1.0 1.05 1.10
    python3 audition_voice.py --text "Hi, I'm Alfie!"

Then:  ls -1 voice_audition/  and play the ones you like, e.g.
    paplay voice_audition/spk076_rate1.08.wav
"""
import argparse
import os
import wave

import numpy as np
from piper.voice import PiperVoice
from piper.config import SynthesisConfig

# Resolve the installed voice model (falls back to the source tree).
_CANDIDATE_DIRS = [
    os.path.expanduser(
        "~/alfiebot_ws/install/alfie_tts/share/alfie_tts/voices"),
    os.path.expanduser("~/alfiebot_ws/src/alfie_tts/voices"),
]

# Starting candidates. Lighter/higher androgynous adult voices tend to pitch up
# into a believable child. Widen this list freely once you hear the neighborhood.
DEFAULT_SPEAKERS = [65, 66, 67, 76, 515, 617, 623, 903]
# 1.0 = native pitch; >1.0 shrinks apparent vocal-tract size (more child-like).
DEFAULT_RATES = [1.00, 1.05, 1.10]
DEFAULT_TEXT = "Hi, I'm Alfie! Do you want to play a game with me?"

# Match speech.py's tuning so auditions sound like the real node.
NOISE_SCALE = 0.667
NOISE_W = 0.8


def find_voice():
    for d in _CANDIDATE_DIRS:
        model = os.path.join(d, "en_US-libritts_r-medium.onnx")
        if os.path.exists(model):
            return model, model + ".json"
    raise FileNotFoundError(
        "en_US-libritts_r-medium.onnx not found; build alfie_tts or check paths.")


def synth_to_wav(voice, text, speaker_id, rate, length_scale, out_path):
    """Synthesize `text` and write a WAV, resampling the sample-rate header by
    `rate` to shift pitch+tempo (same trick as the live node's playback_rate_scale)."""
    cfg = SynthesisConfig(
        speaker_id=speaker_id,
        length_scale=length_scale,
        noise_scale=NOISE_SCALE,
        noise_w_scale=NOISE_W,
    )
    pcm = bytearray()
    native_rate = voice.config.sample_rate
    for chunk in voice.synthesize(text, cfg):
        pcm.extend(chunk.audio_int16_bytes)

    with wave.open(out_path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        # Writing a higher sample rate than synthesized speeds up playback,
        # raising pitch — exactly what RawOutputStream(samplerate=...) does live.
        w.setframerate(int(native_rate * rate))
        w.writeframes(bytes(pcm))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speakers", type=int, nargs="+", default=DEFAULT_SPEAKERS)
    ap.add_argument("--rates", type=float, nargs="+", default=DEFAULT_RATES,
                    help="playback_rate_scale values; >1.0 = higher/child-like")
    ap.add_argument("--length-scale", type=float, default=1.00,
                    help="tempo; raise to counteract speed-up from high rates")
    ap.add_argument("--text", default=DEFAULT_TEXT)
    ap.add_argument("--out", default="voice_audition")
    ap.add_argument("--play", action="store_true", help="play each clip via paplay")
    args = ap.parse_args()

    model, config = find_voice()
    print(f"Loading {model} ...")
    voice = PiperVoice.load(model, config, use_cuda=False)
    os.makedirs(args.out, exist_ok=True)

    for spk in args.speakers:
        for rate in args.rates:
            name = f"spk{spk:03d}_rate{rate:.2f}.wav"
            path = os.path.join(args.out, name)
            synth_to_wav(voice, args.text, spk, rate, args.length_scale, path)
            print(f"  wrote {path}")
            if args.play:
                os.system(f"paplay '{path}'")

    print(f"\nDone. {len(args.speakers) * len(args.rates)} clips in ./{args.out}/")
    print("Listen, then set speaker_id + playback_rate_scale in speech.py to match.")


if __name__ == "__main__":
    main()
