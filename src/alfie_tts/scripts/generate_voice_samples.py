#!/usr/bin/env python3
"""Generate an mp3 sample for every speaker in the Piper LibriTTS-R voice model.

The en_US-libritts_r-medium model is multi-speaker (904 speakers). This script
synthesizes the same sample sentence with each speaker_id and writes one mp3 per
voice to ~/samples so they can be auditioned to pick a robot voice.

Piper yields raw int16 PCM; we pipe it to ffmpeg to encode mp3. onnxruntime's
Run() releases the GIL, so a ThreadPoolExecutor gives real parallelism across
speakers while sharing a single loaded voice.

Usage:
    python3 generate_voice_samples.py [--speakers N] [--workers K] [--text "..."]
"""

import argparse
import os
import subprocess
import sys
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed

from piper.voice import PiperVoice
from piper.config import SynthesisConfig

# Match the runtime prosody settings used by alfie_tts/speech.py so the samples
# sound like what the robot would actually produce.
LENGTH_SCALE = 0.80
NOISE_SCALE = 0.667
NOISE_W = 0.8

DEFAULT_TEXT = (
    "Hello, I'm Alfie. This is a sample of my voice so you can hear how I sound."
)


def find_voice_dir():
    """Locate the installed/source voices directory."""
    candidates = [
        os.path.expanduser(
            "~/alfiebot_ws/install/alfie_tts/share/alfie_tts/voices"),
        os.path.expanduser("~/alfiebot_ws/src/alfie_tts/voices"),
    ]
    for d in candidates:
        if os.path.exists(os.path.join(d, "en_US-libritts_r-medium.onnx")):
            return d
    sys.exit("Could not find the voice model directory.")


def encode_mp3(pcm_bytes, sample_rate, out_path):
    """Encode raw signed-16-bit little-endian mono PCM to an mp3 via ffmpeg."""
    proc = subprocess.run(
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-f", "s16le", "-ar", str(sample_rate), "-ac", "1",
            "-i", "pipe:0",
            "-codec:a", "libmp3lame", "-qscale:a", "2",
            out_path,
        ],
        input=pcm_bytes,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.decode(errors="replace"))


def synthesize_speaker(voice, speaker_id, text, out_dir):
    """Synthesize `text` for one speaker and write an mp3. Returns out path."""
    out_path = os.path.join(out_dir, f"speaker_{speaker_id:03d}.mp3")
    # Resume support: skip a speaker that already has a non-empty mp3 so a
    # re-run only fills in what a previous interrupted run missed.
    if os.path.exists(out_path) and os.path.getsize(out_path) > 0:
        return out_path

    cfg = SynthesisConfig(
        speaker_id=speaker_id,
        length_scale=LENGTH_SCALE,
        noise_scale=NOISE_SCALE,
        noise_w_scale=NOISE_W,
    )
    pcm = bytearray()
    for chunk in voice.synthesize(text, cfg):
        pcm.extend(chunk.audio_int16_bytes)

    encode_mp3(bytes(pcm), voice.config.sample_rate, out_path)
    return out_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speakers", type=int, default=None,
                        help="Number of speakers to render (default: all).")
    parser.add_argument("--workers", type=int, default=4,
                        help="Parallel synthesis threads (default: 4).")
    parser.add_argument("--text", default=DEFAULT_TEXT,
                        help="Sample sentence to speak.")
    parser.add_argument("--out", default=os.path.expanduser("~/samples"),
                        help="Output directory (default: ~/samples).")
    args = parser.parse_args()

    voice_dir = find_voice_dir()
    model = os.path.join(voice_dir, "en_US-libritts_r-medium.onnx")
    config = os.path.join(voice_dir, "en_US-libritts_r-medium.onnx.json")

    voice = PiperVoice.load(model, config, use_cuda=False)
    num_speakers = voice.config.num_speakers
    total = num_speakers if args.speakers is None else min(args.speakers, num_speakers)

    os.makedirs(args.out, exist_ok=True)
    print(f"Model has {num_speakers} speakers. Rendering {total} to {args.out} "
          f"with {args.workers} workers.")

    done = 0
    done_lock = threading.Lock()
    failures = []

    def task(sid):
        return synthesize_speaker(voice, sid, args.text, args.out)

    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(task, sid): sid for sid in range(total)}
        for fut in as_completed(futures):
            sid = futures[fut]
            try:
                fut.result()
            except Exception as e:
                failures.append((sid, str(e)))
            with done_lock:
                done += 1
                if done % 20 == 0 or done == total:
                    print(f"  {done}/{total} done", flush=True)

    print(f"\nFinished. {total - len(failures)} succeeded, {len(failures)} failed.")
    for sid, err in failures:
        print(f"  speaker {sid}: {err}")


if __name__ == "__main__":
    main()
