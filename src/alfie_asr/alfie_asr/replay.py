"""
replay — feed WAV files through the same hearing pipeline the ASR node runs.

  ros2 run alfie_asr replay clip.wav [more.wav ...] [--asr] [--no-turn]
                                     [--turn-threshold 0.06] [--min-silence-ms 0]

Prints every VAD edge and committed turn (with its turn score and reason), so
a miss captured with the node's `capture_dir` can be reproduced and a threshold
change judged on real audio rather than by feel. WAVs must be 16 kHz mono
16-bit (what `capture_dir` writes). `--asr` also transcribes each turn with
Parakeet (loads the model; slow to start).
"""
import argparse
import sys
import wave

import numpy as np

from alfie_asr.hearing import SAMPLE_RATE, build_pipeline
from alfie_asr.model_manager import get_turn_model_path, get_vad_model_path


def read_wav(path):
    with wave.open(path, 'rb') as w:
        if w.getframerate() != SAMPLE_RATE or w.getnchannels() != 1 or w.getsampwidth() != 2:
            raise SystemExit(f'{path}: need 16 kHz mono 16-bit, got '
                             f'{w.getframerate()} Hz x{w.getnchannels()} '
                             f'{8 * w.getsampwidth()}-bit')
        return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('wavs', nargs='+')
    ap.add_argument('--asr', action='store_true', help='transcribe each turn')
    ap.add_argument('--no-turn', action='store_true', help='VAD silence only')
    ap.add_argument('--turn-threshold', type=float, default=0.06)
    ap.add_argument('--vad-threshold', type=float, default=0.45)
    ap.add_argument('--min-silence-ms', type=int, default=0, help='0 = auto')
    ap.add_argument('--tail-ms', type=int, default=3000,
                    help='silence appended so a final turn can close')
    a = ap.parse_args(argv)

    asr = None
    if a.asr:
        import onnx_asr
        from alfie_asr.model_manager import get_asr_model_dir
        asr = onnx_asr.load_model('istupakov/parakeet-tdt-0.6b-v2-onnx',
                                  path=str(get_asr_model_dir()), quantization='int8')

    for path in a.wavs:
        pipe = build_pipeline(
            get_vad_model_path(), None if a.no_turn else get_turn_model_path(),
            vad_threshold=a.vad_threshold, min_silence_ms=a.min_silence_ms,
            turn_threshold=a.turn_threshold, log=lambda m: print(f'  ! {m}'))
        audio = np.concatenate([read_wav(path),
                                np.zeros(int(a.tail_ms / 1000 * SAMPLE_RATE), np.int16)])
        print(f'{path}: {audio.size / SAMPLE_RATE:.2f}s '
              f'(turn model {"on" if pipe.turn else "off"})')
        events = []
        for i in range(0, audio.size, 512):
            events += pipe.feed(audio[i:i + 512])
        events += pipe.flush()
        for ev in events:
            t = ev.sample / SAMPLE_RATE
            if ev.kind == 'turn':
                line = (f'  {t:7.2f}s  TURN  {ev.audio.size / SAMPLE_RATE:.2f}s '
                        f'{ev.reason} p={ev.turn_prob:.3f} '
                        f'commit +{ev.lag_samples * 1000 // SAMPLE_RATE} ms after speech')
                if asr is not None:
                    line += f'  {asr.recognize(ev.audio, sample_rate=SAMPLE_RATE)!r}'
                print(line)
            else:
                print(f'  {t:7.2f}s  {ev.kind}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
