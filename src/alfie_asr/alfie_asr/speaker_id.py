"""
speaker_id — who said this? Voiceprints from 3D-Speaker's CAM++ on CPU.

Each committed utterance is embedded (80-band Kaldi fbank, mean-normalized ->
CAM++ ONNX -> 512-d, L2-normalized; ~25 ms on the Orin's CPU) and compared by
cosine similarity against enrolled voiceprints. A match needs ``min_audio_s`` of
speech, a score >= ``threshold`` and a ``margin`` over the runner-up — short
words ("stop") and close calls stay anonymous rather than guessed.

Enrollment uses the last few utterances (heard within ``enroll_window_s``),
which is what "Alfie, learn my voice — I'm Alan" naturally provides; with three
or more, any that don't resemble the rest (another voice) are dropped. A re-enroll
folds new audio into the existing voiceprint (count-weighted mean). Voiceprints
live in a JSON file outside the vault (~/.alfie_asr/voiceprints.json): they are
biometric data and the vault may be shared.

Validated 2026-10-22 on LibriTTS-R voices through Piper: enrolled from three
utterances, 11/12 test utterances right (one true speaker at 0.49, just under the
threshold), no false matches (unknown voices scored up to 0.44).
"""
import json
import threading
import time
from pathlib import Path

import numpy as np

MODEL_URL = ("https://github.com/k2-fsa/sherpa-onnx/releases/download/"
             "speaker-recongition-models/3dspeaker_speech_campplus_sv_en_voxceleb_16k.onnx")
SAMPLE_RATE = 16000


def _mel(f):
    return 1127.0 * np.log1p(np.asarray(f) / 700.0)


def _mel_banks(num_bins=80, n_fft=512, low=20.0, high=SAMPLE_RATE / 2):
    """Kaldi's triangular mel filterbank (torchaudio.compliance.kaldi.get_mel_banks)."""
    n_freq = n_fft // 2
    fft_mel = _mel(np.arange(n_freq) * SAMPLE_RATE / n_fft)
    lo, hi = _mel(low), _mel(high)
    delta = (hi - lo) / (num_bins + 1)
    left = lo + np.arange(num_bins)[:, None] * delta
    center, right = left + delta, left + 2 * delta
    up = (fft_mel - left) / (center - left)
    down = (right - fft_mel) / (right - center)
    banks = np.maximum(0.0, np.minimum(up, down))
    return np.pad(banks, ((0, 0), (0, 1)))          # n_fft/2 + 1 columns


_BANKS = None


def fbank(audio):
    """80-band Kaldi fbank of 16 kHz float audio, mean-normalized (CAM++ input).

    numpy port of ``torchaudio.compliance.kaldi.fbank`` with its defaults (25 ms
    povey window, 10 ms shift, snip edges, DC removal, 0.97 pre-emphasis, power
    spectrum, log mel), dither 0. Plain numpy on purpose: the ASR node must not
    import torch (cuBLAS clash with the GPU ASR model; see parakeet_asr_node).
    """
    global _BANKS
    x = np.asarray(audio, dtype=np.float64)
    win, hop, n_fft = 400, 160, 512
    if len(x) < win:
        return np.zeros((0, 80), dtype=np.float32)
    n = 1 + (len(x) - win) // hop
    frames = np.lib.stride_tricks.sliding_window_view(x, win)[::hop][:n].copy()
    frames -= frames.mean(axis=1, keepdims=True)
    frames[:, 1:] -= 0.97 * frames[:, :-1]
    frames[:, 0] -= 0.97 * frames[:, 0]
    frames *= np.power(0.5 - 0.5 * np.cos(2 * np.pi * np.arange(win) / (win - 1)), 0.85)
    power = np.abs(np.fft.rfft(frames, n=n_fft)) ** 2
    if _BANKS is None:
        _BANKS = _mel_banks()
    feats = np.log(np.maximum(power @ _BANKS.T, np.finfo(np.float32).eps))
    feats -= feats.mean(axis=0, keepdims=True)
    return feats.astype(np.float32)


def normalize(v):
    v = np.asarray(v, dtype=np.float32)
    n = float(np.linalg.norm(v))
    return v / n if n > 0 else v


class SpeakerID:
    def __init__(self, model_path, store_path, *, threshold=0.5, margin=0.08,
                 min_audio_s=1.0, enroll_window_s=90.0, enroll_min_s=3.0,
                 embed_fn=None):
        """``embed_fn(audio) -> vector`` overrides the ONNX model (tests)."""
        self.store_path = Path(store_path).expanduser()
        self.threshold = threshold
        self.margin = margin
        self.min_audio_s = min_audio_s
        self.enroll_window_s = enroll_window_s
        self.enroll_min_s = enroll_min_s
        self._lock = threading.Lock()
        self._recent = []          # (monotonic, seconds, embedding)
        self._prints = {}          # name -> {"emb": np.array, "n": int}
        if embed_fn is not None:
            self._embed = embed_fn
        else:
            import onnxruntime as ort
            self._sess = ort.InferenceSession(str(model_path),
                                              providers=["CPUExecutionProvider"])
            self._embed = lambda a: self._sess.run(
                None, {"x": fbank(a)[None]})[0][0]
        self._load()

    # --- persistence ------------------------------------------------------------

    def _load(self):
        try:
            data = json.loads(self.store_path.read_text())
        except (OSError, json.JSONDecodeError):
            return
        for name, v in data.items():
            self._prints[name] = {"emb": normalize(v["emb"]), "n": int(v.get("n", 1))}

    def _save(self):
        self.store_path.parent.mkdir(parents=True, exist_ok=True)
        data = {k: {"emb": [round(float(x), 6) for x in v["emb"]], "n": v["n"]}
                for k, v in self._prints.items()}
        tmp = self.store_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data))
        tmp.chmod(0o600)
        tmp.replace(self.store_path)

    def names(self):
        with self._lock:
            return sorted(self._prints)

    # --- identify -----------------------------------------------------------------

    def identify(self, audio):
        """Embed ``audio`` (16 kHz float) and return ``(name or "", score)``.

        Every utterance long enough is remembered for enrollment.
        """
        seconds = len(audio) / SAMPLE_RATE
        if seconds < self.min_audio_s:
            return "", 0.0
        emb = normalize(self._embed(audio))
        now = time.monotonic()
        with self._lock:
            self._recent = [r for r in self._recent
                            if now - r[0] < self.enroll_window_s][-6:]
            self._recent.append((now, seconds, emb))
            scored = sorted(((float(emb @ v["emb"]), k)
                             for k, v in self._prints.items()), reverse=True)
        if not scored:
            return "", 0.0
        best, name = scored[0]
        runner = scored[1][0] if len(scored) > 1 else -1.0
        if best >= self.threshold and best - runner >= self.margin:
            return name, best
        return "", best

    # --- enroll -------------------------------------------------------------------

    def enroll(self, name):
        """Fold the recent utterances into ``name``'s voiceprint."""
        name = " ".join((name or "").split())[:40]
        if not name:
            return False, "a name is required"
        now = time.monotonic()
        with self._lock:
            recent = [r for r in self._recent if now - r[0] < self.enroll_window_s]
            # Someone else may have spoken in the window: with 3+ utterances,
            # drop any that don't resemble the rest (leave-one-out mean).
            if len(recent) >= 3:
                embs = np.array([r[2] for r in recent])
                total_emb = embs.sum(axis=0)
                recent = [r for r, e in zip(recent, embs)
                          if float(normalize(total_emb - e) @ e) >= 0.3]
            total = sum(r[1] for r in recent)
            if total < self.enroll_min_s:
                return False, (f"I need about {self.enroll_min_s:.0f} seconds of "
                               f"speech first; I have {total:.1f}. Talk to me a "
                               "little more, then ask again.")
            new = normalize(np.mean([r[2] for r in recent], axis=0))
            old = self._prints.get(name)
            if old:
                n = old["n"]
                new = normalize(old["emb"] * n + new * len(recent))
                count = n + len(recent)
            else:
                count = len(recent)
            self._prints[name] = {"emb": new, "n": count}
            self._recent = []
            self._save()
        return True, f"learned {name}'s voice from {len(recent)} utterance(s)"

    def forget(self, name):
        with self._lock:
            match = [k for k in self._prints if k.lower() == (name or "").lower()]
            if not match:
                return False, f"I don't know a voice called {name}"
            del self._prints[match[0]]
            self._save()
        return True, f"forgot {match[0]}'s voice"
