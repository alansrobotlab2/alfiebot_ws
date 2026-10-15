"""
hearing — the ROS-free half of the ASR node: VAD segmentation, end-of-turn
detection, and the hold that glues a mid-sentence pause back together.

Ported from lloyd's ``agent-services/voice/`` (vad.py, turn.py and the worker's
hold loop). Nothing here imports rclpy, so the whole pipeline can be driven
from a WAV (see ``replay.py``) and unit-tested.

Pipeline, per 16 kHz frame:

  * ``SileroSegmenter`` closes a *segment* after ``min_silence_ms`` of silence.
    It keeps ``speech_pad_ms`` of pre-roll from before the onset (the old node
    started its buffer at the VAD's ``start`` event and clipped the first
    syllable) and a segment that hits ``max_utterance_ms`` continues straight
    into the next instead of being thrown away.
  * ``Turn1Mini`` (p99lab, Apache-2.0, 5.3M params, ~2 ms per 160 ms step on one
    core) is fed *every* frame with its state carried — its answer depends on
    the ~7 s before a close, so scoring an utterance from a zero state disagrees
    with the stream on 13-40% of closes (lloyd, 2026-10-09).
  * At a segment close the turn score decides: complete -> commit; unfinished
    -> hold, and glue whatever comes next onto it. A held turn is released as
    soon as the speaker is quiet and the score crosses (``hold_released``), at
    ``hold_timeout_ms`` if they simply trailed off, and never while they are
    still mid-utterance (releasing at the deadline mid-word cut people off).
    Everything fails open: no turn model means every close commits.

Time is counted in samples since the stream opened, so behaviour is
deterministic under replay.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

LOG = logging.getLogger('alfie_asr.hearing')

SAMPLE_RATE = 16000
#: Silero v5/v6 accept exactly 512 samples at 16 kHz (32 ms).
VAD_FRAME = 512
_VAD_CONTEXT = 64  # samples of the previous frame Silero is given


class SileroOnnx:
    """Silero VAD through onnxruntime directly — no torch.

    The ``silero-vad`` package wraps the same ONNX graph in torch; this is that
    wrapper's arithmetic (a (2, 1, 128) recurrent state and the last 64 samples
    of the previous frame prepended to each 512), lifted from lloyd where it was
    checked equal to the package to 1e-6.
    """

    def __init__(self, path):
        import onnxruntime as ort

        opts = ort.SessionOptions()
        opts.inter_op_num_threads = 1
        opts.intra_op_num_threads = 1
        self._session = ort.InferenceSession(
            str(path), sess_options=opts, providers=['CPUExecutionProvider'])
        self._sr = np.array(SAMPLE_RATE, dtype=np.int64)
        self.reset_states()

    def reset_states(self):
        self._state = np.zeros((2, 1, 128), dtype=np.float32)
        self._context = np.zeros((1, _VAD_CONTEXT), dtype=np.float32)

    def __call__(self, frame):
        x = np.concatenate([self._context,
                            np.asarray(frame, dtype=np.float32).reshape(1, -1)], axis=1)
        out, self._state = self._session.run(
            None, {'input': x, 'state': self._state, 'sr': self._sr})
        self._context = x[:, -_VAD_CONTEXT:]
        return float(out.reshape(-1)[0])


# --- segmentation ---------------------------------------------------------------

@dataclass
class Segment:
    """A closed span of speech."""

    audio: np.ndarray       # float32 @ 16 kHz, including the pads
    start_sample: int
    end_sample: int         # where the speech stopped (trailing silence excluded)
    reason: str             # "silence" | "max_duration"

    @property
    def duration_s(self):
        return self.audio.size / SAMPLE_RATE


@dataclass
class _SegState:
    triggered: bool = False
    silence_run: int = 0
    buf: list = field(default_factory=list)
    buf_len: int = 0
    start_sample: int = 0
    onset_sample: int = 0   # where Silero first said speech (after the pre-roll)


class SileroSegmenter:
    """Streaming speech segmentation: feed 512-sample frames, get Segments."""

    def __init__(self, model, threshold=0.45, min_silence_ms=380, speech_pad_ms=220,
                 min_utterance_ms=250, max_utterance_ms=30000):
        self._model = model
        self.threshold = float(threshold)
        # Silero's own hysteresis: leaving speech needs a clearly lower score
        # than entering it, so a brief dip mid-word does not close the segment.
        self.exit_threshold = max(0.0, self.threshold - 0.15)
        self._min_silence = int(min_silence_ms / 1000 * SAMPLE_RATE)
        self._pad = int(speech_pad_ms / 1000 * SAMPLE_RATE)
        self._min_utt = int(min_utterance_ms / 1000 * SAMPLE_RATE)
        self._max_utt = int(max_utterance_ms / 1000 * SAMPLE_RATE)
        self.cursor = 0
        self._st = _SegState()
        self._preroll = []
        self._preroll_len = 0
        self.last_prob = 0.0

    @property
    def in_speech(self):
        return self._st.triggered

    @property
    def silence_run(self):
        return self._st.silence_run

    def feed_frame(self, frame):
        """Feed one 512-sample float32 frame.

        Returns ``(onset, closed_at, segment)``: ``onset`` is True on the frame
        speech began; ``closed_at`` is the speech-end sample when a span of
        speech closed on silence (even one too short to keep), else None;
        ``segment`` is the kept Segment, else None.
        """
        self.cursor += frame.size
        try:
            p = self._model(np.ascontiguousarray(frame, dtype=np.float32))
        except Exception as e:  # never let one bad frame kill hearing
            LOG.warning('silero predict failed: %s', e)
            p = 0.0
        self.last_prob = p
        st = self._st

        if not st.triggered:
            self._preroll.append(frame)
            self._preroll_len += frame.size
            while self._preroll_len - self._preroll[0].size >= self._pad:
                self._preroll_len -= self._preroll[0].size
                self._preroll.pop(0)
            if p >= self.threshold:
                st.triggered = True
                st.buf = list(self._preroll)
                st.buf_len = self._preroll_len
                st.start_sample = self.cursor - self._preroll_len
                st.onset_sample = self.cursor - frame.size
                st.silence_run = 0
                self._preroll, self._preroll_len = [], 0
                return True, None, None
            return False, None, None

        st.buf.append(frame)
        st.buf_len += frame.size
        if p >= self.exit_threshold:
            st.silence_run = 0
        else:
            st.silence_run += frame.size

        closed = st.silence_run >= self._min_silence
        capped = st.buf_len >= self._max_utt
        if not (closed or capped):
            return False, None, None

        audio = np.concatenate(st.buf)
        speech_end = st.start_sample + audio.size - (st.silence_run if closed else 0)
        # Keep only `pad` of the closing silence: the rest is what the turn
        # decision needed, not something said, and it invites ASR hallucination.
        if closed and st.silence_run > self._pad:
            audio = audio[: audio.size - (st.silence_run - self._pad)]
        seg = Segment(audio=audio, start_sample=st.start_sample,
                      end_sample=speech_end,
                      reason='silence' if closed else 'max_duration')
        self._st = _SegState()
        if capped:
            # Mid-sentence by definition: continue at once rather than waiting
            # for a fresh onset, or the word after the cap is lost.
            self._st.triggered = True
            self._st.start_sample = st.start_sample + audio.size
            self._st.onset_sample = self._st.start_sample
        # Judge length on the speech itself, not the pads around it (with
        # 220 ms of pre-roll and tail, a pad-inclusive check never drops anything).
        closed_at = speech_end if closed else None
        if speech_end - st.onset_sample < self._min_utt and not capped:
            return False, closed_at, None
        return False, closed_at, seg


# --- end of turn -----------------------------------------------------------------

#: One 160 ms step of 16 kHz audio: the ONNX graph's fixed input length.
T1M_STEP = 2560
#: One 20 ms frame: the graph answers one probability per frame, 8 per step.
T1M_FRAME = 320


class Turn1Mini:
    """p99lab's turn-1-mini, streamed. Its scores are NOT calibrated: LiveKit's
    eot-bench 300 ms operating point is 0.06, and lloyd ships 0.06."""

    def __init__(self, model_path, threshold=0.06):
        import onnxruntime as ort

        self.threshold = float(threshold)
        so = ort.SessionOptions()
        so.intra_op_num_threads = 1
        so.inter_op_num_threads = 1
        self._session = ort.InferenceSession(
            str(Path(model_path).expanduser()), sess_options=so,
            providers=['CPUExecutionProvider'])
        self._state_inputs = [i for i in self._session.get_inputs() if i.name != 'audio']
        self._outputs = [o.name for o in self._session.get_outputs()]
        if self._outputs[0] != 'probs':
            raise RuntimeError(f'unexpected turn-1-mini outputs: {self._outputs}')
        self.reset()

    def reset(self):
        self._state = {i.name: np.zeros(i.shape, dtype=np.float32)
                       for i in self._state_inputs}
        self._buf = np.zeros(0, dtype=np.float32)
        self.latest = 0.0

    def _step(self, chunk, state):
        out = self._session.run(
            self._outputs, {'audio': chunk.reshape(1, T1M_STEP).astype(np.float32), **state})
        new = {n.replace('new_state_', 'state_'): v
               for n, v in zip(self._outputs[1:], out[1:])}
        return np.asarray(out[0]).reshape(-1), new

    def push(self, audio):
        """Feed any length of float32 16 kHz audio; updates ``latest``."""
        self._buf = np.concatenate([self._buf, np.asarray(audio, dtype=np.float32).reshape(-1)])
        while self._buf.size >= T1M_STEP:
            probs, self._state = self._step(self._buf[:T1M_STEP], self._state)
            self._buf = self._buf[T1M_STEP:]
            self.latest = float(probs[-1])

    def peek(self):
        """P(end of turn) at the most recent audio, including a partial step:
        pad with zeros and run on a copy of the state (the graph's own rule)."""
        rem = self._buf.size
        if rem < T1M_FRAME:
            return self.latest
        chunk = np.zeros(T1M_STEP, dtype=np.float32)
        chunk[:rem] = self._buf
        probs, _ = self._step(chunk, self._state)
        return float(probs[min(7, rem // T1M_FRAME - 1)])


# --- the pipeline ----------------------------------------------------------------

@dataclass
class HearingEvent:
    kind: str                       # "speech_start" | "speech_end" | "turn"
    sample: int                     # stream position the event refers to
    audio: Optional[np.ndarray] = None
    turn_prob: float = -1.0
    reason: str = ''
    #: Samples between the end of speech and the moment of this event (the
    #: closing silence plus any hold) — lets the node back out speech_end time.
    lag_samples: int = 0


class HearingPipeline:
    """Frames in, ``HearingEvent``s out. One per audio stream; not thread-safe."""

    def __init__(self, segmenter, turn=None, hold_timeout_ms=2200, hold_max_s=12.0):
        self.segmenter = segmenter
        self.turn = turn
        self._hold_timeout = int(hold_timeout_ms / 1000 * SAMPLE_RATE)
        self._hold_max = int(hold_max_s * SAMPLE_RATE)
        self._held = []             # list of Segment audio arrays
        self._held_len = 0
        self._held_end = 0          # speech-end sample of the last held segment
        self._held_deadline = None  # sample index; None = nothing held
        self._pending = np.zeros(0, dtype=np.float32)

    @property
    def holding(self):
        return self._held_deadline is not None

    def feed(self, samples):
        """Feed int16 or float32 16 kHz audio of any length."""
        x = np.asarray(samples)
        if x.dtype == np.int16:
            x = x.astype(np.float32) / 32768.0
        self._pending = np.concatenate([self._pending, x.astype(np.float32).reshape(-1)])
        events = []
        while self._pending.size >= VAD_FRAME:
            frame = self._pending[:VAD_FRAME]
            self._pending = self._pending[VAD_FRAME:]
            events.extend(self._feed_frame(frame))
        return events

    def _feed_frame(self, frame):
        events = []
        seg_ = self.segmenter
        if self.turn is not None:
            try:
                self.turn.push(frame)
            except Exception as e:
                LOG.warning('turn-1-mini failed (%s); disabling it (fail open)', e)
                self.turn = None
        onset, closed_at, seg = seg_.feed_frame(frame)
        now = seg_.cursor
        if onset:
            events.append(HearingEvent('speech_start', now))
        if closed_at is not None:
            events.append(HearingEvent('speech_end', closed_at))
        if seg is not None:
            events.extend(self._on_segment(seg, now))
        elif self.holding and not seg_.in_speech:
            events.extend(self._check_hold(now))
        return events

    def _score(self):
        if self.turn is None:
            return None
        try:
            return self.turn.peek()
        except Exception as e:
            LOG.warning('turn-1-mini peek failed (%s); disabling it (fail open)', e)
            self.turn = None
            return None

    def _on_segment(self, seg, now):
        self._held.append(seg.audio)
        self._held_len += seg.audio.size
        self._held_end = seg.end_sample
        if seg.reason == 'max_duration':
            # Still talking: keep holding; the next close decides.
            if self._held_deadline is None:
                self._held_deadline = now + self._hold_timeout
            return []
        p = self._score()
        if p is None:
            return [self._release(now, -1.0, 'no_turn_model')]
        if p >= self.turn.threshold:
            return [self._release(now, p, 'complete')]
        if self._held_len >= self._hold_max:
            return [self._release(now, p, 'hold_max')]
        self._held_deadline = now + self._hold_timeout
        LOG.debug('holding %.2fs turn (p=%.3f)', self._held_len / SAMPLE_RATE, p)
        return []

    def _check_hold(self, now):
        # Only consulted while the speaker is quiet: a deadline that passes
        # mid-utterance waits for that utterance to close.
        p = self._score()
        if p is not None and p >= self.turn.threshold:
            return [self._release(now, p, 'hold_released')]
        if now >= self._held_deadline:
            return [self._release(now, -1.0 if p is None else p, 'hold_timeout')]
        return []

    def _release(self, now, p, reason):
        audio = np.concatenate(self._held) if self._held else np.zeros(0, np.float32)
        ev = HearingEvent('turn', now, audio=audio, turn_prob=p, reason=reason,
                          lag_samples=max(0, now - self._held_end))
        self._held, self._held_len, self._held_deadline = [], 0, None
        return ev

    def flush(self):
        """Commit whatever is held (stream ending)."""
        if not self.holding:
            return []
        return [self._release(self.segmenter.cursor, -1.0, 'flush')]


def build_pipeline(vad_model_path, turn_model_path=None, *, vad_threshold=0.45,
                   min_silence_ms=None, speech_pad_ms=220, min_utterance_ms=250,
                   max_utterance_ms=30000, turn_threshold=0.06,
                   hold_timeout_ms=2200, hold_max_s=12.0, log=None):
    """Build a pipeline; the turn model is optional and fails open.

    ``min_silence_ms`` defaults to 380 with a turn model and 500 without: 380
    alone splits ~4 in 10 utterances at a breath, which only a turn model can
    glue back (lloyd's vad_eval gate).
    """
    turn = None
    if turn_model_path:
        try:
            if Path(turn_model_path).expanduser().exists():
                turn = Turn1Mini(turn_model_path, threshold=turn_threshold)
            elif log:
                log(f'turn-1-mini not found at {turn_model_path}; '
                    'committing on VAD silence alone')
        except Exception as e:
            if log:
                log(f'turn-1-mini failed to load ({e}); committing on VAD silence alone')
            turn = None
    if min_silence_ms is None or min_silence_ms <= 0:
        min_silence_ms = 380 if turn is not None else 500
    seg = SileroSegmenter(SileroOnnx(vad_model_path), threshold=vad_threshold,
                          min_silence_ms=min_silence_ms, speech_pad_ms=speech_pad_ms,
                          min_utterance_ms=min_utterance_ms,
                          max_utterance_ms=max_utterance_ms)
    return HearingPipeline(seg, turn, hold_timeout_ms=hold_timeout_ms,
                           hold_max_s=hold_max_s)
