"""HearingPipeline: pre-roll, segment closes, the end-of-turn hold and release.

The VAD and turn models are faked so the state machine is tested exactly: a
frame is "speech" when it is non-zero, and the turn score is whatever the test
sets. A last test runs the real ONNX models when they are on disk.
"""
import os

import numpy as np
import pytest

from alfie_asr.hearing import (SAMPLE_RATE, VAD_FRAME, HearingPipeline,
                               SileroSegmenter, build_pipeline)


class FakeVad:
    def __call__(self, frame):
        return 0.9 if np.abs(frame).max() > 0 else 0.0


class FakeTurn:
    threshold = 0.06

    def __init__(self):
        self.score = 0.0

    def push(self, audio):
        pass

    def peek(self):
        return self.score


def speech(s):
    return np.full(int(s * SAMPLE_RATE), 0.1, np.float32)


def silence(s):
    return np.zeros(int(s * SAMPLE_RATE), np.float32)


def run(pipe, *chunks):
    events = []
    for c in chunks:
        for i in range(0, c.size, VAD_FRAME):
            events += pipe.feed(c[i:i + VAD_FRAME])
    return events


def make(turn=None, **kw):
    seg = SileroSegmenter(FakeVad(), min_silence_ms=380, speech_pad_ms=220, **kw)
    return HearingPipeline(seg, turn, hold_timeout_ms=2200, hold_max_s=12.0)


def turns(events):
    return [e for e in events if e.kind == 'turn']


def test_preroll_keeps_audio_from_before_the_onset():
    pipe = make()
    ev = run(pipe, silence(1.0), speech(1.0), silence(1.0))
    (t,) = turns(ev)
    # 220 ms of pre-roll + 1 s of speech + 220 ms of the closing silence kept.
    assert abs(t.audio.size / SAMPLE_RATE - 1.44) < 0.07
    assert t.reason == 'no_turn_model'


def test_onset_and_close_are_reported():
    pipe = make()
    kinds = [e.kind for e in run(pipe, silence(0.5), speech(0.6), silence(0.6))]
    assert kinds == ['speech_start', 'speech_end', 'turn']


def test_a_complete_turn_commits_at_the_close():
    turn = FakeTurn()
    turn.score = 0.5
    pipe = make(turn)
    (t,) = turns(run(pipe, speech(1.0), silence(0.5)))
    assert t.reason == 'complete'
    assert abs(t.lag_samples / SAMPLE_RATE - 0.38) < 0.04


def test_an_unfinished_turn_is_held_and_glued_to_what_follows():
    turn = FakeTurn()
    pipe = make(turn)
    ev = run(pipe, speech(1.0), silence(0.6))
    assert not turns(ev) and pipe.holding
    turn.score = 0.5   # the continuation finishes the thought
    (t,) = turns(run(pipe, speech(0.8), silence(0.5)))
    assert t.reason == 'complete'
    assert t.audio.size / SAMPLE_RATE > 1.8   # both halves, one turn


def test_a_hold_is_released_when_the_score_crosses_in_silence():
    turn = FakeTurn()
    pipe = make(turn)
    run(pipe, speech(1.0), silence(0.5))
    assert pipe.holding
    turn.score = 0.07
    (t,) = turns(run(pipe, silence(0.1)))
    assert t.reason == 'hold_released'


def test_a_hold_times_out_but_never_mid_utterance():
    turn = FakeTurn()
    pipe = make(turn)
    run(pipe, speech(1.0), silence(0.5))
    # They start talking again before the deadline and keep going past it.
    ev = run(pipe, silence(1.0), speech(3.0))
    assert not turns(ev)
    (t,) = turns(run(pipe, silence(0.5)) + run(pipe, silence(2.5)))
    assert t.reason == 'hold_timeout'


def test_a_capped_segment_continues_instead_of_being_dropped():
    pipe = make(max_utterance_ms=2000)
    ev = run(pipe, speech(3.0), silence(0.6))
    (t,) = turns(ev)
    assert t.audio.size / SAMPLE_RATE > 2.9


def test_too_short_is_dropped_but_still_closes_the_vad_edge():
    pipe = make()
    ev = run(pipe, speech(0.1), silence(0.6))
    assert not turns(ev)
    assert [e.kind for e in ev] == ['speech_start', 'speech_end']


VAD_PATH = os.path.expanduser('~/.alfie_asr/models/silero_vad.onnx')
TURN_PATH = os.path.expanduser('~/.alfie_asr/models/turn-1-mini/onnx/turn-1-mini.step.onnx')


@pytest.mark.skipif(not (os.path.exists(VAD_PATH) and os.path.exists(TURN_PATH)),
                    reason='models not downloaded')
def test_real_models_load_and_run():
    pipe = build_pipeline(VAD_PATH, TURN_PATH)
    assert pipe.turn is not None
    assert pipe.segmenter._min_silence == int(0.38 * SAMPLE_RATE)
    rng = np.random.default_rng(0)
    run(pipe, (rng.standard_normal(SAMPLE_RATE) * 0.001).astype(np.float32))
    assert 0.0 <= pipe.turn.latest <= 1.0
