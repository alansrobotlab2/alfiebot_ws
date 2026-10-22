"""
timeline — where one spoken turn's time went, as a single ``[latency]`` line.

lloyd's ``voice/timeline.py`` in Alfie's seams (pipecat's LatencyBreakdown).
Every mark is a wall-clock time (``time.time()``), so the ASR node's
``speech_end`` — stamped on another process — lines up with the agent's own:

  speech_end   the speaker stopped (from ASRResult)
  asr_done     the transcript arrived at the agent
  turn_start   the agent started the turn (after gating)
  prefetch     the per-turn memory prefetch finished (prefetch.py)
  first_delta  the model's first token of spoken text
  first_clause the first clause was published to TTS
  first_audio  TTS started playing this turn's audio
  done         the turn finished generating

The line prints each stage, in the order it happened, as a delta from the
previous one, plus the headline ``eos->audio``: from the end of speech to the
first sound (a filler counts — it is what the listener heard — and the line
notes it).
"""
from __future__ import annotations

import time

STAGES = ("speech_end", "asr_done", "turn_start", "prefetch", "first_delta",
          "first_clause", "first_audio", "done")


class TurnTimeline:
    def __init__(self, speech_end: float | None = None):
        self.marks: dict[str, float] = {}
        if speech_end:
            self.marks["speech_end"] = float(speech_end)
        self.notes: list[str] = []

    def mark(self, stage: str, t: float | None = None) -> None:
        """Record a stage once; later marks of the same stage are ignored."""
        if stage not in self.marks:
            self.marks[stage] = time.time() if t is None else t

    def note(self, text: str) -> None:
        self.notes.append(text)

    def eos_to_audio(self) -> float | None:
        a, b = self.marks.get("speech_end"), self.marks.get("first_audio")
        return None if a is None or b is None else b - a

    def format(self) -> str:
        parts = []
        prev = None
        # Chronological: a filler can put first_clause/first_audio ahead of
        # first_delta, and the line should read in the order things happened.
        order = {st: i for i, st in enumerate(STAGES)}
        for st, t in sorted(self.marks.items(), key=lambda kv: (kv[1], order.get(kv[0], 99))):
            if prev is None:
                parts.append(st)
            else:
                parts.append(f"{st} +{(t - prev) * 1000:.0f}ms")
            prev = t
        total = self.eos_to_audio()
        head = f"eos->audio {total:.2f}s | " if total is not None else ""
        tail = f" | {'; '.join(self.notes)}" if self.notes else ""
        return f"[latency] {head}{' -> '.join(parts)}{tail}"
