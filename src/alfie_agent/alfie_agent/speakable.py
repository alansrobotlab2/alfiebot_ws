"""Streamed text -> speakable clauses.

Ported from lloyd (agent-services/voice/speakable.py). The agent feeds the
model's spoken text in as it streams and publishes each clause to TTS the
moment it is complete, so Alfie starts talking one clause behind the model
instead of one whole answer behind it.

The reply used to reach TTS as one finished message, rewritten for speech by
the secondary model first. Speaking it while it streams means two jobs move
here, and both have to work on text that arrives a few characters at a time:

* **Where to cut.** A clause goes to TTS the moment it is complete, so the
  first sound is one sentence behind the model rather than one answer. The
  *first* clause may also be cut at a comma once it is long enough, because
  that is the one the listener is waiting on; later clauses are cut only at
  sentence ends and very short ones are held and joined to the next, since a
  string of three-word syntheses sounds like a list being read.
* **What not to say.** The voice turn asks the model for plain speech (see
  `prompt_builder._SPOKEN_RULES`), and this is the belt to that
  brace: fenced code is skipped whole, table rows are dropped, headings and
  list markers are stripped, inline markdown is unwrapped, URLs are removed,
  and everything after a line that is only `---` is the written part of the
  reply and is not spoken at all.

Block constructs (fences, dividers, tables, list markers) are only knowable at
the start of a line, so a partial line that *could* be one is held until it
can be classified. A partial line that cannot be one flows straight through —
otherwise a long first sentence would wait for its newline.
"""
from __future__ import annotations

import re

_ABBREV = {
    "e.g", "i.e", "etc", "vs", "mr", "mrs", "ms", "dr", "st", "no", "approx",
    "fig", "inc", "ltd", "jr", "sr", "a.m", "p.m",
}

_DIVIDERS = {"---", "***", "___"}
_BULLET = re.compile(r"^([-*+•])\s+")
_NUMBERED = re.compile(r"^\d{1,3}[.)]\s+")
_SENTENCE_END = re.compile(r"[.!?…](?:[\"'”’)\]]*)(?=\s)")
_SOFT = re.compile(r"[,;:](?=\s)|\s[—–-]\s")

_URL = re.compile(r"(?:https?://|www\.)\S+", re.IGNORECASE)
_IMAGE = re.compile(r"!\[([^\]]*)\]\([^)]*\)")
_LINK = re.compile(r"\[([^\]]+)\]\([^)]*\)")
_BOLD = re.compile(r"(\*\*|__)(.+?)\1")
_ITALIC = re.compile(r"(?<![\w*])([*_])(?!\s)(.+?)(?<!\s)\1(?![\w*])")
_CODE = re.compile(r"`([^`]*)`")
_SPACES = re.compile(r"\s+")


def clean_inline(text: str) -> str:
    """Unwrap inline markdown and drop what cannot be said."""
    t = _IMAGE.sub(r"\1", text)
    t = _LINK.sub(r"\1", t)
    t = _URL.sub("", t)
    t = _BOLD.sub(r"\2", t)
    t = _ITALIC.sub(r"\2", t)
    t = _CODE.sub(r"\1", t)
    t = t.replace("`", "")
    t = t.lstrip("#> ").strip()
    t = _SPACES.sub(" ", t)
    # A clause that was only markup, or only punctuation, is nothing to say.
    if not re.search(r"[A-Za-z0-9]", t):
        return ""
    return t


def _maybe_block(head: str) -> bool:
    """Could this *incomplete* line still turn out to be a block construct?"""
    if not head:
        return True
    if head.startswith(("`", "|")):
        return True
    if any(d.startswith(head) for d in _DIVIDERS):
        return True
    # "- " / "1. " decide on their second or third character.
    if head[0] in "-*+•" and len(head) < 2:
        return True
    if re.fullmatch(r"\d{1,3}[.)]?", head):
        return True
    return False


class ClauseStream:
    """Feed deltas; get back clauses ready to speak."""

    def __init__(self, first_soft_chars: int = 20, min_chars: int = 24,
                 max_chars: int = 240) -> None:
        self.first_soft_chars = first_soft_chars
        self.min_chars = min_chars
        self.max_chars = max_chars
        self._raw = ""
        self._mid_line = False
        self._pending = ""
        self._held = ""
        self._in_fence = False
        self._emitted_any = False
        #: A `---` line was seen; the rest of the reply is for the chat.
        self.stopped = False
        #: Code was skipped — worth one spoken pointer at the end.
        self.skipped_code = False

    # ── ingestion ────────────────────────────────────────────────────────

    def feed(self, delta: str) -> list[str]:
        if self.stopped or not delta:
            return []
        self._raw += delta
        self._ingest(final=False)
        return self._extract(final=False)

    def flush(self) -> list[str]:
        if not self.stopped:
            if self._raw and not self._raw.endswith("\n"):
                self._raw += "\n"
            self._ingest(final=True)
        return self._extract(final=True)

    def _ingest(self, final: bool) -> None:
        while self._raw and not self.stopped:
            nl = self._raw.find("\n")
            if self._in_fence:
                if nl < 0:
                    return
                line, self._raw = self._raw[:nl], self._raw[nl + 1:]
                if line.strip().startswith("```"):
                    self._in_fence = False
                continue
            if self._mid_line:
                if nl < 0:
                    self._pending += self._raw
                    self._raw = ""
                    return
                self._pending += self._raw[:nl] + "\n"
                self._raw = self._raw[nl + 1:]
                self._mid_line = False
                continue
            line = self._raw if nl < 0 else self._raw[:nl]
            head = line.strip()
            if nl < 0 and not final and _maybe_block(head):
                return
            if head.startswith("```"):
                self._in_fence = True
                self.skipped_code = True
                self._raw = "" if nl < 0 else self._raw[nl + 1:]
                continue
            if head in _DIVIDERS:
                self.stopped = True
                self._raw = ""
                return
            if head.startswith("|"):
                if nl < 0:
                    return  # wait for the row to finish, then drop it
                self._raw = self._raw[nl + 1:]
                continue
            # Classify on the stripped line, but keep a partial line's trailing
            # space: the next delta usually starts the next word, and "The "
            # + "build" must not arrive at TTS as "Thebuild".
            lead = line.lstrip()
            if lead.startswith("#"):
                lead = lead.lstrip("#").lstrip()
            body = _BULLET.sub("", _NUMBERED.sub("", lead))
            if nl >= 0:
                body = body.rstrip()
            if nl < 0:
                self._pending += body
                self._raw = ""
                self._mid_line = True
                return
            # A structural line ends a clause whether or not it has a full stop.
            self._pending += body + "\n"
            self._raw = self._raw[nl + 1:]

    # ── segmentation ─────────────────────────────────────────────────────

    def _boundary(self) -> int:
        """Index just past the first clause boundary in `_pending`, or -1."""
        p = self._pending
        nl = p.find("\n")
        best = nl + 1 if nl >= 0 else -1
        for m in _SENTENCE_END.finditer(p):
            end = m.end()
            if best >= 0 and end >= best:
                break
            word = re.findall(r"[\w.]+$", p[:m.start()])
            if word and word[-1].lower().rstrip(".") in _ABBREV:
                continue
            # "3.5" never matches (no whitespace after the dot); "U.S. and"
            # would, and is rare enough in speech to accept.
            best = end
            break
        if best < 0 and not self._emitted_any and len(p) >= self.first_soft_chars:
            for m in _SOFT.finditer(p):
                if m.end() >= self.first_soft_chars // 2:
                    return m.end()
        if best < 0 and len(p) >= self.max_chars:
            cut = max(p.rfind(",", 0, self.max_chars), p.rfind(" ", 0, self.max_chars))
            return (cut + 1) if cut > 0 else self.max_chars
        return best

    def _extract(self, final: bool) -> list[str]:
        out: list[str] = []
        while True:
            b = self._boundary()
            if b < 0:
                break
            clause, self._pending = self._pending[:b], self._pending[b:].lstrip()
            self._offer(clean_inline(clause), out)
        if final:
            rest = clean_inline(self._pending)
            self._pending = ""
            self._offer(rest, out)
            if self._held:
                out.append(self._held)
                self._held = ""
        return out

    def _offer(self, clause: str, out: list[str]) -> None:
        if not clause:
            return
        if self._held:
            clause = f"{self._held} {clause}"
            self._held = ""
        if self._emitted_any and len(clause) < self.min_chars:
            self._held = clause
            return
        self._emitted_any = True
        out.append(clause)
