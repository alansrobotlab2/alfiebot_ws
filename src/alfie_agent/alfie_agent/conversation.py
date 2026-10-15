"""
conversation — the words a conversation is managed with, as opposed to the
words it is about.

Ported from lloyd (agent-services/voice/conversation.py) with Alfie's name, plus
the transcript-side wake helpers from lloyd's gate:

* **Backchannels are not turns.** "Yeah.", "Okay.", "Mm." while Alfie talks is
  the listener listening. Answering one cost lloyd 59 seconds of speech nobody
  asked for. The exception is an answer: after a question, "yeah" means yes, so
  only the pure acknowledgements ("mm", "I see") stay backchannels.
* **Closers end the conversation** ("that's all", "thanks Alfie") and, if Alfie
  is talking, stop him.
* **The transcript can wake Alfie.** An utterance that *opens* with a wake
  phrase (one leading "uh"/"um"/"oh" allowed, never two — "so I told Alfie" is
  the name, not an address) wakes him even when openWakeWord missed it. In
  lloyd this path caught 40 of 48 wake phrases against 17-23 for the acoustic
  model, with no false accepts.
* **A mention is not a request.** "Did Alfie finish?" sounds like "hi Alfie" to
  the acoustic model; a transcript naming him mid-sentence is dropped, while the
  name first or last ("what time is it, Alfie?") is still an address.
* **Echo.** A transcript that is mostly the words Alfie is saying is his own
  voice leaking past the AEC, not the user.
"""
from __future__ import annotations

import re

_WORD = re.compile(r"[a-z0-9']+")

NAME = "alfie"
#: How Parakeet tends to spell the name. Matched as whole words.
NAME_SPELLINGS = frozenset({"alfie", "alfy", "alfi", "alphie", "alphy", "elfie"})

#: Acknowledgements that answer nothing — never a turn, even after a question.
PURE_BACKCHANNELS = frozenset({
    "mm", "mmm", "mhm", "mmhmm", "hmm", "uhhuh", "uh huh",
    "huh", "ah", "oh", "i see", "gotcha", "got it", "cool", "nice", "wow",
    "interesting", "crazy", "ha", "haha", "right right",
    "makes sense", "fair enough", "good to know", "love it", "no worries",
    "no problem", "well done", "good job", "nice one", "good stuff",
})

#: Acknowledgements that ARE answers when Alfie just asked something.
ANSWERING_BACKCHANNELS = frozenset({
    "yeah", "yea", "yep", "yup", "okay", "ok", "right", "sure", "alright",
    "all right", "thanks", "thank you", "okay thanks", "ok thanks",
    "yeah okay", "okay cool", "yeah yeah", "okay okay",
    "great", "good", "perfect", "awesome", "excellent", "sweet", "fine",
    "sounds good", "all good", "very good", "very nice",
})

#: Words with no meaning of their own inside an acknowledgement: "yeah, that's
#: great" is "yeah great". Never "is" or "that" — "Is that right?" asks.
_ACK_FILLERS = frozenset({"that's", "thats", "so", "really", "very", "pretty",
                          "um", "uh", "well"}) | NAME_SPELLINGS

#: Said to end the conversation. Matched on the whole utterance, so "stop the
#: music" is a request, not a closer.
CLOSERS = frozenset({
    "stop", "stop it", "stop talking", "that's all", "thats all", "that is all",
    "that's it", "thats it", "that's it for now", "that's all for now",
    "never mind", "nevermind", "i'm done", "im done", "we're done",
    "thanks alfie", "thank you alfie", "thanks that's all", "thanks done",
    "thanks that's it", "okay that's all", "ok that's all", "goodbye",
    "bye", "bye alfie", "goodbye alfie", "that'll do", "that will do",
    "quiet", "be quiet", "shush", "hush", "go to sleep",
})

#: The subset that also stops speech mid-reply.
STOPPERS = frozenset({
    "stop", "stop it", "stop talking", "quiet", "be quiet", "shush", "hush",
    "enough", "never mind", "nevermind", "wait", "hold on", "hang on",
    "alfie stop", "okay stop", "ok stop",
})

#: One leading disfluency allowed before a wake phrase.
_LEAD_FILLERS = frozenset({"uh", "um", "oh", "er", "hmm", "ah", "okay", "ok"})
_WAKE_HEADS = frozenset({"hey", "hi", "hello", "okay", "ok", "yo"})


def normalize(text: str) -> str:
    """Lower-case words only, single-spaced: 'Yeah, okay.' -> 'yeah okay'."""
    t = (text or "").lower().replace("mm-hmm", "mmhmm").replace("uh-huh", "uhhuh")
    t = t.replace("’", "'")
    return " ".join(_WORD.findall(t))


def words(text: str) -> list[str]:
    return normalize(text).split()


def _canon_name(ws: list[str]) -> list[str]:
    return [NAME if w in NAME_SPELLINGS else w for w in ws]


def _ack_phrases(ws: list[str], vocab: frozenset):
    """Split words into vocabulary phrases, longest first; None if any word
    belongs to none."""
    out = []
    i = 0
    while i < len(ws):
        for k in (3, 2, 1):
            cand = " ".join(ws[i:i + k])
            if len(ws[i:i + k]) == k and cand in vocab:
                out.append(cand)
                i += k
                break
        else:
            return None
    return out


def ends_with_question(text: str) -> bool:
    """Did this spoken text end by asking something?"""
    t = (text or "").rstrip().rstrip("\"'”’)")
    return t.endswith("?")


def is_backchannel(text: str, after_question: bool = False) -> bool:
    """A short acknowledgement rather than something to answer."""
    if ends_with_question(text):
        return False
    n = normalize(text)
    if not n:
        return False
    if n in PURE_BACKCHANNELS:
        return True
    if not after_question and n in ANSWERING_BACKCHANNELS:
        return True
    kept = [w for w in n.split() if w not in _ACK_FILLERS]
    if not kept or len(kept) > 3:
        return False
    phrases = _ack_phrases(kept, PURE_BACKCHANNELS | ANSWERING_BACKCHANNELS)
    if phrases is None:
        return False
    if after_question:
        return all(ph in PURE_BACKCHANNELS for ph in phrases)
    return True


_POLITE_TAIL = (" thanks", " thank you", " alfie", " please")
_POLITE_HEAD = ("thanks ", "thank you ", "okay ", "ok ", "alright ", "all right ",
                "alfie ")


def _strip_polite(n: str) -> str:
    changed = True
    while changed and n:
        changed = False
        for tail in _POLITE_TAIL:
            if n.endswith(tail):
                n, changed = n[: -len(tail)], True
        for head in _POLITE_HEAD:
            if n.startswith(head):
                n, changed = n[len(head):], True
    return n


def is_closer(text: str) -> bool:
    n = " ".join(_canon_name(words(text)))
    if n in CLOSERS:
        return True
    core = _strip_polite(n)
    return bool(core) and core != n and core in CLOSERS


def is_stopper(text: str) -> bool:
    n = " ".join(_canon_name(words(text)))
    if n.startswith(NAME + " ") and n[len(NAME) + 1:] in STOPPERS:
        return True
    if n.endswith(" " + NAME) and n[:-len(NAME) - 1] in STOPPERS:
        return True
    return n in STOPPERS


def split_wake(text: str):
    """If ``text`` opens with a wake phrase, return the command after it
    (possibly ""); otherwise None.

    "Hey Alfie, what time is it?" -> "what time is it?"; "Um, Alfie." -> "";
    "So I told Alfie" -> None (two words before the name, not an address).
    """
    raw = (text or "").split()
    ws = _canon_name([re.sub(r"[^a-z0-9']", "", w.lower()) for w in raw])
    i = 0
    if i < len(ws) and ws[i] in _LEAD_FILLERS and not (
            ws[i] in _WAKE_HEADS and i + 1 < len(ws) and ws[i + 1] == NAME):
        i += 1
    if i < len(ws) and ws[i] in _WAKE_HEADS and i + 1 < len(ws) and ws[i + 1] == NAME:
        i += 1
    if i < len(ws) and ws[i] == NAME:
        return " ".join(raw[i + 1:]).lstrip(" ,.!?;:-").strip()
    return None


def mentions_name_midsentence(text: str) -> bool:
    """True when the name appears, but neither first (after a wake head or one
    filler) nor last: "did Alfie finish?", "I told Alfie", "Alfie's battery"."""
    ws = _canon_name(words(text))
    raw = (text or "").lower()
    if re.search(r"\balfie's\b", raw) and split_wake(text) is None:
        return True
    if NAME not in ws:
        return False
    if split_wake(text) is not None:
        return False
    return ws[-1] != NAME


def echo_overlap(heard: str, spoken: str) -> float:
    """Fraction of the transcript's words found in what Alfie was saying.

    Own-voice leakage past the AEC comes back as a fragment of the reply, so
    a high overlap means echo. Short transcripts are judged leniently by the
    caller (one word overlapping proves nothing).
    """
    hw = words(heard)
    if not hw:
        return 0.0
    sw = set(words(spoken))
    if not sw:
        return 0.0
    return sum(1 for w in hw if w in sw) / len(hw)


def longest_shared_run(heard: str, spoken: str) -> int:
    """Length of the longest run of consecutive words the transcript shares
    with what Alfie said. Echo comes back as a contiguous fragment of the reply;
    a user reusing a few of its words ("what time is it" after "...once upon a
    time...") does not."""
    hw, sw = words(heard), words(spoken)
    best = 0
    prev = [0] * (len(sw) + 1)
    for i in range(1, len(hw) + 1):
        cur = [0] * (len(sw) + 1)
        for j in range(1, len(sw) + 1):
            if hw[i - 1] == sw[j - 1]:
                cur[j] = prev[j - 1] + 1
                best = max(best, cur[j])
        prev = cur
    return best


def is_echo(heard: str, spoken: str, min_overlap: float = 0.6) -> bool:
    """Is this transcript Alfie's own voice leaking past the AEC?

    Most of its words must be in the reply AND they must include a run of at
    least three consecutive reply words (or the whole transcript, if shorter).
    The words a listener uses to stop him, and a bare wake, are never echo.
    """
    n = len(words(heard))
    # A bare wake ("Alfie.") is never echo; "Alfie lived in a workshop" can be,
    # when the reply itself named him.
    if n == 0 or is_stopper(heard) or is_closer(heard) or split_wake(heard) == "":
        return False
    return (echo_overlap(heard, spoken) >= min_overlap
            and longest_shared_run(heard, spoken) >= min(3, n))


#: Spoken before a slow tool runs, when the model said nothing first — the
#: "One moment." of lloyd's filler, but saying what Alfie is doing (contextual
#: fillers raise rapport as well as perceived speed).
TOOL_FILLERS = {
    "look": "Let me take a look.",
    "identify_room": "Let me look around.",
    "learn_room": "Let me take that in.",
    "vault_search": "Let me check my notes.",
    "vault_read": "Let me check my notes.",
    "vault_write": "Let me write that down.",
}
GENERIC_FILLER = "One moment."
PROGRESS_LINE = "Still working on it."
