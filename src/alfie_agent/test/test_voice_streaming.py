"""Streaming speech: clause cutting, the spoken-text filter, and harness hooks."""
from alfie_agent import harness
from alfie_agent.harness import SpokenFilter
from alfie_agent.speakable import ClauseStream
from alfie_agent.timeline import TurnTimeline


def _drip(text, size=3):
    return [text[i:i + size] for i in range(0, len(text), size)]


def _clauses(text, size=3):
    cs = ClauseStream()
    out = []
    for piece in _drip(text, size):
        out += cs.feed(piece)
    return out + cs.flush()


# --- ClauseStream ---------------------------------------------------------------

def test_sentences_come_out_whatever_the_slicing():
    text = "The can is on the table. I can see it clearly from here. Want me to grab it?"
    for size in (1, 2, 5, 50):
        assert _clauses(text, size) == [
            "The can is on the table.",
            "I can see it clearly from here.",
            "Want me to grab it?",
        ]


def test_first_clause_is_cut_early_at_a_comma():
    cs = ClauseStream()
    got = []
    for piece in _drip("The forecast for tomorrow, as far as I can tell, looks sunny"):
        got += cs.feed(piece)
    # Cut at the first comma at least 10 chars in once 20 chars have arrived,
    # long before the sentence ends.
    assert got[0] == "The forecast for tomorrow,"


def test_short_later_clauses_are_joined():
    assert _clauses("Here is the answer to that. Yes. It is four.") == [
        "Here is the answer to that.", "Yes. It is four."]


def test_markdown_is_not_spoken():
    out = _clauses("**Sure.** Here:\n- one thing\n- two things\n```\ncode\n```\nDone now.")
    joined = " ".join(out)
    assert "*" not in joined and "code" not in joined and "-" not in joined
    assert "one thing" in joined and "Done now." in joined


def test_abbreviations_and_decimals_do_not_cut():
    assert _clauses("It is 3.5 meters from the door, e.g. by the shelf. Okay then, see you.") == [
        "It is 3.5 meters from the door,", "e.g. by the shelf. Okay then, see you."]
    assert _clauses("The can is about 3.5 meters away. Dr. Smith left it there.") == [
        "The can is about 3.5 meters away.", "Dr. Smith left it there."]


# --- SpokenFilter ---------------------------------------------------------------

def _filter(text, size=2):
    f = SpokenFilter()
    out = "".join(f.feed(p) for p in _drip(text, size))
    return out + f.flush(), f


def test_think_blocks_are_dropped_even_split_across_deltas():
    out, _ = _filter("<think>\nhmm let me see\n</think>\n\nIt is noon.")
    assert out == "It is noon."


def test_stray_close_think_and_leading_whitespace():
    out, _ = _filter("\n\n</think>\n\nHello there.")
    assert out == "Hello there."


def test_text_before_a_tool_call_is_spoken_and_the_call_is_not():
    out, f = _filter('Let me look. <tool_call>{"name": "look", "arguments": {}}</tool_call> extra')
    assert out.strip() == "Let me look."
    assert f.saw_tool


def test_a_lone_angle_bracket_is_not_swallowed():
    out, _ = _filter("Two is < three.")
    assert out == "Two is < three."


# --- harness streaming hooks ----------------------------------------------------

class StreamingLLM:
    def __init__(self, scripted):
        self._scripted = list(scripted)

    def stream_completion(self, messages, *, is_current, max_tokens=None, on_delta=None):
        text = self._scripted.pop(0)
        if on_delta:
            for piece in _drip(text, 4):
                on_delta(piece)
        return text


def test_run_turn_streams_spoken_text_and_reports_tools():
    llm = StreamingLLM([
        'Let me look.<tool_call>{"name": "look", "arguments": {}}</tool_call>',
        "I see a red can.",
    ])
    spoken, tools_run, first = [], [], []
    out = harness.run_turn(
        "what do you see", [], system_prompt="SYS", llm=llm,
        call_tool=lambda n, a: {"ok": True}, is_current=lambda: True,
        on_text=spoken.append, on_tool=tools_run.append,
        on_first_delta=lambda: first.append(1))
    assert out == "I see a red can."
    assert "".join(spoken) == "Let me look.I see a red can."
    assert tools_run == ["look"]
    assert first == [1]


# --- timeline -------------------------------------------------------------------

def test_timeline_reports_eos_to_audio():
    tl = TurnTimeline(speech_end=100.0)
    tl.mark("asr_done", 100.2)
    tl.mark("turn_start", 100.25)
    tl.mark("first_audio", 101.0)
    tl.mark("first_audio", 105.0)  # later marks of a stage are ignored
    line = tl.format()
    assert "eos->audio 1.00s" in line
    assert "asr_done +200ms" in line
