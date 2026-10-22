"""
harness — the tool-calling agent loop.

Because the MLC-LLM server can't do native OpenAI tool-calling, the tool protocol
lives in the prompt: Qwen3 is asked to emit ``<tool_call>{...}</tool_call>``
blocks, which we parse out of the streamed text (mirroring lloyd's tolerant
``_commit_tool_calls`` quirk handling). The loop:

  1. stream a completion,
  2. scan for tool calls; if none, the cleaned text is the spoken answer,
  3. otherwise execute each tool and feed the results back (as a ``user`` turn
     wrapping ``<tool_response>`` — chatml has no ``tool`` role), then loop,
  4. stop at ``max_tool_iters`` with a final tool-free answer pass.

Cancellation: an ``is_current`` predicate is checked before every LLM call and
every tool call, so a barge-in or newer turn aborts and returns ``None`` without
speaking.

Streaming: with ``on_text`` the spoken part of every generation is handed out
as it streams (``SpokenFilter`` keeps ``<think>`` and ``<tool_call>`` blocks
out of it), so the caller can speak while the model writes — including text the
model says *before* a tool call ("Let me look."), which is heard before the
tool runs. ``on_tool(name)`` fires just before each tool executes.
"""
import json
import re
import time

_TOOL_CALL_RE = re.compile(r"<tool_call>\s*(.*?)\s*</tool_call>", re.DOTALL)
_THINK_RE = re.compile(r"<think>.*?</think>", re.DOTALL)


def _extract_json_object(s):
    """Return the first balanced ``{...}`` substring of ``s``, or None."""
    start = s.find("{")
    if start < 0:
        return None
    depth = 0
    for i in range(start, len(s)):
        c = s[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return s[start:i + 1]
    return None


def _loads_tolerant(blob):
    """Parse a tool-call JSON blob, tolerating trailing junk / stray braces."""
    blob = blob.strip()
    try:
        return json.loads(blob)
    except json.JSONDecodeError:
        obj = _extract_json_object(blob)
        if obj is not None:
            try:
                return json.loads(obj)
            except json.JSONDecodeError:
                return None
    return None


def parse_tool_calls(text):
    """
    Extract tool calls from model text.

    Returns a list of dicts with ``name`` + ``arguments`` (a dict), or an
    ``error`` key for a block that couldn't be parsed. An unterminated
    ``<tool_call>`` (e.g. output cut off) is handled by taking the tail.
    """
    blobs = _TOOL_CALL_RE.findall(text)
    if not blobs and "<tool_call>" in text:
        blobs = [text.split("<tool_call>", 1)[1]]
    calls = []
    for blob in blobs:
        parsed = _loads_tolerant(blob)
        if not isinstance(parsed, dict) or "name" not in parsed:
            calls.append({"name": None, "arguments": {},
                          "error": f"could not parse tool call: {blob.strip()[:200]}"})
            continue
        args = parsed.get("arguments", {})
        if not isinstance(args, dict):
            args = {}
        calls.append({"name": parsed["name"], "arguments": args})
    return calls


def strip_markup(text):
    """Strip <think> reasoning and <tool_call> blocks, leaving spoken text."""
    text = _THINK_RE.sub("", text)
    if "</think>" in text:  # unterminated leading think block
        text = text.split("</think>")[-1]
    text = _TOOL_CALL_RE.sub("", text)
    if "<tool_call>" in text:  # dangling unterminated tool call
        text = text.split("<tool_call>")[0]
    return text.strip()


class SpokenFilter:
    """Pass streamed model text through, minus ``<think>``/``<tool_call>`` blocks.

    Tags can be split across deltas, so a trailing fragment that could still
    become a tag is held until the next delta decides it. Once a tool call
    opens, nothing else in that generation is spoken: the protocol asks the
    model to stop after it, and anything it adds is not meant for the listener.
    Leading whitespace is never emitted (Qwen's empty ``<think></think>`` under
    /no_think leaves a run of newlines).
    """

    _OPEN = ("<think>", "<tool_call>")

    def __init__(self):
        self._buf = ""
        self._state = "out"     # "out" | "think" | "tool"
        self._started = False   # emitted any non-whitespace yet
        self.saw_tool = False

    def feed(self, delta):
        self._buf += delta or ""
        out = []
        while self._buf:
            if self._state == "tool":
                self._buf = ""
                break
            if self._state == "think":
                end = self._buf.find("</think>")
                if end < 0:
                    # Keep only a tail that could be the start of "</think>".
                    self._buf = self._buf[-(len("</think>") - 1):]
                    break
                self._buf = self._buf[end + len("</think>"):]
                self._state = "out"
                continue
            lt = self._buf.find("<")
            if lt < 0:
                out.append(self._buf)
                self._buf = ""
                break
            out.append(self._buf[:lt])
            rest = self._buf[lt:]
            if rest.startswith("<think>"):
                self._state, self._buf = "think", rest[len("<think>"):]
            elif rest.startswith("</think>"):
                self._buf = rest[len("</think>"):]  # stray close: drop the tag
            elif rest.startswith("<tool_call>"):
                self._state, self._buf, self.saw_tool = "tool", "", True
            elif any(t.startswith(rest) for t in self._OPEN + ("</think>",)):
                self._buf = rest     # could still become a tag: wait
                break
            else:
                out.append("<")
                self._buf = rest[1:]
        return self._emit("".join(out))

    def flush(self):
        """End of the generation: release a held fragment that never became a tag."""
        rest = self._buf if self._state == "out" else ""
        self._buf = ""
        return self._emit(rest)

    def _emit(self, text):
        if not self._started:
            text = text.lstrip()
            if text:
                self._started = True
        return text


def _tool_response_turn(pairs):
    """Build a single user turn wrapping all (name, result) tool responses."""
    blocks = []
    for name, result in pairs:
        blocks.append("<tool_response>\n"
                      + json.dumps({"tool": name, "result": result})
                      + "\n</tool_response>")
    return {"role": "user", "content": "\n".join(blocks)}


def approx_tokens(text):
    """Cheap token estimate (~4 chars/token) — no tokenizer needed on-device."""
    return max(1, len(text or "") // 4)


def budget_trim(turns, max_tokens):
    """
    Drop the oldest turn-pairs until ``turns`` fits ``max_tokens`` (approx).

    Turns are appended as (user, assistant) pairs, so we drop from the front two
    at a time to keep roles paired and always retain at least the last pair.
    Returns a new list; the input is not mutated.
    """
    turns = list(turns)
    total = sum(approx_tokens(t.get("content", "")) for t in turns)
    while total > max_tokens and len(turns) > 2:
        dropped = turns[:2]
        turns = turns[2:]
        total -= sum(approx_tokens(t.get("content", "")) for t in dropped)
    return turns


# --- episode summarisation (idle compaction; see agent_node) ------------------

_SUMMARIZER_SYS = (
    "You compress a short spoken conversation between a user and Alfie (a desktop "
    "robot) into durable memory. Reply with ONLY a JSON object and nothing else: "
    '{"summary": a one to three sentence third-person summary of what happened '
    'and what the user wanted, "facts": a JSON array of durable facts worth '
    "remembering about the user or their world (names, preferences, ongoing "
    'tasks), each {"entity": who or what the fact is about (a person\'s name if '
    "known, else User), \"category\": one of preference, relationship, goal, "
    'event, state, skill, general, "fact": one short self-contained sentence} '
    "— use [] if there are none. Small talk is not a fact}. Do not include any "
    "other text. /no_think"
)
_SUMMARIZER_INSTR = "Summarize the conversation above as the JSON object described."


def _parse_fact(item):
    """One summarizer fact -> ``{"entity", "category", "fact"}`` or None.

    Tolerates the older plain-string shape (filed under ``User``/``general``).
    """
    if isinstance(item, dict):
        fact = str(item.get("fact") or "").strip()
        if not fact:
            return None
        return {"entity": str(item.get("entity") or "User").strip() or "User",
                "category": str(item.get("category") or "general").strip(),
                "fact": fact}
    fact = str(item or "").strip()
    return {"entity": "User", "category": "general", "fact": fact} if fact else None


def parse_summary(raw):
    """Parse the summarizer's reply into ``{"summary", "facts"}`` (tolerant).

    ``facts`` is a list of ``{"entity", "category", "fact"}`` dicts.
    """
    text = strip_markup(raw or "")
    obj = _loads_tolerant(text)
    if isinstance(obj, dict) and "summary" in obj:
        facts = obj.get("facts", [])
        if not isinstance(facts, list):
            facts = []
        return {"summary": str(obj.get("summary") or "").strip(),
                "facts": [f for f in map(_parse_fact, facts) if f]}
    # Fallback: no parseable JSON — keep the plain text as the summary.
    return {"summary": text.strip()[:400], "facts": []}


def _speakers_note(speakers):
    speakers = sorted(set(speakers))
    if len(speakers) == 1:
        return f"The user in this conversation is {speakers[0]}. "
    if speakers:
        return (f"Voices recognized in this conversation: {', '.join(speakers)} "
                "(lines are not attributed). ")
    return ""


def summarize(turns, llm, *, max_tokens=256, speakers=()):
    """
    Summarize an episode's turns into ``{"summary", "facts"}`` via the LLM.

    Runs off the conversation critical path (idle compaction). Returns an empty
    summary if the call is cancelled or yields nothing. ``speakers`` are the
    voices recognized in the episode (alfie_asr speaker ID), so facts are filed
    under the person rather than ``User``.
    """
    convo = "\n".join(
        f"{'User' if t.get('role') == 'user' else 'Alfie'}: {t.get('content', '')}"
        for t in turns
    )
    messages = [
        {"role": "system", "content": _SUMMARIZER_SYS},
        {"role": "user", "content": convo + "\n\n" + _speakers_note(speakers)
         + _SUMMARIZER_INSTR},
    ]
    raw = llm.stream_completion(messages, is_current=lambda: True,
                                max_tokens=max_tokens)
    if raw is None:
        return {"summary": "", "facts": []}
    return parse_summary(raw)


def run_turn(user_text, history, *, system_prompt, llm, call_tool, is_current,
             logger=None, max_tool_iters=4, max_tokens=None, on_text=None,
             on_tool=None, on_first_delta=None, taint_tools=frozenset(),
             motion_tools=frozenset()):
    """
    Run one conversational turn, resolving any tool calls.

    Returns the spoken reply text, or ``None`` if the turn was cancelled.
    ``history`` is a list of prior {role, content} turns and is not mutated.
    ``call_tool(name, arguments) -> dict`` executes a tool.

    The recent-memory window (if any) is folded into ``system_prompt`` by the
    prompt builder — MLC-LLM only accepts a system prompt at position 0, so it
    cannot ride in as a second ``system`` message.

    ``on_text(text)`` receives the spoken part of each generation as it streams
    (see ``SpokenFilter``); ``on_tool(name)`` is called before each tool runs;
    ``on_first_delta()`` once, at the first token of the turn.

    Untrusted content guard: once a tool in ``taint_tools`` (web pages, lloyd's
    relayed answers) has returned in this turn, tools in ``motion_tools`` are
    refused for the rest of it — a page saying "start the task" must not move
    the robot. The user can simply ask again in a new turn.
    """
    messages = ([{"role": "system", "content": system_prompt}]
                + list(history)
                + [{"role": "user", "content": user_text}])

    first = {"seen": False}

    def generate():
        """One streamed completion, its spoken text forwarded to ``on_text``."""
        if on_text is None and on_first_delta is None:
            return llm.stream_completion(messages, is_current=is_current,
                                         max_tokens=max_tokens)
        filt = SpokenFilter()

        def on_delta(piece):
            if not first["seen"]:
                first["seen"] = True
                if on_first_delta is not None:
                    on_first_delta()
            if on_text is not None:
                text = filt.feed(piece)
                if text:
                    on_text(text)

        raw = llm.stream_completion(messages, is_current=is_current,
                                    max_tokens=max_tokens, on_delta=on_delta)
        if raw is not None and on_text is not None:
            text = filt.flush()
            if text:
                on_text(text)
        return raw

    turn_t0 = time.monotonic()
    tainted = False
    for gen in range(max_tool_iters):
        if not is_current():
            return None
        t0 = time.monotonic()
        prompt_tok = sum(approx_tokens(m.get("content", "")) for m in messages)
        raw = generate()
        if raw is None:
            return None  # cancelled mid-stream
        if logger:
            dt = time.monotonic() - t0
            logger(f"gen {gen}: {dt:.2f}s (prompt ~{prompt_tok} tok, "
                   f"out ~{approx_tokens(raw)} tok)")

        calls = parse_tool_calls(raw)
        if not calls:
            if logger:
                logger(f"turn done in {time.monotonic() - turn_t0:.2f}s "
                       f"({gen + 1} generation(s), no tool)" if gen == 0 else
                       f"turn done in {time.monotonic() - turn_t0:.2f}s "
                       f"({gen + 1} generation(s))")
            return strip_markup(raw)

        # Record the model's tool-call turn, then run the tools and feed the
        # results back as a single user turn (chatml has no `tool` role).
        messages.append({"role": "assistant", "content": raw})
        pairs = []
        for call in calls:
            if not is_current():
                return None
            name = call.get("name")
            if call.get("error") or not name:
                pairs.append((name or "unknown",
                              {"error": call.get("error", "invalid tool call")}))
                continue
            if logger:
                logger(f"tool call: {name}({call['arguments']})")
            if tainted and name in motion_tools:
                if logger:
                    logger(f"refused {name}: web content entered this turn")
                pairs.append((name, {"error": (
                    "refused: movement is disabled for the rest of this turn "
                    "because it read web or relayed content; ask the user to "
                    "repeat the request")}))
                continue
            if on_tool is not None:
                on_tool(name)
            try:
                result = call_tool(name, call["arguments"])
            except Exception as e:  # tools shouldn't raise, but never crash a turn
                result = {"error": f"tool {name} raised: {e}"}
            if name in taint_tools:
                tainted = True
            pairs.append((name, result))
        messages.append(_tool_response_turn(pairs))

    # Budget exhausted — force a final, tool-free spoken answer.
    if not is_current():
        return None
    messages.append({"role": "user",
                     "content": "Answer now in plain spoken language without "
                                "calling any more tools."})
    raw = generate()
    if raw is None:
        return None
    if logger:
        logger(f"turn done in {time.monotonic() - turn_t0:.2f}s "
               f"(hit max_tool_iters={max_tool_iters})")
    return strip_markup(raw)
