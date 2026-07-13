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
"""
import json
import re

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


def _tool_response_turn(pairs):
    """Build a single user turn wrapping all (name, result) tool responses."""
    blocks = []
    for name, result in pairs:
        blocks.append("<tool_response>\n"
                      + json.dumps({"tool": name, "result": result})
                      + "\n</tool_response>")
    return {"role": "user", "content": "\n".join(blocks)}


def run_turn(user_text, history, *, system_prompt, llm, call_tool, is_current,
             logger=None, max_tool_iters=4, max_tokens=None):
    """
    Run one conversational turn, resolving any tool calls.

    Returns the spoken reply text, or ``None`` if the turn was cancelled.
    ``history`` is a list of prior {role, content} turns and is not mutated.
    ``call_tool(name, arguments) -> dict`` executes a tool.
    """
    messages = ([{"role": "system", "content": system_prompt}]
                + list(history)
                + [{"role": "user", "content": user_text}])

    for _ in range(max_tool_iters):
        if not is_current():
            return None
        raw = llm.stream_completion(messages, is_current=is_current,
                                    max_tokens=max_tokens)
        if raw is None:
            return None  # cancelled mid-stream

        calls = parse_tool_calls(raw)
        if not calls:
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
            try:
                result = call_tool(name, call["arguments"])
            except Exception as e:  # tools shouldn't raise, but never crash a turn
                result = {"error": f"tool {name} raised: {e}"}
            pairs.append((name, result))
        messages.append(_tool_response_turn(pairs))

    # Budget exhausted — force a final, tool-free spoken answer.
    if not is_current():
        return None
    messages.append({"role": "user",
                     "content": "Answer now in plain spoken language without "
                                "calling any more tools."})
    raw = llm.stream_completion(messages, is_current=is_current,
                                max_tokens=max_tokens)
    if raw is None:
        return None
    return strip_markup(raw)
