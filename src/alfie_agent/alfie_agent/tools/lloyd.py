"""
lloyd — ``ask_lloyd``: hand a request to lloyd, the desktop agent, and relay its
answer.

Rather than port lloyd's email / calendar / Discord / backlog / knowledge-graph
tools, Alfie forwards the request to lloyd's own agent loop over its HTTP API and
speaks the result:

    POST <lloyd_url>/api/message/stream  {"text": ..., "session_id": ...}
    -> text/event-stream; the final ``event: done`` carries
       ``{"response": <final text>, "session_id": ...}``.

lloyd only accepts /api from its trusted networks (Tailscale's 100.64.0.0/10 by
default), so the Orin reaches it over the tailnet; ``lloyd_url`` is e.g.
``http://lloyd-host:8080``. The request is prefixed with a note asking for a
short spoken answer, and lloyd's session id is kept for the live episode so
follow-ups ("and what about Friday?") land in the same lloyd conversation;
``reset_session()`` starts a fresh one (agent_node calls it per episode).

lloyd's answer can carry web content and messages from other people, so this tool
is in ``TAINTING_TOOLS``: motion tools are refused for the rest of the turn.
"""
import json
import re
import threading

import requests

TAINTING_TOOLS = frozenset({"ask_lloyd"})

_PREFIX = ("[Voice request relayed by Alfie, the robot. Reply in one to three "
           "short, plain spoken sentences: no markdown, lists or links.] ")
_MAX_ANSWER = 700

_url = ""
_timeout = 90.0
_session = {"id": ""}
_lock = threading.Lock()


def configure(url="", timeout=90.0):
    global _url, _timeout
    _url = (url or "").rstrip("/")
    _timeout = float(timeout)


def reset_session():
    with _lock:
        _session["id"] = ""


def _err(msg):
    return {"error": msg}


def parse_sse(lines):
    """Yield (event, data-dict) pairs from an iterable of SSE text lines."""
    event, data = None, []
    for line in lines:
        if line is None:
            continue
        if line == "":
            if event or data:
                try:
                    payload = json.loads("\n".join(data)) if data else {}
                except json.JSONDecodeError:
                    payload = {"raw": "\n".join(data)}
                yield event or "message", payload
            event, data = None, []
        elif line.startswith("event:"):
            event = line[6:].strip()
        elif line.startswith("data:"):
            data.append(line[5:].lstrip())
    if event or data:
        try:
            yield event or "message", json.loads("\n".join(data)) if data else {}
        except json.JSONDecodeError:
            yield event or "message", {"raw": "\n".join(data)}


def speakable(text):
    """Strip markdown from lloyd's answer and cap its length."""
    text = re.sub(r"```.*?```", " ", text or "", flags=re.S)
    text = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", text)       # links -> label
    text = re.sub(r"[*_`#>|]+", "", text)
    text = re.sub(r"^\s*[-•]\s+", "", text, flags=re.M)
    text = " ".join(text.split())
    if len(text) > _MAX_ANSWER:
        cut = text[:_MAX_ANSWER]
        text = cut[:cut.rfind(". ") + 1] or cut
    return text


def _ask_lloyd(params):
    request = " ".join(str(params.get("request") or "").split())
    if not request:
        return _err("request is required")
    if not _url:
        return _err("I'm not connected to Lloyd (lloyd_url isn't set).")
    with _lock:
        session_id = _session["id"]
    try:
        with requests.post(f"{_url}/api/message/stream",
                           json={"text": _PREFIX + request, "session_id": session_id},
                           stream=True, timeout=(5, _timeout)) as r:
            if r.status_code == 403:
                return _err("Lloyd refused the connection (this robot isn't on "
                            "his trusted network).")
            r.raise_for_status()
            for event, data in parse_sse(r.iter_lines(decode_unicode=True)):
                if event == "error":
                    return _err(f"Lloyd hit an error: {data.get('detail', 'unknown')}")
                if event == "done":
                    with _lock:
                        _session["id"] = data.get("session_id") or session_id
                    if data.get("cancelled"):
                        return _err("Lloyd's turn was cancelled.")
                    answer = speakable(data.get("response") or "")
                    return {"answer": answer or "Lloyd didn't say anything."}
    except requests.exceptions.ConnectionError:
        return _err("I can't reach Lloyd right now.")
    except requests.exceptions.Timeout:
        return _err("Lloyd is taking too long to answer.")
    except requests.exceptions.RequestException as e:
        return _err(f"asking Lloyd failed ({type(e).__name__}).")
    return _err("Lloyd's answer was cut off.")


_TOOLS = [
    {
        "name": "ask_lloyd",
        "description": "Ask Lloyd, Alan's desktop AI agent, to do or answer what "
                       "you can't: email, calendar, messages, his projects and "
                       "notes, deep research. Slow (seconds to a minute).",
        "parameters": {
            "type": "object",
            "properties": {"request": {"type": "string"}},
            "required": ["request"],
        },
    },
]


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    if name != "ask_lloyd":
        return _err(f"unknown tool: {name}")
    return _ask_lloyd(arguments or {})
