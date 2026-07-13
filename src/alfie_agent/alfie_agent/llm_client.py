"""
llm_client — streaming client for the local MLC-LLM OpenAI-compatible server.

Extracted from the original agent_node's ``_stream_llm``. Talks to the MLC-LLM
server (Qwen3.6 35B-A3B) at ``<base_url>/chat/completions`` with ``stream=True``
and returns the raw accumulated text. Note: no ``tools`` field is sent — MLC's
chatml serving doesn't do native OpenAI tool-calling, so tool calls are prompted
into the text and parsed downstream (see harness.py). Cleaning of <think>/
<tool_call> markup is left to the caller, which needs the raw text.

The stream is cancellable: a caller passes an ``is_current`` predicate that is
checked on every chunk, so a barge-in or a newer turn aborts the request without
publishing anything (returns ``None``).
"""
import json

import requests


class LLMClient:
    def __init__(self, base_url, model_id_fallback, *,
                 timeout=(5, 60), temperature=0.7, max_tokens=200):
        self.base_url = base_url.rstrip("/")
        self.model_id_fallback = model_id_fallback
        self.timeout = timeout
        self.temperature = temperature
        self.max_tokens = max_tokens
        self._model_id = None

    def resolve_model_id(self):
        """
        Resolve and cache the served model id, falling back to the default.

        Queries ``/v1/models``; on failure returns the configured fallback id.
        """
        if self._model_id:
            return self._model_id
        try:
            r = requests.get(f"{self.base_url}/models", timeout=self.timeout)
            r.raise_for_status()
            self._model_id = r.json()["data"][0]["id"]
        except Exception:
            self._model_id = self.model_id_fallback
        return self._model_id

    def stream_completion(self, messages, *, is_current, max_tokens=None):
        """
        Stream a chat completion and return the raw accumulated text.

        Returns ``None`` if ``is_current()`` goes false mid-stream (cancelled).
        Raises on transport/HTTP errors (the caller logs and recovers).
        """
        payload = {
            "model": self.resolve_model_id(),
            "messages": messages,
            "stream": True,
            "temperature": self.temperature,
            "max_tokens": max_tokens or self.max_tokens,
        }
        parts = []
        with requests.post(f"{self.base_url}/chat/completions", json=payload,
                           stream=True, timeout=self.timeout) as r:
            r.raise_for_status()
            for line in r.iter_lines(decode_unicode=True):
                if not is_current():
                    return None  # cancelled (barge-in or newer turn)
                if not line or not line.startswith("data:"):
                    continue
                data = line[5:].strip()
                if data == "[DONE]":
                    break
                try:
                    delta = json.loads(data)["choices"][0]["delta"]
                except (json.JSONDecodeError, KeyError, IndexError):
                    continue
                piece = delta.get("content")
                if piece:
                    parts.append(piece)
        return "".join(parts)
