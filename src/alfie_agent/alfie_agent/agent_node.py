"""
agent_node — the conversation bridge.

Closes the loop between ASR and TTS: it subscribes to transcripts (`asrresult`),
sends them to the local MLC-LLM server (Qwen3.6 35B-A3B, OpenAI-compatible API),
and publishes the reply as a `speechrequest` for TTS to speak. It also publishes
a `generating` flag (true while the LLM is producing a turn) that drives the
"thinking" LED state, and honours `barge_in` so a user can interrupt.

Design notes:
  * Wake-gated: transcripts are only processed while a listening window (opened
    by a `wake` event) is open, so background speech is ignored. The window
    re-opens after each reply for hands-free follow-ups; a bare wake phrase is
    stripped and just (re)opens the window.
  * Gated on `llm/ready` so nothing is sent before the 35B server has loaded.
  * The LLM call runs in a worker thread so ROS callbacks never block.
  * Turns are invalidated by a monotonically increasing turn id: a new
    transcript or a barge-in bumps the id, and any in-flight worker notices on
    its next streamed chunk and bails without publishing speech.
  * Qwen emits <think>...</think> reasoning by default; we request /no_think
    and strip any think blocks so the robot never speaks its reasoning.
"""
import re
import json
import threading
import time

import requests

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Empty, String
from alfie_msgs.msg import ASRResult, SpeechRequest, Speaking

LLM_BASE_URL = "http://localhost:8000/v1"
MODEL_ID_FALLBACK = "dist/qwen3_6-35B-A3B-q4f16_1"
SYSTEM_PROMPT = (
    "You are Alfie, a friendly desktop robot. Keep replies short and "
    "conversational, one or two sentences, suitable for being spoken aloud. "
    "Do not use markdown, emoji, or stage directions. /no_think"
)
MAX_HISTORY_TURNS = 6          # user+assistant pairs kept as context
REQUEST_TIMEOUT = (5, 60)      # (connect, read) seconds
MAX_TOKENS = 200
TEMPERATURE = 0.7
SPEAK_VOLUME = 100

# Wake-word gating: a `wake` event opens a listening window; transcripts are only
# processed while it is open. The window is (re)opened on a wake and extended on
# each accepted user command (which covers the reply plus a follow-up gap) — it
# is NOT extended by Alfie's own speech, so background audio can't keep it alive
# indefinitely. When it lapses, the wake word is required again (and the next
# wake starts a fresh conversation).
FOLLOWUP_WINDOW_S = 8.0
WAKE_PHRASES_DEFAULT = {'hey alfie', 'alfie'}
# A very short transcript arriving right after a wake is the (possibly mis-heard)
# wake phrase itself, not a command — acknowledge it instead of answering it.
WAKE_UTTERANCE_SUPPRESS_S = 3.0
MAX_WAKE_UTTERANCE_WORDS = 2

_THINK_RE = re.compile(r"<think>.*?</think>", re.DOTALL)


class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.speech_pub = self.create_publisher(SpeechRequest, 'speechrequest', qos)
        self.generating_pub = self.create_publisher(Bool, 'generating', qos)

        self.create_subscription(ASRResult, 'asrresult', self.on_asrresult, qos)
        self.create_subscription(Empty, 'barge_in', self.on_barge_in, qos)
        self.create_subscription(Speaking, 'speaking', self.on_speaking, qos)

        # Wake events open the conversation window. RELIABLE to match wakeword_node.
        wake_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, 'wake', self.on_wake, wake_qos)

        # The LLM server (35B MoE) takes ~30 s to load. Gate on its latched
        # `llm/ready` so transcripts heard during startup are dropped instead of
        # hitting a connection-refused. Latched QoS so we still get the retained
        # value if the agent starts after the server is already up.
        ready_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'llm/ready', self.on_llm_ready, ready_qos)
        self._llm_ready = False

        self._lock = threading.Lock()
        self._turn = 0
        self._history = []          # list of {"role", "content"}
        self._model_id = None       # resolved lazily from /v1/models

        # Wake-gating state.
        self._listen_until = 0.0    # monotonic deadline; window open while now < it
        self._wake_time = 0.0       # monotonic time of the last wake event
        self._speaking = False
        self._wake_phrases = set(WAKE_PHRASES_DEFAULT)

        self.publish_generating(False)
        self.get_logger().info(
            'AgentNode initialized (waiting for llm/ready; say the wake word to talk).')

    # --- ROS callbacks (kept short; real work happens on worker threads) ---

    def on_llm_ready(self, msg):
        was_ready = self._llm_ready
        self._llm_ready = bool(msg.data)
        if self._llm_ready and not was_ready:
            self.get_logger().info('LLM is ready; waiting for wake word.')
        elif not self._llm_ready and was_ready:
            self.get_logger().warn('LLM went not-ready; pausing.')

    def on_wake(self, msg):
        phrase = (msg.data or '').strip().lower()
        now = time.monotonic()
        fresh = now >= self._listen_until   # window was closed -> new conversation
        if phrase:
            self._wake_phrases.add(phrase)
        self._listen_until = now + FOLLOWUP_WINDOW_S
        self._wake_time = now
        if fresh:
            with self._lock:
                self._history = []
            self.get_logger().info(f"Wake '{phrase}': new conversation, listening.")
        else:
            self.get_logger().info(f"Wake '{phrase}': window extended.")

    def on_speaking(self, msg):
        # Track TTS state only. The window is deliberately NOT extended here — if
        # it were, Alfie answering background audio would keep it open forever.
        self._speaking = msg.is_speaking

    def on_asrresult(self, msg):
        text = (msg.asrresult or '').strip()
        if not text:
            return
        if not self._llm_ready:
            self.get_logger().warn('LLM not ready; ignoring transcript.',
                                   throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        if now >= self._listen_until:
            self.get_logger().info(f'No wake word; ignoring: {text}',
                                   throttle_duration_sec=5.0)
            return
        command, _ = self._strip_wake(text)
        if not command:
            # Bare wake phrase ("Hey Alfie") — acknowledge and keep listening.
            self._listen_until = now + FOLLOWUP_WINDOW_S
            self.get_logger().info('Wake acknowledged; listening for a command.')
            return
        if (now - self._wake_time) < WAKE_UTTERANCE_SUPPRESS_S and \
                len(command.split()) <= MAX_WAKE_UTTERANCE_WORDS:
            # Very short transcript right after a wake = the (mis-heard) wake
            # phrase itself. Keep listening, don't answer it.
            self._listen_until = now + FOLLOWUP_WINDOW_S
            self.get_logger().info(f'Ignoring wake utterance: {command}')
            return
        # Keep the window alive across LLM + TTS for this turn.
        self._listen_until = now + FOLLOWUP_WINDOW_S
        self.get_logger().info(f'Heard: {command}')
        with self._lock:
            self._turn += 1
            my_turn = self._turn
        threading.Thread(target=self._run_turn, args=(command, my_turn),
                         daemon=True).start()

    def on_barge_in(self, msg):
        # Invalidate any in-flight turn and drop the thinking state immediately.
        with self._lock:
            self._turn += 1
        self.publish_generating(False)

    def _strip_wake(self, text):
        """Remove a leading wake phrase; return (command, had_wake). Compares
        word-by-word, case- and punctuation-insensitive, longest phrase first."""
        words = text.split()
        low = [re.sub(r'[^a-z0-9]', '', w.lower()) for w in words]
        for ph in sorted(self._wake_phrases, key=lambda p: len(p.split()), reverse=True):
            pw = ph.split()
            if low[:len(pw)] == pw:
                return ' '.join(words[len(pw):]).strip(), True
        return text.strip(), False

    # --- helpers ---

    def publish_generating(self, value):
        m = Bool()
        m.data = bool(value)
        self.generating_pub.publish(m)

    def _is_current(self, my_turn):
        with self._lock:
            return my_turn == self._turn

    def _resolve_model_id(self):
        if self._model_id:
            return self._model_id
        try:
            r = requests.get(f"{LLM_BASE_URL}/models", timeout=REQUEST_TIMEOUT)
            r.raise_for_status()
            self._model_id = r.json()["data"][0]["id"]
        except Exception:
            self._model_id = MODEL_ID_FALLBACK
        return self._model_id

    def _run_turn(self, text, my_turn):
        self.publish_generating(True)
        reply = None
        try:
            reply = self._stream_llm(text, my_turn)
        except Exception as e:
            self.get_logger().error(f'LLM request failed: {e}')

        if not self._is_current(my_turn):
            return  # superseded by a newer turn or a barge-in

        self.publish_generating(False)
        if reply:
            with self._lock:
                self._history.append({"role": "user", "content": text})
                self._history.append({"role": "assistant", "content": reply})
                # keep only the most recent MAX_HISTORY_TURNS pairs
                self._history = self._history[-2 * MAX_HISTORY_TURNS:]
            self.get_logger().info(f'Reply: {reply}')
            out = SpeechRequest()
            out.text = reply
            out.volume = SPEAK_VOLUME
            self.speech_pub.publish(out)

    def _stream_llm(self, text, my_turn):
        with self._lock:
            messages = ([{"role": "system", "content": SYSTEM_PROMPT}]
                        + list(self._history)
                        + [{"role": "user", "content": text}])
        payload = {
            "model": self._resolve_model_id(),
            "messages": messages,
            "stream": True,
            "temperature": TEMPERATURE,
            "max_tokens": MAX_TOKENS,
        }
        parts = []
        with requests.post(f"{LLM_BASE_URL}/chat/completions", json=payload,
                           stream=True, timeout=REQUEST_TIMEOUT) as r:
            r.raise_for_status()
            for line in r.iter_lines(decode_unicode=True):
                if not self._is_current(my_turn):
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
        return self._clean("".join(parts))

    @staticmethod
    def _clean(reply):
        reply = _THINK_RE.sub("", reply)
        # drop any unterminated leading think block
        if "</think>" in reply:
            reply = reply.split("</think>")[-1]
        return reply.strip()


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
