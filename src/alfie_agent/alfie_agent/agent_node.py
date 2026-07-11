"""
agent_node — the conversation bridge.

Closes the loop between ASR and TTS: it subscribes to transcripts (`asrresult`),
sends them to the local MLC-LLM server (Qwen3-1.7B, OpenAI-compatible API), and
publishes the reply as a `speechrequest` for TTS to speak. It also publishes a
`generating` flag (true while the LLM is producing a turn) that drives the
"thinking" LED state, and honours `barge_in` so a user can interrupt.

Design notes:
  * The LLM call runs in a worker thread so ROS callbacks never block.
  * Turns are invalidated by a monotonically increasing turn id: a new
    transcript or a barge-in bumps the id, and any in-flight worker notices on
    its next streamed chunk and bails without publishing speech.
  * Qwen3 emits <think>...</think> reasoning by default; we request /no_think
    and strip any think blocks so the robot never speaks its reasoning.
"""
import re
import json
import threading

import requests

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Empty
from alfie_msgs.msg import ASRResult, SpeechRequest

LLM_BASE_URL = "http://localhost:8000/v1"
MODEL_ID_FALLBACK = "/data/qwen3-1.7b-q4f16_1-MLC"
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

_THINK_RE = re.compile(r"<think>.*?</think>", re.DOTALL)


class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.speech_pub = self.create_publisher(SpeechRequest, 'speechrequest', qos)
        self.generating_pub = self.create_publisher(Bool, 'generating', qos)

        self.create_subscription(ASRResult, 'asrresult', self.on_asrresult, qos)
        self.create_subscription(Empty, 'barge_in', self.on_barge_in, qos)

        self._lock = threading.Lock()
        self._turn = 0
        self._history = []          # list of {"role", "content"}
        self._model_id = None       # resolved lazily from /v1/models

        self.publish_generating(False)
        self.get_logger().info('AgentNode initialized.')

    # --- ROS callbacks (kept short; real work happens on worker threads) ---

    def on_asrresult(self, msg):
        text = (msg.asrresult or '').strip()
        if not text:
            return
        self.get_logger().info(f'Heard: {text}')
        with self._lock:
            self._turn += 1
            my_turn = self._turn
        threading.Thread(target=self._run_turn, args=(text, my_turn),
                         daemon=True).start()

    def on_barge_in(self, msg):
        # Invalidate any in-flight turn and drop the thinking state immediately.
        with self._lock:
            self._turn += 1
        self.publish_generating(False)

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
