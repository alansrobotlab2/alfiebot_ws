"""
wakeword_node — openWakeWord detection on the reSpeaker's AEC-cleaned audio.

Runs one or more openWakeWord models over `audio_frames` and:
  * publishes `wake` (std_msgs/String = spoken phrase) when a model fires — this
    opens/extends the agent's conversation window;
  * publishes `barge_in` (std_msgs/Empty) if a wake fires while TTS is speaking,
    so you can interrupt Alfie by saying its name.

Fully offline: the oww base models (melspectrogram + embedding) and the wake
models are bundled in this package's `models/` dir and loaded by path — nothing
is downloaded at runtime.

Audio contract: `audio_frames` is int16[512] @ 16 kHz (~32 ms). openWakeWord
wants 80 ms (1280-sample) hops, so frames are accumulated and fed in 1280-sample
chunks.
"""
import os
import re
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String, Empty
from alfie_msgs.msg import AudioFrame, Speaking

from openwakeword.model import Model

OWW_CHUNK = 1280   # samples per openWakeWord hop (80 ms @ 16 kHz)
DEBUG_FLOOR = 0.1  # log near-miss scores >= this when debug_scores is on (tuning)

# Base (shared) oww models bundled alongside the wake models.
MELSPEC_MODEL = 'melspectrogram.onnx'
EMBEDDING_MODEL = 'embedding_model.onnx'

# Default models (custom-trained). "stop" is a cancel word: it barges in / aborts
# the reply but does NOT open a conversation window.
DEFAULT_WAKE_MODELS = ['hey_alfie.onnx', 'alfie.onnx', 'stop.onnx']
DEFAULT_CANCEL_KEYS = ['stop']
# Per-model detection thresholds (live-tunable via thr_<key> params).
DEFAULT_THRESHOLDS = {'hey_alfie': 0.45, 'alfie': 0.45, 'stop': 0.25}

# Energy-gate barge-in: wake-word matching fails during double-talk, but the AEC
# keeps the mic quiet (~250 RMS residual) while Alfie speaks, so a sustained loud
# mic signal means the user is talking over him -> barge in. No wake match needed.
# Tuned for a room with background TV (residual ~250, background peaks ~2400,
# a deliberate speak-up over TTS peaks ~4000+). Lower it (~1200) in a quiet room.
BARGE_THRESHOLD_DEFAULT = 2800.0   # mic RMS floor to count as the user talking
BARGE_HOLD_DEFAULT = 2             # consecutive 80 ms hops above it (~160 ms)


def _key_to_phrase(key):
    """'hey_jarvis_v0.1' -> 'hey jarvis' ; 'hey_alfie' -> 'hey alfie'."""
    key = re.sub(r'_v\d+(\.\d+)*$', '', key)   # drop version suffix
    return key.replace('_', ' ').strip()


class WakeWordNode(Node):
    def __init__(self):
        super().__init__('wakeword_node')

        model_dir = self.declare_parameter(
            'model_dir',
            os.path.join(get_package_share_directory('alfie_wakeword'), 'models')
        ).value
        wake_models = self.declare_parameter('wake_models', DEFAULT_WAKE_MODELS).value
        self._default_threshold = float(
            self.declare_parameter('default_threshold', 0.5).value)
        # Cancel words (keys) barge in but don't open a conversation window.
        self._cancel_keys = set(
            self.declare_parameter('cancel_keys', DEFAULT_CANCEL_KEYS).value)
        self._debounce = float(self.declare_parameter('debounce_sec', 1.5).value)
        # Log near-miss scores during threshold tuning.
        self._debug_scores = bool(self.declare_parameter('debug_scores', True).value)
        # Energy-gate barge-in (live-tunable).
        self._barge_enabled = bool(self.declare_parameter('barge_enabled', True).value)
        self._barge_threshold = float(
            self.declare_parameter('barge_energy_threshold', BARGE_THRESHOLD_DEFAULT).value)
        self._barge_hold = int(self.declare_parameter('barge_hold_hops', BARGE_HOLD_DEFAULT).value)

        wake_paths = [os.path.join(model_dir, m) for m in wake_models]
        for p in wake_paths:
            if not os.path.exists(p):
                self.get_logger().error(f'Wake model not found: {p}')

        self.get_logger().info('Loading openWakeWord models (offline)...')
        self.oww = Model(
            wakeword_models=wake_paths,
            inference_framework='onnx',
            melspec_model_path=os.path.join(model_dir, MELSPEC_MODEL),
            embedding_model_path=os.path.join(model_dir, EMBEDDING_MODEL),
        )
        model_keys = list(self.oww.models.keys())
        # Per-key thresholds as live-tunable params: `thr_<key>` (e.g. thr_alfie).
        # Tune at runtime: ros2 param set /alfie/wakeword_node thr_alfie 0.45
        self._thresholds = {}
        for key in model_keys:
            default = DEFAULT_THRESHOLDS.get(key, self._default_threshold)
            self._thresholds[key] = float(
                self.declare_parameter(f'thr_{key}', default).value)
        self.add_on_set_parameters_callback(self._on_set_params)
        self.get_logger().info(
            f'Models: {model_keys} | thresholds {self._thresholds} | '
            f'cancel words {sorted(self._cancel_keys)}')

        self._buf = np.zeros(0, dtype=np.int16)
        self._last_fire = {}
        self.speaking = False
        self._barge_run = 0        # consecutive loud hops while speaking
        self._last_barge = 0.0

        audio_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # wake is RELIABLE (must not be dropped); barge_in is BEST_EFFORT to match
        # the TTS subscriber.
        wake_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.wake_pub = self.create_publisher(String, 'wake', wake_qos)
        # Every detection (wake or cancel) is echoed here for LED feedback.
        self.detect_pub = self.create_publisher(String, 'wakeword/detection', wake_qos)
        self.barge_pub = self.create_publisher(Empty, 'barge_in', audio_qos)

        self.create_subscription(AudioFrame, 'audio_frames', self.on_audio, audio_qos)
        self.create_subscription(Speaking, 'speaking', self.on_speaking, audio_qos)

        self.get_logger().info('WakeWordNode initialized.')

    def on_speaking(self, msg):
        self.speaking = msg.is_speaking
        if not msg.is_speaking:
            self._barge_run = 0

    def _on_set_params(self, params):
        # Live tuning: `thr_<key>` thresholds + the barge-in energy gate.
        for p in params:
            if p.name.startswith('thr_'):
                key = p.name[len('thr_'):]
                if key in self._thresholds:
                    self._thresholds[key] = float(p.value)
                    self.get_logger().info(f'threshold {key} -> {float(p.value):.2f}')
            elif p.name == 'barge_energy_threshold':
                self._barge_threshold = float(p.value)
                self.get_logger().info(f'barge threshold -> {self._barge_threshold:.0f}')
            elif p.name == 'barge_hold_hops':
                self._barge_hold = int(p.value)
                self.get_logger().info(f'barge hold -> {self._barge_hold}')
            elif p.name == 'barge_enabled':
                self._barge_enabled = bool(p.value)
        return SetParametersResult(successful=True)

    def on_audio(self, msg):
        self._buf = np.concatenate(
            [self._buf, np.asarray(msg.audioframe, dtype=np.int16)])
        while len(self._buf) >= OWW_CHUNK:
            chunk = self._buf[:OWW_CHUNK]
            self._buf = self._buf[OWW_CHUNK:]
            scores = self.oww.predict(chunk)
            self._handle_scores(scores)
            self._handle_barge_energy(chunk)

    def _handle_barge_energy(self, chunk):
        # While Alfie speaks, a sustained loud mic (well above the AEC residual)
        # is the user talking over him -> barge in. No wake-word match needed.
        if not (self._barge_enabled and self.speaking):
            self._barge_run = 0
            return
        rms = float(np.sqrt(np.mean(chunk.astype(np.float32) ** 2)))
        if self._debug_scores and rms >= 250:
            self.get_logger().info(
                f'  barge-rms {rms:.0f} (thr {self._barge_threshold:.0f}, run {self._barge_run})',
                throttle_duration_sec=0.3)
        if rms >= self._barge_threshold:
            self._barge_run += 1
        else:
            self._barge_run = 0
        now = time.monotonic()
        if self._barge_run >= self._barge_hold and (now - self._last_barge) > self._debounce:
            self._last_barge = now
            self._barge_run = 0
            self.get_logger().info(f'Energy barge-in (rms {rms:.0f})')
            self.barge_pub.publish(Empty())
            d = String(); d.data = 'bargein'   # red LED flash
            self.detect_pub.publish(d)

    def _handle_scores(self, scores):
        now = time.monotonic()
        for key, score in scores.items():
            score = float(score)
            thr = self._thresholds.get(key, self._default_threshold)
            if score >= thr and (now - self._last_fire.get(key, 0.0)) > self._debounce:
                self._last_fire[key] = now
                self._fire_wake(key, score)
            elif self._debug_scores and score >= DEBUG_FLOOR:
                self.get_logger().info(f'  near-miss {key}={score:.2f} (thr {thr})',
                                       throttle_duration_sec=0.4)

    def _fire_wake(self, key, score):
        phrase = _key_to_phrase(key)
        # Echo the raw key for LED feedback (green flash for wake, red for cancel).
        d = String()
        d.data = key
        self.detect_pub.publish(d)
        if key in self._cancel_keys:
            # Cancel word ("stop"): abort the reply/playback; do NOT open a window.
            self.get_logger().info(f"Cancel: '{phrase}' ({score:.2f})")
            self.barge_pub.publish(Empty())
            return
        speaking = self.speaking
        self.get_logger().info(
            f"Wake: '{phrase}' ({score:.2f})" + (' [barge-in]' if speaking else ''))
        m = String()
        m.data = phrase
        self.wake_pub.publish(m)
        if speaking:
            self.barge_pub.publish(Empty())


def main(args=None):
    rclpy.init(args=args)
    node = WakeWordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
