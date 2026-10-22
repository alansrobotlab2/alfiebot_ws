"""
asr_node — hearing: Silero VAD + turn-1-mini end-of-turn + Parakeet ASR.

Audio arrives on `audio_frames` (int16[512] @ 16 kHz). Three threads, so no
stage can starve another (lloyd's "one stream, one clock, one thread"):

  * the ROS callback only enqueues frames on an *unbounded* queue — a bounded
    one that dropped would reintroduce the silent gaps this rework removes;
  * the hearing thread runs ``hearing.HearingPipeline`` (VAD with pre-roll, the
    streamed turn model, the hold) and publishes `vad/speech` edges at once;
  * the transcription thread runs Parakeet on each committed turn and publishes
    `asrresult` with timing fields for the agent's latency log.

Topics:
  `vad/speech` (Bool)    true at a speech onset, false when a segment closes.
                         The agent uses it for sustained-speech barge-in.
  `asrresult` (ASRResult) one committed turn: transcript + speech_end time,
                         durations, the turn score and the commit reason.

Optional `capture_dir`: every committed turn is saved as a 16 kHz WAV plus a
JSON line, so misses can be replayed through the same pipeline
(`ros2 run alfie_asr replay <wav>`).
"""
import os
import sys

# Set ONNX Runtime options BEFORE importing any ONNX-related libraries
os.environ['ORT_DISABLE_AFFINITY'] = '1'  # Disable thread affinity
os.environ['OMP_NUM_THREADS'] = '4'  # Set number of OpenMP threads

# No HF_HUB_OFFLINE here: model_manager loads from ~/.alfie_asr/models and only
# touches HuggingFace for the one-time download when that directory is empty.

import json
import queue
import threading
import time
import wave
from pathlib import Path

import numpy as np
import onnxruntime as ort
import onnx_asr

from alfie_asr.model_manager import (ensure_models, ensure_speaker_model,
                                     ensure_turn_model, ModelNotAvailableError)

# IMPORTANT: Load the ASR model before anything that might initialise CUDA
# through PyTorch (cuBLAS handle conflict). Nothing below imports torch any more
# — the VAD runs through onnxruntime directly (hearing.SileroOnnx) — but the
# order is kept so a future import can't reintroduce the clash.
print("Checking for local models...")
try:
    _asr_model_dir, _vad_model_path = ensure_models()
except ModelNotAvailableError as e:
    print(f"FATAL: {e}")
    sys.exit(1)
_turn_model_path = ensure_turn_model()
_speaker_model_path = ensure_speaker_model()

print("Pre-loading Parakeet ASR model with GPU...")
ort.set_default_logger_severity(3)
_sess_options = ort.SessionOptions()
_sess_options.intra_op_num_threads = 4
_sess_options.inter_op_num_threads = 4
_sess_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
_sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
_providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
_asr_model = onnx_asr.load_model(
    "istupakov/parakeet-tdt-0.6b-v2-onnx",
    path=str(_asr_model_dir),
    quantization="int8",
    sess_options=_sess_options,
    providers=_providers
)
print("ASR model pre-loaded successfully")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from alfie_msgs.msg import AudioFrame, ASRResult
from alfie_msgs.srv import EnrollSpeaker

from alfie_asr.hearing import SAMPLE_RATE, build_pipeline
from alfie_asr.speaker_id import SpeakerID


class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')

        # --- parameters (lloyd's measured defaults; see hearing.py) ---
        vad_threshold = float(self.declare_parameter('vad_threshold', 0.45).value)
        # 0 = auto: 380 ms with the turn model, 500 ms without.
        min_silence_ms = int(self.declare_parameter('min_silence_ms', 0).value)
        speech_pad_ms = int(self.declare_parameter('speech_pad_ms', 220).value)
        min_utterance_ms = int(self.declare_parameter('min_utterance_ms', 250).value)
        max_utterance_ms = int(self.declare_parameter('max_utterance_ms', 30000).value)
        turn_enabled = bool(self.declare_parameter('turn_enabled', True).value)
        turn_path = self.declare_parameter(
            'turn_model_path', str(_turn_model_path or '')).value
        turn_threshold = float(self.declare_parameter('turn_threshold', 0.06).value)
        hold_timeout_ms = int(self.declare_parameter('hold_timeout_ms', 2200).value)
        hold_max_s = float(self.declare_parameter('hold_max_s', 12.0).value)
        capture_dir = self.declare_parameter('capture_dir', '').value
        # Speaker ID (speaker_id.py): CAM++ voiceprints on CPU, ~25 ms per turn.
        speaker_enabled = bool(self.declare_parameter('speaker_id_enabled', True).value)
        speaker_threshold = float(self.declare_parameter('speaker_threshold', 0.5).value)
        speaker_store = self.declare_parameter(
            'speaker_store', '~/.alfie_asr/voiceprints.json').value

        self.pipeline = build_pipeline(
            _vad_model_path, turn_path if turn_enabled else None,
            vad_threshold=vad_threshold, min_silence_ms=min_silence_ms,
            speech_pad_ms=speech_pad_ms, min_utterance_ms=min_utterance_ms,
            max_utterance_ms=max_utterance_ms, turn_threshold=turn_threshold,
            hold_timeout_ms=hold_timeout_ms, hold_max_s=hold_max_s,
            log=self.get_logger().warn)
        self.asr_model = _asr_model
        self._capture_dir = Path(capture_dir).expanduser() if capture_dir else None
        if self._capture_dir:
            self._capture_dir.mkdir(parents=True, exist_ok=True)

        self.speaker_id = None
        if speaker_enabled and _speaker_model_path is not None:
            try:
                self.speaker_id = SpeakerID(_speaker_model_path, speaker_store,
                                            threshold=speaker_threshold)
                self.create_service(EnrollSpeaker, 'asr/enroll_speaker',
                                    self._on_enroll)
                self.get_logger().info(
                    f'speaker ID on: {len(self.speaker_id.names())} voice(s) enrolled')
            except Exception as e:
                self.get_logger().error(f'speaker ID unavailable: {e}')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher_ = self.create_publisher(ASRResult, 'asrresult', qos)
        self.vad_pub = self.create_publisher(Bool, 'vad/speech', qos)

        # (frame, wall arrival time). Unbounded on purpose: never drop audio.
        self._frames = queue.Queue()
        self._turns = queue.Queue()
        threading.Thread(target=self._hearing_loop, daemon=True).start()
        threading.Thread(target=self._asr_loop, daemon=True).start()

        self.create_subscription(AudioFrame, 'audio_frames', self.audio_callback, qos)

        seg = self.pipeline.segmenter
        self.get_logger().info(
            f'ASRNode initialized: vad thr {seg.threshold:.2f}, min silence '
            f'{seg._min_silence * 1000 // SAMPLE_RATE} ms, pad {speech_pad_ms} ms, '
            f'turn model {"on (thr %.2f)" % turn_threshold if self.pipeline.turn else "off"}.')

    def audio_callback(self, msg):
        # Audio is always processed (even while the robot speaks): the reSpeaker's
        # hardware AEC removes the robot's own voice, and the agent decides what
        # an utterance during speech means (barge-in, backchannel or echo).
        self._frames.put((np.frombuffer(msg.audioframe, dtype=np.int16).copy(),
                          time.time()))

    def _publish_vad(self, value):
        m = Bool()
        m.data = value
        self.vad_pub.publish(m)

    def _hearing_loop(self):
        while rclpy.ok():
            frame, arrived = self._frames.get()
            try:
                events = self.pipeline.feed(frame)
            except Exception as e:
                self.get_logger().error(f'hearing pipeline failed: {e}')
                continue
            # `arrived` is the wall time of the newest audio; events refer back
            # into it by sample count.
            backlog = self._frames.qsize()
            if backlog > 30:  # ~1 s behind real time
                self.get_logger().warn(f'hearing is {backlog} frames behind',
                                       throttle_duration_sec=5.0)
            for ev in events:
                if ev.kind == 'speech_start':
                    self._publish_vad(True)
                elif ev.kind == 'speech_end':
                    self._publish_vad(False)
                elif ev.kind == 'turn':
                    speech_end = arrived - ev.lag_samples / SAMPLE_RATE
                    self.get_logger().info(
                        f'turn committed: {ev.audio.size / SAMPLE_RATE:.2f}s '
                        f'({ev.reason}, p={ev.turn_prob:.3f})')
                    self._turns.put((ev, speech_end))

    def _asr_loop(self):
        while rclpy.ok():
            ev, speech_end = self._turns.get()
            if ev.audio is None or ev.audio.size == 0:
                continue
            t0 = time.monotonic()
            try:
                text = str(self.asr_model.recognize(ev.audio, sample_rate=SAMPLE_RATE))
            except Exception as e:
                self.get_logger().error(f'transcription failed: {e}')
                continue
            asr_s = time.monotonic() - t0
            speaker, score, spk_ms = '', 0.0, 0.0
            if self.speaker_id is not None:
                t1 = time.monotonic()
                try:
                    speaker, score = self.speaker_id.identify(ev.audio)
                except Exception as e:
                    self.get_logger().warn(f'speaker ID failed: {e}',
                                           throttle_duration_sec=30.0)
                spk_ms = (time.monotonic() - t1) * 1000
            msg = ASRResult()
            msg.asrresult = text
            msg.speech_end = float(speech_end)
            msg.duration = float(ev.audio.size / SAMPLE_RATE)
            msg.asr_seconds = float(asr_s)
            msg.turn_prob = float(ev.turn_prob)
            msg.reason = ev.reason
            msg.speaker = speaker
            msg.speaker_score = float(score)
            self.publisher_.publish(msg)
            who = f' [{speaker or "?"} {score:.2f}, {spk_ms:.0f} ms]' if spk_ms else ''
            self.get_logger().info(
                f'[asr] {msg.duration:.2f}s audio in {asr_s * 1000:.0f} ms: {text!r}{who}')
            if self._capture_dir:
                self._capture(ev, msg)

    def _on_enroll(self, req, resp):
        if req.forget:
            resp.ok, resp.message = self.speaker_id.forget(req.name)
        else:
            resp.ok, resp.message = self.speaker_id.enroll(req.name)
        self.get_logger().info(f'enroll_speaker({req.name!r}, forget={req.forget}): '
                               f'{resp.message}')
        return resp

    def _capture(self, ev, msg):
        stamp = time.strftime('%Y%m%d_%H%M%S') + f'_{int(time.time() * 1000) % 1000:03d}'
        try:
            pcm = np.clip(ev.audio * 32768.0, -32768, 32767).astype(np.int16)
            with wave.open(str(self._capture_dir / f'{stamp}.wav'), 'wb') as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(SAMPLE_RATE)
                w.writeframes(pcm.tobytes())
            with open(self._capture_dir / 'turns.jsonl', 'a') as f:
                f.write(json.dumps({'wav': f'{stamp}.wav', 'text': msg.asrresult,
                                    'reason': msg.reason, 'turn_prob': msg.turn_prob,
                                    'asr_seconds': msg.asr_seconds}) + '\n')
        except Exception as e:
            self.get_logger().warn(f'capture failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ASRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
