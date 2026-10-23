"""
idle_behavior — low-priority "life" behaviors for Alfie.

Publishes gentle idle motion to the command_mux at the lowest motion priority
(`cmd/eyes/idle`, `cmd/head/idle`). Any real commander (VR, GR00T, agent gaze)
outranks it, so this only shows through when nothing else owns the head/eyes.

    cmd/eyes/idle  (EyeCmd)  — constant dim glow; blinks off for `eye_ack_sec`
                               to acknowledge a wake word or an interrupt
    cmd/head/idle  (HeadCmd) — head behavior, chosen by `head_mode`:

      wander  — small open-loop sinusoidal look-around.
      attend  — look at whoever is talking / nearby:
                * yaw turns toward the reSpeaker direction-of-arrival while the
                  ASR's VAD (`vad/speech`) reports speech — the XVF3800's own
                  speech flag fires on room noise — ignored while Alfie speaks.
                * pitch (and a yaw refinement) centers the nearest face (largest
                  box) at `face_rate_hz`. `face_backend: yunet` runs OpenCV's
                  YuNet on the CPU over the left_wide camera (~10 ms, no GPU);
                  `nanoowl` polls the on-demand `nanoowl/detect` service with
                  "a face" (GPU — paused for a conversation turn so it doesn't
                  slow ASR / the LLM). YuNet falls back to NanoOWL if its model
                  can't load. The image offset is
                  applied to the head pose at the frame's capture time (from a
                  short pose history), so it tracks while the head is moving.
                The reSpeaker sits on head_link, so DOA is head-relative: the yaw
                target is current yaw + DOA offset, sampled only while the head is
                settled. The XVF3800 holds its last DOA estimate until new speech
                re-converges it, so after each turn the pre-turn reading is
                ignored until it changes (else it gets added a second time).
                While someone is talking the head rolls a little to the left
                (a listening tilt), then levels out. While Alfie speaks, the
                head nods on stressed syllables, driven by the TTS loudness
                envelope (`tts/level`). Face polling
                pauses while a higher-priority head source is active or the
                LLM is generating, so it doesn't steal GPU.
"""

import bisect
import collections
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Empty, Float32, String
from sensor_msgs.msg import CompressedImage
from alfie_msgs.msg import Doa, EyeCmd, HeadCmd, HeadState, ServoCmd, Speaking
from alfie_msgs.srv import Detect

YAW, PITCH, ROLL = 0, 1, 2  # HeadCmd.servos / HeadState.servos order


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class IdleBehaviorNode(Node):

    def __init__(self):
        super().__init__('idle_behavior_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ---- Parameters -----------------------------------------------------
        self.declare_parameter('rate_hz', 50.0)  # = head firmware servo loop (attend ramp)
        self.declare_parameter('eyes_enabled', True)
        self.declare_parameter('eye_idle_pwm', 1)
        self.declare_parameter('eye_ack_sec', 0.333)  # eyes-off blink on wake / barge-in
        self.declare_parameter('head_enabled', True)
        self.declare_parameter('head_mode', 'attend')  # 'attend' | 'wander'
        # wander
        self.declare_parameter('head_yaw_amp_rad', 0.15)
        self.declare_parameter('head_pitch_amp_rad', 0.05)
        self.declare_parameter('head_period_sec', 12.0)
        self.declare_parameter('head_speed', 0.3)
        # attend: joint conventions + limits
        self.declare_parameter('attend_speed', 1.5)          # rad/s servo speed
        # Trapezoidal ramp, run host-side (see _profile); 0 = instant, which looks
        # mechanical. Applies to accel and decel.
        self.declare_parameter('attend_accel', 4.0)          # rad/s^2
        self.declare_parameter('smooth_hz', 2.0)  # yaw/pitch setpoint low-pass (see _smooth); 0 = off
        self.declare_parameter('yaw_limits', [-3.14, 3.14])  # rad; yaw servo travel is +/-pi
        # A DOA target past a yaw limit by more than this goes round the other way.
        self.declare_parameter('yaw_unwind_margin', 0.35)  # rad (~20 deg)
        # Servo frame (+ = down). Up matches the head firmware's stop (~-0.72);
        # faces further up than that (+ face_max_above) are rejected as ceiling.
        self.declare_parameter('pitch_limits', [-0.7, 0.4])  # rad
        self.declare_parameter('neutral_pitch', 0.0)
        # Measured live 2026-10-18: servo +yaw turns right, +pitch looks down.
        self.declare_parameter('yaw_sign', -1.0)    # +1: positive yaw turns left (CCW)
        self.declare_parameter('pitch_sign', -1.0)  # +1: positive pitch looks up
        self.declare_parameter('settle_tol_rad', 0.08)  # head "settled" if within this
        self.declare_parameter('settle_sec', 0.5)       # ...and target unchanged this long
        # attend: DOA
        # Calibrated 2026-10-18: front 180, left 91, behind 1 -> angle decreases CCW.
        self.declare_parameter('doa_sign', -1.0)         # +1: DOA angle increases CCW
        self.declare_parameter('doa_offset_deg', 180.0)  # DOA reading when speaker is dead ahead
        self.declare_parameter('doa_fresh_deg', 3.0)     # post-turn: reading must move this much
        self.declare_parameter('doa_held_sec', 8.0)  # unchanged this long before speech: stale
        self.declare_parameter('speech_gate', 'asr')     # 'asr' (vad/speech) | 'array' (XVF VAD)
        self.declare_parameter('doa_onset_delay_sec', 0.4)  # DOA lags a new speaker
        # From a long-held estimate the array can take ~1 s to converge: longer
        # than a bare "hey alfie". Keep taking samples the array itself flags as
        # speech for this long after vad/speech drops.
        self.declare_parameter('doa_tail_sec', 2.0)
        self.declare_parameter('doa_min_samples', 3)     # consecutive speech samples (10 Hz)
        self.declare_parameter('doa_max_spread_deg', 25.0)
        self.declare_parameter('doa_deadband_deg', 15.0)
        # Look up this much (from neutral) on a DOA turn: the speaker is usually
        # standing, and a level gaze can miss the face to lock onto.
        self.declare_parameter('doa_pitch_up_deg', 15.0)
        self.declare_parameter('speaking_holdoff_sec', 0.7)  # ignore DOA after own TTS
        # attend: faces
        self.declare_parameter('face_enabled', True)
        self.declare_parameter('face_backend', 'yunet')  # 'yunet' (CPU) | 'nanoowl' (GPU)
        self.declare_parameter(
            'yunet_model', '/home/alfie/face_models/face_detection_yunet_2022mar.onnx')
        self.declare_parameter('yunet_score', 0.6)          # keep following down to this
        self.declare_parameter('face_acquire_score', 0.85)  # yunet: to turn to a new face
        self.declare_parameter(
            'face_image_topic', 'stereo_camera/left_wide/image_raw/compressed')
        self.declare_parameter('face_turn_pause_sec', 6.0)  # nanoowl: max pause after speech
        self.declare_parameter('face_service', 'nanoowl/detect')
        self.declare_parameter('face_prompt', 'a face')
        self.declare_parameter('face_threshold', 0.15)
        self.declare_parameter('face_rate_hz', 10.0)     # yunet: every frame at the 10 fps camera
        self.declare_parameter('face_refine_yaw', True)
        self.declare_parameter('face_deadband', 0.06)    # normalized image units, to start moving
        # Once the face is off center by `face_deadband`, follow it continuously
        # (alpha-beta filtered direction + velocity) until it has been still —
        # under `face_vel_min` and within `face_retarget_rad` of the target — for
        # `face_track_hold_sec`.
        self.declare_parameter('face_alpha', 0.5)        # position gain per fix
        self.declare_parameter('face_beta', 0.15)        # velocity gain per fix
        self.declare_parameter('face_vel_min', 0.05)     # rad/s; slower counts as still
        self.declare_parameter('face_retarget_rad', 0.025)
        self.declare_parameter('face_predict_sec', 0.3)  # max extrapolation past the last fix
        self.declare_parameter('face_track_hold_sec', 1.0)
        self.declare_parameter('face_confirm_rad', 0.1)  # same face in consecutive detections
        # Normalized; wider is rejected. 0.2 = ~70 px = a face ~30 cm from the lens
        # (one at 0.5 m is ~40 px); a ceiling fan scored 0.9+ at ~100 px.
        self.declare_parameter('face_max_width', 0.2)
        self.declare_parameter('face_max_above', 0.15)  # rad past the pitch up-limit
        # Capture -> publish delay not covered by the image stamp (the camera node
        # stamps at appsink: exposure + USB + JPEG decode/resize/encode).
        self.declare_parameter('camera_latency_sec', 0.1)
        # left_wide; measured 2026-10-19 from image shift vs head yaw (~136 px/rad)
        self.declare_parameter('camera_hfov_deg', 104.0)
        # After a DOA turn, only a face near the speaker's direction is tracked
        # (else the face being looked at drags the head back mid-turn).
        self.declare_parameter('speaker_lock_sec', 4.0)    # extended while they talk
        self.declare_parameter('speaker_match_deg', 26.0)  # face must be this close
        self.declare_parameter('face_lost_sec', 8.0)     # then pitch returns to neutral
        self.declare_parameter('attend_timeout_sec', 45.0)  # then yaw returns to center
        # attend: listening tilt (roll) while vad/speech is on
        self.declare_parameter('listen_roll_rad', 0.15)    # servo frame; sign = "to the left"
        self.declare_parameter('listen_roll_hold_sec', 1.0)  # keep the tilt after speech ends
        self.declare_parameter('roll_speed', 0.5)          # rad/s, gentler than yaw/pitch
        self.declare_parameter('roll_accel', 1.5)          # rad/s^2
        # attend: speech nods, beat-triggered from the TTS loudness envelope
        self.declare_parameter('nod_enabled', True)
        self.declare_parameter('nod_amp_rad', 0.06)        # peak dip (down) per beat
        self.declare_parameter('nod_peak_sec', 0.15)       # time to the bottom of the dip
        self.declare_parameter('nod_refractory_sec', 0.35)  # min time between beats
        self.declare_parameter('nod_onset_ratio', 0.5)     # beat when env > ratio * recent peak
        self.declare_parameter('nod_min_level', 0.01)      # RMS floor (ignore near-silence)
        self.declare_parameter('higher_head_sources', ['init', 'tool', 'vr', 'groot', 'agent'])
        self.declare_parameter('higher_source_timeout', 0.5)

        g = self.get_parameter
        self.rate = float(g('rate_hz').value)
        self.eyes_enabled = bool(g('eyes_enabled').value)
        self.eye_idle_pwm = max(0, min(4095, int(g('eye_idle_pwm').value)))
        self.eye_ack_sec = float(g('eye_ack_sec').value)
        self.head_enabled = bool(g('head_enabled').value)
        self.head_mode = str(g('head_mode').value)
        self.head_yaw_amp = float(g('head_yaw_amp_rad').value)
        self.head_pitch_amp = float(g('head_pitch_amp_rad').value)
        self.head_period = float(g('head_period_sec').value)
        self.head_speed = float(g('head_speed').value)

        self.attend_speed = float(g('attend_speed').value)
        self.attend_accel = float(g('attend_accel').value)
        smooth_hz = float(g('smooth_hz').value)
        self.smooth_w = 2.0 * math.pi * smooth_hz
        self.smooth_lag = 1.0 / (math.pi * smooth_hz) if smooth_hz > 0.0 else 0.0
        self.yaw_lim = [float(v) for v in g('yaw_limits').value]
        self.yaw_unwind_margin = float(g('yaw_unwind_margin').value)
        self.pitch_lim = [float(v) for v in g('pitch_limits').value]
        self.neutral_pitch = float(g('neutral_pitch').value)
        self.yaw_sign = float(g('yaw_sign').value)
        self.pitch_sign = float(g('pitch_sign').value)
        self.settle_tol = float(g('settle_tol_rad').value)
        self.settle_sec = float(g('settle_sec').value)
        self.doa_sign = float(g('doa_sign').value)
        self.doa_offset = math.radians(float(g('doa_offset_deg').value))
        self.doa_min_samples = max(1, int(g('doa_min_samples').value))
        self.doa_max_spread = math.radians(float(g('doa_max_spread_deg').value))
        self.doa_deadband = math.radians(float(g('doa_deadband_deg').value))
        self.doa_pitch_up = math.radians(float(g('doa_pitch_up_deg').value))
        self.speaking_holdoff = float(g('speaking_holdoff_sec').value)
        self.doa_fresh = math.radians(float(g('doa_fresh_deg').value))
        self.doa_held_sec = float(g('doa_held_sec').value)
        self.speech_gate = str(g('speech_gate').value)
        self.doa_onset_delay = float(g('doa_onset_delay_sec').value)
        self.doa_tail = float(g('doa_tail_sec').value)
        self.face_enabled = bool(g('face_enabled').value)
        self.face_prompt = str(g('face_prompt').value)
        self.face_backend = str(g('face_backend').value)
        self.face_turn_pause = float(g('face_turn_pause_sec').value)
        self.face_threshold = float(g('face_threshold').value)
        self.face_period = 1.0 / max(0.05, float(g('face_rate_hz').value))
        self.face_refine_yaw = bool(g('face_refine_yaw').value)
        self.face_deadband = float(g('face_deadband').value)
        self.face_retarget = float(g('face_retarget_rad').value)
        self.face_confirm = float(g('face_confirm_rad').value)
        self.face_max_width = float(g('face_max_width').value)
        self.face_max_above = float(g('face_max_above').value)
        self.face_acquire_score = float(g('face_acquire_score').value)
        self.face_prev = (0.0, [])  # (time, aims) of the previous detection
        self.face_alpha = float(g('face_alpha').value)
        self.face_beta = float(g('face_beta').value)
        self.face_vel_min = float(g('face_vel_min').value)
        self.face_predict = float(g('face_predict_sec').value)
        self.fa = [0.0, 0.0]        # followed face direction [yaw, pitch] (filtered)
        self.fv = [0.0, 0.0]        # ...and its angular velocity
        self.fa_t = 0.0             # capture time of the last fix
        self.face_track_hold = float(g('face_track_hold_sec').value)
        self.follow_until = 0.0     # following a face (continuous target) until then
        self.camera_latency = float(g('camera_latency_sec').value)
        hfov = math.radians(float(g('camera_hfov_deg').value))
        self.tan_half_h = math.tan(hfov / 2.0)
        self.tan_half_v = self.tan_half_h * 256.0 / 352.0  # refined from detection dims
        self.face_lost = float(g('face_lost_sec').value)
        self.speaker_lock = float(g('speaker_lock_sec').value)
        self.speaker_match = math.radians(float(g('speaker_match_deg').value))
        self.speaker_yaw = None   # yaw target of the current DOA speaker
        self.doa_look_until = 0.0  # keep the raised DOA pitch until then if no face
        self.speaker_until = 0.0
        self.attend_timeout = float(g('attend_timeout_sec').value)
        self.listen_roll = float(g('listen_roll_rad').value)
        self.listen_roll_hold = float(g('listen_roll_hold_sec').value)
        self.roll_speed = float(g('roll_speed').value)
        self.roll_accel = float(g('roll_accel').value)
        self.higher_timeout = float(g('higher_source_timeout').value)
        self.nod_enabled = bool(g('nod_enabled').value)
        self.nod_amp = float(g('nod_amp_rad').value)
        self.nod_peak = max(0.02, float(g('nod_peak_sec').value))
        self.nod_refractory = float(g('nod_refractory_sec').value)
        self.nod_onset_ratio = float(g('nod_onset_ratio').value)
        self.nod_min_level = float(g('nod_min_level').value)

        # ---- Publishers -----------------------------------------------------
        self.eyes_pub = self.create_publisher(EyeCmd, 'cmd/eyes/idle', qos)
        self.eye_ack_until = 0.0
        self.eye_ack_timer = None
        if self.eyes_enabled and self.eye_ack_sec > 0.0:
            # wakeword/detection carries wake, cancel ("stop") and energy barge-in;
            # barge_in[/soft] also catches interrupts raised by other nodes.
            self.create_subscription(
                String, 'wakeword/detection', lambda _m: self._eye_ack(), 10)
            self.create_subscription(Empty, 'barge_in', lambda _m: self._eye_ack(), qos)
            self.create_subscription(Empty, 'barge_in/soft', lambda _m: self._eye_ack(), qos)
        self.head_pub = self.create_publisher(HeadCmd, 'cmd/head/idle', qos)

        # ---- Attend state ---------------------------------------------------
        self.cur = None             # [yaw, pitch] from low/headstate
        self.cur_roll = 0.0
        self.sp = None              # commanded [yaw, pitch, roll] setpoint (host-side ramp)
        self.sv = [0.0, 0.0, 0.0]   # its velocity
        self.sp_t = 0.0
        self.sm, self.smv = [0.0, 0.0], [0.0, 0.0]  # smoothed yaw/pitch and velocity
        self.pose_hist = collections.deque(maxlen=200)  # (t, yaw, pitch), ~2-4 s
        self.still_ref = None       # pose the head was last seen still at
        self.last_motion = 0.0      # last time the head moved > 0.01 rad
        self.target = None          # [yaw, pitch] we command
        self.target_changed = 0.0   # time the target last moved
        self.last_attention = 0.0   # last accepted DOA or face fix
        self.last_face = 0.0
        self.doa_buf = []           # recent head-relative DOA (rad, CCW+)
        self.last_doa_angle = None  # latest raw DOA reading (deg)
        self.doa_stale_angle = None  # raw reading held from before the last turn
        self.doa_onset_angle = None  # long-held raw reading at the last speech onset
        self.doa_changed_t = 0.0     # last time the raw reading changed
        self.doa_why = collections.Counter()  # rejections this utterance (for the log)
        self.doa_seen = []                    # raw readings this utterance (for the log)
        self.speaking_until = 0.0
        self.asr_speech_since = None  # vad/speech onset time while speech is on
        self.asr_speech_ended = 0.0   # last vad/speech falling edge
        self.turn_until = 0.0         # nanoowl paused until then (speech -> reply)
        self.listening = False
        self.tts_env = 0.0          # TTS loudness envelope (RMS, attack/release)
        self.tts_peak = 0.0         # slowly decaying recent envelope peak
        self.tts_level_t = None
        self.nod_armed = True       # re-armed once the envelope dips between beats
        self.nod_t = None           # time of the last beat
        self.generating = False
        self.higher_seen = 0.0
        self.face_pending_since = None
        self.last_face_call = 0.0

        if self.head_enabled and self.head_mode == 'attend':
            self.create_subscription(HeadState, 'low/headstate', self._on_head_state, qos)
            self.create_subscription(Doa, 'respeaker/doa', self._on_doa, qos)
            self.create_subscription(Speaking, 'speaking', self._on_speaking, qos)
            self.create_subscription(Bool, 'vad/speech', self._on_vad, qos)
            if self.nod_enabled:
                self.create_subscription(Float32, 'tts/level', self._on_tts_level, qos)
            self.create_subscription(Bool, 'generating', self._on_generating, qos)
            for src in g('higher_head_sources').value:
                self.create_subscription(
                    HeadCmd, f'cmd/head/{src}', self._on_higher_source, qos)
            self.face_cli = self.create_client(Detect, str(g('face_service').value))
            self.yunet = None
            self.face_img = None
            if self.face_enabled and self.face_backend == 'yunet':
                self._init_yunet(str(g('yunet_model').value), float(g('yunet_score').value),
                                 str(g('face_image_topic').value), qos)

        self.start_ns = self.get_clock().now().nanoseconds
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f'idle_behavior started (eyes={self.eyes_enabled}, '
            f'head={self.head_enabled}, mode={self.head_mode})')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _elapsed(self) -> float:
        return (self.get_clock().now().nanoseconds - self.start_ns) / 1e9

    # ---- attend inputs ------------------------------------------------------
    def _on_head_state(self, msg: HeadState) -> None:
        self.cur = [msg.servos[YAW].current_location, msg.servos[PITCH].current_location]
        self.cur_roll = msg.servos[ROLL].current_location
        now = self._now()
        self.pose_hist.append((now, self.cur[0], self.cur[1]))
        if self.target is None:
            self.target = list(self.cur)  # start from wherever the head is
        if (self.still_ref is None or abs(self.cur[0] - self.still_ref[0]) > 0.01
                or abs(self.cur[1] - self.still_ref[1]) > 0.01):
            self.still_ref = list(self.cur)
            self.last_motion = now

    def _init_yunet(self, model: str, score: float, topic: str, qos) -> None:
        try:
            import cv2
            import numpy as np
            self._cv2, self._np = cv2, np
            cv2.setNumThreads(2)
            self.yunet = cv2.FaceDetectorYN.create(model, '', (352, 256), score, 0.3, 20)
            self.yunet_size = (352, 256)
        except Exception as e:
            self.get_logger().warn(f'YuNet unavailable ({e}); falling back to NanoOWL')
            self.face_backend = 'nanoowl'
            return
        self.create_subscription(CompressedImage, topic, self._on_face_img, qos)
        self.get_logger().info(f'face tracking: YuNet on {topic}')

    def _on_face_img(self, msg: CompressedImage) -> None:
        # Detect as frames arrive rather than on the next tick (saves up to a tick).
        self.face_img = msg
        now = self._now()
        if (self.target is not None and now >= self.speaking_until
                and not self._outranked(now)):
            self._detect_yunet(now)

    def _on_speaking(self, msg: Speaking) -> None:
        if msg.is_speaking:
            self.turn_until = 0.0  # reply has started: the turn's GPU rush is over
            self.speaking_until = float('inf')
        elif self.speaking_until == float('inf'):
            self.speaking_until = self._now() + self.speaking_holdoff

    def _on_vad(self, msg: Bool) -> None:
        if not msg.data:
            if self.asr_speech_since is not None:
                self.asr_speech_ended = self._now()
            self.asr_speech_since = None
            if self.turn_until == float('inf'):
                self.turn_until = self._now() + self.face_turn_pause
        elif self.asr_speech_since is None:
            self.asr_speech_since = self._now()
            # The chip holds its last estimate until speech re-converges it. A new
            # speaker moves the reading within moments of speaking (often just
            # *before* the ASR VAD fires); one unchanged for long before onset is
            # an old held value (e.g. from before this node started).
            if (self.last_doa_angle is not None
                    and self._now() - self.doa_changed_t > self.doa_held_sec):
                self.doa_onset_angle = self.last_doa_angle
            self.turn_until = float('inf')  # ASR + LLM are about to want the GPU

    def _speech(self, msg: Doa, now: float) -> bool:
        if self.speech_gate == 'array':
            return msg.speech_detected
        if self.asr_speech_since is None:
            return msg.speech_detected and now - self.asr_speech_ended < self.doa_tail
        return now - self.asr_speech_since > self.doa_onset_delay

    def _on_tts_level(self, msg: Float32) -> None:
        # ~42 Hz: one RMS value per 512-sample window, published just before play.
        now = self._now()
        dt = 0.024 if self.tts_level_t is None else min(0.2, now - self.tts_level_t)
        self.tts_level_t = now
        level = float(msg.data)
        tau = 0.03 if level > self.tts_env else 0.12  # fast attack, slower release
        self.tts_env += (level - self.tts_env) * (1.0 - math.exp(-dt / tau))
        self.tts_peak = max(self.tts_env, self.tts_peak * math.exp(-dt / 2.0))
        thresh = max(self.nod_min_level, self.nod_onset_ratio * self.tts_peak)
        if self.tts_env < 0.6 * thresh:
            self.nod_armed = True
        elif (self.nod_armed and self.tts_env > thresh
                and (self.nod_t is None or now - self.nod_t > self.nod_refractory)):
            self.nod_armed = False
            self.nod_t = now

    def _nod_offset(self, now: float) -> float:
        """Pitch offset toward 'down' for the current beat (alpha-function dip)."""
        if self.nod_t is None:
            return 0.0
        x = (now - self.nod_t) / self.nod_peak
        if x > 5.0:
            return 0.0
        return self.nod_amp * x * math.exp(1.0 - x)

    def _on_generating(self, msg: Bool) -> None:
        self.generating = bool(msg.data)

    def _on_higher_source(self, _msg: HeadCmd) -> None:
        self.higher_seen = self._now()

    def _outranked(self, now: float) -> bool:
        return now - self.higher_seen < self.higher_timeout

    def _settled(self, now: float) -> bool:
        if self.cur is None or self.target is None:
            return False
        if now - self.target_changed <= self.settle_sec:
            return False
        reached = (abs(self.cur[0] - self.target[0]) < self.settle_tol
                   and abs(self.cur[1] - self.target[1]) < self.settle_tol)
        # A target past a firmware joint limit is never reached; stopped counts too.
        return reached or now - self.last_motion > 0.3

    def _set_target(self, yaw: float, pitch: float, now: float) -> None:
        yaw = _clamp(yaw, *self.yaw_lim)
        pitch = _clamp(pitch, *self.pitch_lim)
        if abs(yaw - self.target[0]) > 1e-3 or abs(pitch - self.target[1]) > 1e-3:
            if abs(yaw - self.target[0]) > 0.1:
                # The array turns with the head: its held estimate (and any samples
                # collected so far) are wrong after a real turn. Small face nudges
                # don't count, or face tracking would starve DOA entirely.
                self.doa_stale_angle = self.last_doa_angle
                self.doa_buf.clear()
            self.target = [yaw, pitch]
            self.target_changed = now

    def _yaw_still(self, now: float, window: float = 0.5, tol: float = 0.05) -> bool:
        """Head yaw stayed within `tol` over the last `window` s (array not turning)."""
        if self.cur is None or not self.pose_hist or now - self.pose_hist[0][0] < window:
            return False
        recent = [y for t, y, _ in self.pose_hist if t >= now - window]
        return bool(recent) and max(recent) - min(recent) < tol

    def _unwind(self, yaw: float) -> float:
        """Bring a yaw past a limit round the other way (same direction, -/+ 2 pi).

        The yaw servo can't pass through +/-pi, so facing backwards with the
        speaker on the far side the head must swing back round through the
        front. Only just past the limit, clamping is close enough.
        """
        lo, hi = self.yaw_lim
        over = max(lo - yaw, yaw - hi)
        if over <= self.yaw_unwind_margin:
            return _clamp(yaw, lo, hi)
        alt = yaw - math.copysign(2.0 * math.pi, yaw)
        return _clamp(alt, lo, hi)

    def _doa_reject(self, msg: Doa, now: float):
        """Why this DOA sample can't be used, or None if it can."""
        if self.doa_stale_angle is not None:
            if abs(_wrap(math.radians(msg.angle - self.doa_stale_angle))) <= self.doa_fresh:
                return 'stale'  # still the estimate from before the turn
            self.doa_stale_angle = None
        if self.doa_onset_angle is not None:
            if msg.angle == self.doa_onset_angle:
                return 'held'  # unchanged since speech began: still the held estimate
            self.doa_onset_angle = None
        if not self._speech(msg, now):
            return 'no_speech'
        if now < self.speaking_until:
            return 'speaking'
        if self._outranked(now):
            return 'outranked'
        # A turn just commanded may not have started moving yet: the head looks
        # still, but the array is about to swing and its next samples are junk.
        if (self.cur is None or abs(self.cur[0] - self.target[0]) > self.settle_tol
                or not self._yaw_still(now)):
            return 'moving'
        return None

    def _on_doa(self, msg: Doa) -> None:
        now = self._now()
        # The first reading after startup is whatever the chip was holding: leave
        # doa_changed_t at 0 so it counts as held, not freshly changed.
        if self.last_doa_angle is not None and msg.angle != self.last_doa_angle:
            self.doa_changed_t = now
        self.last_doa_angle = msg.angle
        why = self._doa_reject(msg, now)
        if why is not None:
            self.doa_buf.clear()
            if self.asr_speech_since is not None or now - self.asr_speech_ended < self.doa_tail:
                self.doa_why[why] += 1
                if not self.doa_seen or self.doa_seen[-1] != msg.angle:
                    self.doa_seen.append(msg.angle)
            return
        rel = _wrap(self.doa_sign * (math.radians(msg.angle) - self.doa_offset))
        self.doa_buf.append(rel)
        if len(self.doa_buf) < self.doa_min_samples:
            return
        buf = self.doa_buf[-self.doa_min_samples:]
        mean = math.atan2(sum(math.sin(a) for a in buf), sum(math.cos(a) for a in buf))
        if max(abs(_wrap(a - mean)) for a in buf) > self.doa_max_spread:
            self.doa_why['spread'] += 1
            return  # inconsistent: wait for a steadier run of samples
        self.doa_buf.clear()
        self.doa_why.clear()
        self.doa_seen.clear()
        self.last_attention = now
        if abs(mean) < self.doa_deadband:
            return  # already facing the speaker
        yaw = self._unwind(self.cur[0] + self.yaw_sign * mean)
        self.get_logger().info(
            f'DOA {msg.angle} deg ({math.degrees(mean):+.0f} deg) at yaw '
            f'{self.cur[0]:+.2f} -> yaw {yaw:+.2f}')
        self.speaker_yaw = _clamp(yaw, *self.yaw_lim)
        # Plus travel time: an unwind can take ~4 s, and faces swept past on the
        # way must not grab the head.
        travel = abs(yaw - self.cur[0]) / self.attend_speed
        self.speaker_until = now + self.speaker_lock + travel
        # Else the face-lost reset drops the raised pitch on the next tick.
        self.doa_look_until = now + self.face_lost + travel
        self.follow_until = 0.0  # stop following the old face; speaker lock picks up
        self._set_target(yaw, self.neutral_pitch + self.pitch_sign * self.doa_pitch_up, now)

    # ---- faces (NanoOWL) ----------------------------------------------------
    def _pose_at(self, t: float):
        """[yaw, pitch] at wall time t, interpolated from the pose history."""
        h = self.pose_hist
        if not h or t >= h[-1][0]:
            return list(self.cur)
        if t <= h[0][0]:
            return [h[0][1], h[0][2]]
        i = bisect.bisect_left(h, (t,))
        (t0, y0, p0), (t1, y1, p1) = h[i - 1], h[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return [y0 + a * (y1 - y0), p0 + a * (p1 - p0)]

    def _maybe_request_face(self, now: float) -> None:
        if self.face_backend == 'yunet':
            return  # runs from _on_face_img
        if self.face_pending_since is not None:
            if now - self.face_pending_since < 5.0:
                return
            self.face_pending_since = None  # lost response; try again
        if (now - self.last_face_call < self.face_period or self.generating
                or now < self.turn_until or not self.face_cli.service_is_ready()):
            return
        req = Detect.Request()
        req.prompt = self.face_prompt
        req.threshold = self.face_threshold
        self.last_face_call = now
        self.face_pending_since = now
        self.face_cli.call_async(req).add_done_callback(self._on_face)

    def _on_face(self, future) -> None:
        self.face_pending_since = None
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().warn(f'face detect failed: {e}', throttle_duration_sec=30.0)
            return
        if not res.ok:
            return
        dets = res.detections
        boxes = [(d.score, d.x0, d.y0, d.x1, d.y1)
                 for d in dets.detections if d.score >= self.face_threshold]
        self._apply_face(boxes, dets.header.stamp, dets.image_width, dets.image_height)

    def _detect_yunet(self, now: float) -> None:
        msg = self.face_img
        # 0.8: frames jitter around the camera period; don't skip every other one.
        if msg is None or now - self.last_face_call < 0.8 * self.face_period:
            return
        self.last_face_call = now
        self.face_img = None  # never run twice on the same frame
        img = self._cv2.imdecode(self._np.frombuffer(bytes(msg.data), self._np.uint8),
                                 self._cv2.IMREAD_COLOR)
        if img is None:
            return
        h, w = img.shape[:2]
        if (w, h) != self.yunet_size:
            self.yunet.setInputSize((w, h))
            self.yunet_size = (w, h)
        _, faces = self.yunet.detect(img)
        boxes = [] if faces is None else [
            (float(f[14]), f[0] / w, f[1] / h, (f[0] + f[2]) / w, (f[1] + f[3]) / h)
            for f in faces]
        self._apply_face(boxes, msg.header.stamp, w, h)

    def _apply_face(self, boxes, stamp_msg, width: int, height: int) -> None:
        """Center the largest (~nearest) of `boxes` = [(score, x0, y0, x1, y1)], normalized."""
        now = self._now()
        if not boxes or self._outranked(now) or self.cur is None:
            return
        stamp = stamp_msg.sec + stamp_msg.nanosec * 1e-9
        pose = self._pose_at(stamp - self.camera_latency) if stamp > 0.0 else list(self.cur)
        if width > 0:
            self.tan_half_v = self.tan_half_h * height / width

        def aim(b):  # head [yaw, pitch] that would center this box
            return (pose[0] - self.yaw_sign * math.atan(
                        2.0 * ((b[1] + b[3]) / 2.0 - 0.5) * self.tan_half_h),
                    pose[1] - self.pitch_sign * math.atan(
                        2.0 * ((b[2] + b[4]) / 2.0 - 0.5) * self.tan_half_v))

        def face_yaw(b):
            return aim(b)[0]

        # Only act on a face also seen (at about the same head-frame direction) in
        # the previous detection: YuNet throws single-frame false positives that
        # would otherwise yank the head and snap it back.
        # Too wide: a hand / body against the lens, or a ceiling fan. Too high
        # to look at: ceiling, not a person.
        up_lim = self.pitch_lim[0] if self.pitch_sign < 0 else self.pitch_lim[1]
        boxes = [b for b in boxes if b[3] - b[1] < self.face_max_width
                 and (aim(b)[1] - up_lim) * self.pitch_sign <= self.face_max_above]
        aims = [aim(b) for b in boxes]
        prev_t, prev = self.face_prev
        self.face_prev = (now, aims)
        if now - prev_t < 2.5 * self.face_period:
            boxes = [b for b, a in zip(boxes, aims)
                     if any(math.hypot(a[0] - p[0], a[1] - p[1]) < self.face_confirm
                            for p in prev)]
        else:
            boxes = []
        if not boxes:
            return

        if self.asr_speech_since is not None and now < self.speaker_until:
            self.speaker_until = now + self.speaker_lock  # they're still talking
        if self.speaker_yaw is not None and now < self.speaker_until:
            near = [b for b in boxes if abs(face_yaw(b) - self.speaker_yaw) < self.speaker_match]
            if not near:
                return  # keep turning toward the speaker; ignore other faces
            best = min(near, key=lambda b: abs(face_yaw(b) - self.speaker_yaw))
            self.speaker_yaw = face_yaw(best)  # follow them as they move
        elif now - self.last_face < 1.5:
            # Sticky: keep the face we're on (closest to where the head is aimed);
            # switching people is DOA's job, else two people in view ping-pong.
            best = min(boxes, key=lambda b: math.hypot(
                face_yaw(b) - self.target[0], aim(b)[1] - self.target[1]))
        else:
            # New face: only one turned toward us. YuNet catches profiles at
            # 0.6-0.8, intermittently and with a jumpy box, so a person side-on
            # at a desk had the head glancing about; frontal faces score ~0.95+.
            if self.face_backend == 'yunet':
                boxes = [b for b in boxes if b[0] >= self.face_acquire_score]
                if not boxes:
                    return
            best = max(boxes, key=lambda b: (b[3] - b[1]) * (b[4] - b[2]))  # ~nearest
        score, x0, y0, x1, y1 = best
        dx = (x0 + x1) / 2.0 - 0.5  # + = right of center
        dy = (y0 + y1) / 2.0 - 0.5  # + = below center
        self.last_face = now
        self.last_attention = now
        z = aim(best)
        tm = stamp - self.camera_latency if stamp > 0.0 else now  # capture time
        if now < self.follow_until:
            # Following: alpha-beta filter on the face direction. _tick_attend
            # moves the target continuously along it (position + velocity), so
            # the head glides with a walking person instead of stepping to each
            # 10 Hz fix and stopping (which kicks the yaw servo back).
            dt = max(0.02, tm - self.fa_t)
            for i in (YAW, PITCH):
                pred = self.fa[i] + self.fv[i] * dt
                r = z[i] - pred
                self.fa[i] = pred + self.face_alpha * r
                self.fv[i] = _clamp(self.fv[i] + self.face_beta * r / dt,
                                    -self.attend_speed, self.attend_speed)
            self.fa_t = tm
            if (max(abs(v) for v in self.fv) > self.face_vel_min
                    or abs(self.fa[YAW] - self.target[0]) > self.face_retarget
                    or abs(self.fa[PITCH] - self.target[1]) > self.face_retarget):
                self.follow_until = now + self.face_track_hold
        elif abs(dy) > self.face_deadband or (self.face_refine_yaw
                                              and abs(dx) > self.face_deadband):
            self.fa, self.fv, self.fa_t = list(z), [0.0, 0.0], tm
            self.follow_until = now + self.face_track_hold
            self.get_logger().debug(
                f'face {score:.2f} at ({dx:+.2f}, {dy:+.2f}) -> '
                f'yaw {z[0]:+.2f} pitch {z[1]:+.2f}')
            self._follow_target(now)

    def _follow_target(self, now: float) -> None:
        """Aim at the followed face, extrapolated to now (bounded if fixes stop)."""
        # Lead by the smoothing filter's lag too, so following doesn't trail.
        h = _clamp(now - self.fa_t, 0.0, self.face_predict) + self.smooth_lag
        yaw = self.fa[YAW] + self.fv[YAW] * h if self.face_refine_yaw else self.target[0]
        self._set_target(yaw, self.fa[PITCH] + self.fv[PITCH] * h, now)

    # ---- main loop ----------------------------------------------------------
    def _tick(self) -> None:
        t = self._elapsed()

        if self.eyes_enabled:
            self._publish_eyes()

        if not self.head_enabled:
            return
        if self.head_mode == 'attend':
            self._tick_attend()
            return

        yaw = self.head_yaw_amp * math.sin(2.0 * math.pi * t / self.head_period)
        pitch = self.head_pitch_amp * math.sin(
            2.0 * math.pi * t / (self.head_period * 0.7) + 1.0)
        self._publish_head(self._servo(yaw, self.head_speed),
                           self._servo(pitch, self.head_speed),
                           self._servo(0.0, self.head_speed))

    def _tick_attend(self) -> None:
        if self.target is None:
            return  # no head state yet; don't command a pose blind
        now = self._now()
        if self._outranked(now):
            # Someone else owns the head: track its pose so we resume from there
            # instead of snapping back to our old target.
            self.target = list(self.cur)
            self.target_changed = now
            self.sp = [self.cur[0], self.cur[1], self.cur_roll]
            self.sv = [0.0, 0.0, 0.0]
            self.sm, self.smv = list(self.cur), [0.0, 0.0]
            return
        if self.sp is None:
            self.sp = [self.cur[0], self.cur[1], self.cur_roll]
            self.sm, self.smv = list(self.cur), [0.0, 0.0]
            self.sp_t = now
        speaking = now < self.speaking_until
        if speaking:
            # Hold the face/attention timers (and skip face polling: mid-nod frames
            # would jitter the pitch) until Alfie finishes talking.
            self.last_face = max(self.last_face, now - self.face_lost + 1.0)
            self.last_attention = now
        elif self.face_enabled:
            self._maybe_request_face(now)
            if now < self.follow_until:
                self._follow_target(now)
        if now - self.last_face > self.face_lost and now >= self.doa_look_until:
            self._set_target(self.target[0], self.neutral_pitch, now)
        if now - self.last_attention > self.attend_timeout:
            self._set_target(0.0, self.neutral_pitch, now)
        if (self.doa_why and self.asr_speech_since is None
                and now - self.asr_speech_ended > self.doa_tail):
            self.get_logger().info(
                f'DOA: no turn; rejected {dict(self.doa_why)}, readings {self.doa_seen}')
            self.doa_why.clear()
            self.doa_seen.clear()
        listening = now >= self.speaking_until and (
            self.asr_speech_since is not None
            or now - self.asr_speech_ended < self.listen_roll_hold)
        if listening != self.listening:
            self.listening = listening
            self.get_logger().info('listening: tilt' if listening else 'listening: level')
        roll = self.listen_roll if listening else 0.0
        dt = _clamp(now - self.sp_t, 0.0, 0.1)
        self.sp_t = now
        yaw = self._smooth(YAW, self._profile(
            YAW, self.target[0], self.attend_speed, self.attend_accel, dt), dt)
        pitch = self._smooth(PITCH, self._profile(
            PITCH, self.target[1], self.attend_speed, self.attend_accel, dt), dt)
        roll = self._profile(ROLL, roll, self.roll_speed, self.roll_accel, dt)
        if self.nod_enabled and speaking:
            pitch = _clamp(pitch - self.pitch_sign * self._nod_offset(now), *self.pitch_lim)
        # Constant servo speed above the ramp's: the setpoint sets the pace, and
        # rewriting the speed register every tick is one more thing to perturb it.
        self._publish_head(self._servo(yaw, 2.0 * self.attend_speed),
                           self._servo(pitch, 2.0 * self.attend_speed),
                           self._servo(roll, 2.0 * self.roll_speed))

    def _smooth(self, i: int, u: float, dt: float) -> float:
        """Critically damped 2nd-order low-pass on the yaw/pitch setpoint.

        The head + servo position loop rings at ~5-6 Hz; the small kinks a
        10 Hz face fix puts in the setpoint excite it (measured 2026-10-19:
        actual jitter ~2.5x the command's). Rolling off above `smooth_hz`
        keeps that out, for ~1/(pi*smooth_hz) s of lag (led away in follow).
        """
        if self.smooth_w <= 0.0:
            return u
        w = self.smooth_w
        n = max(1, math.ceil(dt / 0.02))  # keep w*h small: explicit steps go unstable
        h = dt / n
        for _ in range(n):
            self.smv[i] += (w * w * (u - self.sm[i]) - 2.0 * w * self.smv[i]) * h
            self.sm[i] += self.smv[i] * h
        return self.sm[i]

    def _profile(self, i: int, goal: float, vmax: float, amax: float, dt: float) -> float:
        """Advance setpoint i one tick along a trapezoidal profile toward `goal`.

        The STS servo's own accel profile restarts from rest whenever the goal
        changes mid-move, so a face-followed target (new goal every frame) made
        the head stop and twitch back each time. Instead the ramp runs here and
        the servo gets a smoothly moving goal with its accel off (as VR does).
        """
        p, v = self.sp[i], self.sv[i]
        err = goal - p
        if amax <= 0.0:
            v = _clamp(err / dt, -vmax, vmax) if dt > 0.0 else 0.0
        else:
            vd = math.copysign(min(vmax, math.sqrt(2.0 * amax * abs(err))), err)
            v += _clamp(vd - v, -amax * dt, amax * dt)
        p += v * dt
        if (goal - p) * err <= 0.0:  # reached or crossed the goal
            p, v = goal, 0.0
        self.sp[i], self.sv[i] = p, v
        return p

    def _eye_ack(self) -> None:
        now = self._now()
        if now < self.eye_ack_until:
            return  # one blink per event burst (wake + barge_in arrive together)
        self.eye_ack_until = now + self.eye_ack_sec
        self._publish_eyes()  # off now, not on the next tick
        if self.eye_ack_timer is not None:
            self.eye_ack_timer.cancel()
            self.destroy_timer(self.eye_ack_timer)
        self.eye_ack_timer = self.create_timer(self.eye_ack_sec, self._eye_ack_done)

    def _eye_ack_done(self) -> None:
        self.eye_ack_timer.cancel()
        self.destroy_timer(self.eye_ack_timer)
        self.eye_ack_timer = None
        self.eye_ack_until = 0.0
        self._publish_eyes()  # back on exactly at the end of the blink

    def _publish_eyes(self) -> None:
        pwm = 0 if self._now() < self.eye_ack_until else self.eye_idle_pwm
        eyes = EyeCmd()
        eyes.eye_pwm = [pwm, pwm]
        self.eyes_pub.publish(eyes)

    def _publish_head(self, yaw: ServoCmd, pitch: ServoCmd, roll: ServoCmd) -> None:
        head = HeadCmd()
        head.servos = [yaw, pitch, roll]  # 0 pan/yaw, 1 tilt/pitch, 2 roll
        head.eye_pwm = [0, 0]  # eyes travel on the eyes channel
        self.head_pub.publish(head)

    def _servo(self, location: float, speed: float, accel: float = 0.0) -> ServoCmd:
        s = ServoCmd()
        s.enabled = True
        s.target_location = float(location)
        s.target_speed = speed
        s.target_acceleration = accel
        s.target_torque = 0.0
        return s


def main(args=None):
    rclpy.init(args=args)
    node = IdleBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
