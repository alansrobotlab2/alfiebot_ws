"""
idle_behavior — low-priority "life" behaviors for Alfie.

Publishes gentle idle motion to the command_mux at the lowest motion priority
(`cmd/eyes/idle`, `cmd/head/idle`). Any real commander (VR, GR00T, agent gaze)
outranks it, so this only shows through when nothing else owns the head/eyes.

    cmd/eyes/idle  (EyeCmd)  — constant dim glow
    cmd/head/idle  (HeadCmd) — head behavior, chosen by `head_mode`:

      wander  — small open-loop sinusoidal look-around.
      attend  — look at whoever is talking / nearby:
                * yaw turns toward the reSpeaker direction-of-arrival while the
                  ASR's VAD (`vad/speech`) reports speech — the XVF3800's own
                  speech flag fires on room noise — ignored while Alfie speaks.
                * pitch (and a yaw refinement) centers the nearest face, found by
                  polling NanoOWL's on-demand `nanoowl/detect` with "a face" at
                  `face_rate_hz` and taking the largest box.
                The reSpeaker sits on head_link, so DOA is head-relative: the yaw
                target is current yaw + DOA offset, sampled only while the head is
                settled. The XVF3800 holds its last DOA estimate until new speech
                re-converges it, so after each turn the pre-turn reading is
                ignored until it changes (else it gets added a second time).
                While someone is talking the head rolls a little to the left
                (a listening tilt), then levels out. Face polling
                pauses while a higher-priority head source is active or the
                LLM is generating, so it doesn't steal GPU.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool
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
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('eyes_enabled', True)
        self.declare_parameter('eye_idle_pwm', 1)
        self.declare_parameter('head_enabled', True)
        self.declare_parameter('head_mode', 'attend')  # 'attend' | 'wander'
        # wander
        self.declare_parameter('head_yaw_amp_rad', 0.15)
        self.declare_parameter('head_pitch_amp_rad', 0.05)
        self.declare_parameter('head_period_sec', 12.0)
        self.declare_parameter('head_speed', 0.3)
        # attend: joint conventions + limits
        self.declare_parameter('attend_speed', 1.5)          # rad/s servo speed
        # Servo-side trapezoidal ramp (STS accel register, ~0.15 rad/s^2 steps);
        # 0 = instant, which looks mechanical. Applies to accel and decel.
        self.declare_parameter('attend_accel', 4.0)          # rad/s^2
        self.declare_parameter('yaw_limits', [-3.14, 3.14])  # rad; firmware enforces the real limit
        # Servo frame (+ = down). Up is left wide open: the head firmware enforces
        # the real mechanical limit, and _settled() copes with a target past it.
        self.declare_parameter('pitch_limits', [-1.6, 0.4])  # rad
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
        self.declare_parameter('speech_gate', 'asr')     # 'asr' (vad/speech) | 'array' (XVF VAD)
        self.declare_parameter('doa_onset_delay_sec', 0.4)  # DOA lags a new speaker
        self.declare_parameter('doa_min_samples', 3)     # consecutive speech samples (10 Hz)
        self.declare_parameter('doa_max_spread_deg', 25.0)
        self.declare_parameter('doa_deadband_deg', 15.0)
        self.declare_parameter('speaking_holdoff_sec', 0.7)  # ignore DOA after own TTS
        # attend: faces (NanoOWL)
        self.declare_parameter('face_enabled', True)
        self.declare_parameter('face_service', 'nanoowl/detect')
        self.declare_parameter('face_prompt', 'a face')
        self.declare_parameter('face_threshold', 0.15)
        self.declare_parameter('face_rate_hz', 1.0)
        self.declare_parameter('face_refine_yaw', True)
        self.declare_parameter('face_deadband', 0.06)    # normalized image units
        self.declare_parameter('camera_hfov_deg', 90.0)  # left_wide; conservative = undershoot
        self.declare_parameter('face_lost_sec', 8.0)     # then pitch returns to neutral
        self.declare_parameter('attend_timeout_sec', 45.0)  # then yaw returns to center
        # attend: listening tilt (roll) while vad/speech is on
        self.declare_parameter('listen_roll_rad', 0.15)    # servo frame; sign = "to the left"
        self.declare_parameter('listen_roll_hold_sec', 1.0)  # keep the tilt after speech ends
        self.declare_parameter('roll_speed', 0.5)          # rad/s, gentler than yaw/pitch
        self.declare_parameter('roll_accel', 1.5)          # rad/s^2
        self.declare_parameter('higher_head_sources', ['tool', 'vr', 'groot', 'agent'])
        self.declare_parameter('higher_source_timeout', 0.5)

        g = self.get_parameter
        self.rate = float(g('rate_hz').value)
        self.eyes_enabled = bool(g('eyes_enabled').value)
        self.eye_idle_pwm = max(0, min(4095, int(g('eye_idle_pwm').value)))
        self.head_enabled = bool(g('head_enabled').value)
        self.head_mode = str(g('head_mode').value)
        self.head_yaw_amp = float(g('head_yaw_amp_rad').value)
        self.head_pitch_amp = float(g('head_pitch_amp_rad').value)
        self.head_period = float(g('head_period_sec').value)
        self.head_speed = float(g('head_speed').value)

        self.attend_speed = float(g('attend_speed').value)
        self.attend_accel = float(g('attend_accel').value)
        self.yaw_lim = [float(v) for v in g('yaw_limits').value]
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
        self.speaking_holdoff = float(g('speaking_holdoff_sec').value)
        self.doa_fresh = math.radians(float(g('doa_fresh_deg').value))
        self.speech_gate = str(g('speech_gate').value)
        self.doa_onset_delay = float(g('doa_onset_delay_sec').value)
        self.face_enabled = bool(g('face_enabled').value)
        self.face_prompt = str(g('face_prompt').value)
        self.face_threshold = float(g('face_threshold').value)
        self.face_period = 1.0 / max(0.05, float(g('face_rate_hz').value))
        self.face_refine_yaw = bool(g('face_refine_yaw').value)
        self.face_deadband = float(g('face_deadband').value)
        hfov = math.radians(float(g('camera_hfov_deg').value))
        self.tan_half_h = math.tan(hfov / 2.0)
        self.tan_half_v = self.tan_half_h * 256.0 / 352.0  # refined from detection dims
        self.face_lost = float(g('face_lost_sec').value)
        self.attend_timeout = float(g('attend_timeout_sec').value)
        self.listen_roll = float(g('listen_roll_rad').value)
        self.listen_roll_hold = float(g('listen_roll_hold_sec').value)
        self.roll_speed = float(g('roll_speed').value)
        self.roll_accel = float(g('roll_accel').value)
        self.higher_timeout = float(g('higher_source_timeout').value)

        # ---- Publishers -----------------------------------------------------
        self.eyes_pub = self.create_publisher(EyeCmd, 'cmd/eyes/idle', qos)
        self.head_pub = self.create_publisher(HeadCmd, 'cmd/head/idle', qos)

        # ---- Attend state ---------------------------------------------------
        self.cur = None             # [yaw, pitch] from low/headstate
        self.still_ref = None       # pose the head was last seen still at
        self.last_motion = 0.0      # last time the head moved > 0.01 rad
        self.target = None          # [yaw, pitch] we command
        self.target_changed = 0.0   # time the target last moved
        self.last_attention = 0.0   # last accepted DOA or face fix
        self.last_face = 0.0
        self.doa_buf = []           # recent head-relative DOA (rad, CCW+)
        self.last_doa_angle = None  # latest raw DOA reading (deg)
        self.doa_stale_angle = None  # raw reading held from before the last turn
        self.speaking_until = 0.0
        self.asr_speech_since = None  # vad/speech onset time while speech is on
        self.asr_speech_ended = 0.0   # last vad/speech falling edge
        self.listening = False
        self.generating = False
        self.higher_seen = 0.0
        self.face_pending_since = None
        self.last_face_call = 0.0

        if self.head_enabled and self.head_mode == 'attend':
            self.create_subscription(HeadState, 'low/headstate', self._on_head_state, qos)
            self.create_subscription(Doa, 'respeaker/doa', self._on_doa, qos)
            self.create_subscription(Speaking, 'speaking', self._on_speaking, qos)
            self.create_subscription(Bool, 'vad/speech', self._on_vad, qos)
            self.create_subscription(Bool, 'generating', self._on_generating, qos)
            for src in g('higher_head_sources').value:
                self.create_subscription(
                    HeadCmd, f'cmd/head/{src}', self._on_higher_source, qos)
            self.face_cli = self.create_client(Detect, str(g('face_service').value))

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
        now = self._now()
        if self.target is None:
            self.target = list(self.cur)  # start from wherever the head is
        if (self.still_ref is None or abs(self.cur[0] - self.still_ref[0]) > 0.01
                or abs(self.cur[1] - self.still_ref[1]) > 0.01):
            self.still_ref = list(self.cur)
            self.last_motion = now

    def _on_speaking(self, msg: Speaking) -> None:
        if msg.is_speaking:
            self.speaking_until = float('inf')
        elif self.speaking_until == float('inf'):
            self.speaking_until = self._now() + self.speaking_holdoff

    def _on_vad(self, msg: Bool) -> None:
        if not msg.data:
            if self.asr_speech_since is not None:
                self.asr_speech_ended = self._now()
            self.asr_speech_since = None
        elif self.asr_speech_since is None:
            self.asr_speech_since = self._now()

    def _speech(self, msg: Doa, now: float) -> bool:
        if self.speech_gate == 'array':
            return msg.speech_detected
        return (self.asr_speech_since is not None
                and now - self.asr_speech_since > self.doa_onset_delay)

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
                # The array turns with the head; its held estimate is now wrong.
                self.doa_stale_angle = self.last_doa_angle
            self.target = [yaw, pitch]
            self.target_changed = now
            self.doa_buf.clear()  # samples taken mid-turn are stale

    def _on_doa(self, msg: Doa) -> None:
        now = self._now()
        self.last_doa_angle = msg.angle
        if self.doa_stale_angle is not None:
            if abs(_wrap(math.radians(msg.angle - self.doa_stale_angle))) <= self.doa_fresh:
                self.doa_buf.clear()
                return  # still the estimate from before the turn
            self.doa_stale_angle = None
        if (not self._speech(msg, now) or now < self.speaking_until
                or self._outranked(now) or not self._settled(now)):
            self.doa_buf.clear()
            return
        rel = _wrap(self.doa_sign * (math.radians(msg.angle) - self.doa_offset))
        self.doa_buf.append(rel)
        if len(self.doa_buf) < self.doa_min_samples:
            return
        buf = self.doa_buf[-self.doa_min_samples:]
        mean = math.atan2(sum(math.sin(a) for a in buf), sum(math.cos(a) for a in buf))
        if max(abs(_wrap(a - mean)) for a in buf) > self.doa_max_spread:
            return  # inconsistent: wait for a steadier run of samples
        self.doa_buf.clear()
        self.last_attention = now
        if abs(mean) < self.doa_deadband:
            return  # already facing the speaker
        yaw = self.cur[0] + self.yaw_sign * mean
        self.get_logger().info(
            f'DOA {msg.angle} deg ({math.degrees(mean):+.0f} deg) at yaw '
            f'{self.cur[0]:+.2f} -> yaw {yaw:+.2f}')
        self._set_target(yaw, self.target[1], now)

    # ---- faces (NanoOWL) ----------------------------------------------------
    def _maybe_request_face(self, now: float) -> None:
        if self.face_pending_since is not None:
            if now - self.face_pending_since < 5.0:
                return
            self.face_pending_since = None  # lost response; try again
        if (now - self.last_face_call < self.face_period or self.generating
                or not self._settled(now) or not self.face_cli.service_is_ready()):
            return
        req = Detect.Request()
        req.prompt = self.face_prompt
        req.threshold = self.face_threshold
        self.last_face_call = now
        self.face_pending_since = now
        self.face_cli.call_async(req).add_done_callback(self._on_face)

    def _on_face(self, future) -> None:
        self.face_pending_since = None
        now = self._now()
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().warn(f'face detect failed: {e}', throttle_duration_sec=30.0)
            return
        if not res.ok or self._outranked(now) or self.cur is None:
            return
        dets = res.detections
        faces = [d for d in dets.detections if d.score >= self.face_threshold]
        if not faces:
            return
        # Largest box ~ nearest face.
        f = max(faces, key=lambda d: (d.x1 - d.x0) * (d.y1 - d.y0))
        if dets.image_width > 0:
            self.tan_half_v = self.tan_half_h * dets.image_height / dets.image_width
        dx = (f.x0 + f.x1) / 2.0 - 0.5  # + = right of center
        dy = (f.y0 + f.y1) / 2.0 - 0.5  # + = below center
        self.last_face = now
        self.last_attention = now
        yaw, pitch = self.target
        if abs(dy) > self.face_deadband:
            pitch = self.cur[1] - self.pitch_sign * math.atan(2.0 * dy * self.tan_half_v)
        if self.face_refine_yaw and abs(dx) > self.face_deadband:
            yaw = self.cur[0] - self.yaw_sign * math.atan(2.0 * dx * self.tan_half_h)
        if [yaw, pitch] != self.target:
            self.get_logger().info(
                f'face {f.score:.2f} at ({dx:+.2f}, {dy:+.2f}) -> '
                f'yaw {yaw:+.2f} pitch {pitch:+.2f}')
        self._set_target(yaw, pitch, now)

    # ---- main loop ----------------------------------------------------------
    def _tick(self) -> None:
        t = self._elapsed()

        if self.eyes_enabled:
            eyes = EyeCmd()
            eyes.eye_pwm = [self.eye_idle_pwm, self.eye_idle_pwm]
            self.eyes_pub.publish(eyes)

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
            return
        if self.face_enabled:
            self._maybe_request_face(now)
        if now - self.last_face > self.face_lost:
            self._set_target(self.target[0], self.neutral_pitch, now)
        if now - self.last_attention > self.attend_timeout:
            self._set_target(0.0, self.neutral_pitch, now)
        listening = now >= self.speaking_until and (
            self.asr_speech_since is not None
            or now - self.asr_speech_ended < self.listen_roll_hold)
        if listening != self.listening:
            self.listening = listening
            self.get_logger().info('listening: tilt' if listening else 'listening: level')
        roll = self.listen_roll if listening else 0.0
        self._publish_head(
            self._servo(self.target[0], self.attend_speed, self.attend_accel),
            self._servo(self.target[1], self.attend_speed, self.attend_accel),
            self._servo(roll, self.roll_speed, self.roll_accel))

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
