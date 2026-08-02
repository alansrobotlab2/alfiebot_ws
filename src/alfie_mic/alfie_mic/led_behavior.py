"""
led_behavior node — maps robot conversation state to the reSpeaker LED ring.

Keeps respeaker_control a pure USB driver: this node only decides *what* the
ring should show and publishes LedCommand; respeaker_control owns the USB writes.

A wake/stop detection briefly flashes the whole ring (green for a wake word,
red for "stop"), overlaid on top of the current state.

State priority (highest first):
  ESTOP     (command_mux latched) -> solid red, overrides everything
  SPEAKING  (tts is talking)      -> cyan amplitude pulse
  THINKING  (llm is generating)   -> animated cyan<->green crossfade breath
  LISTENING (resting/default)     -> DOA follow (ring points at the speaker)
  IDLE      (no activity for a while) -> dim solid

THINKING is a host-driven animation: the ring can only show one colour at a
time (no per-pixel control on the XVF3800), so we stream LED_COLOR at FRAME_HZ to
crossfade the whole ring cyan<->green. respeaker_control only issues a USB write
for the field that changed, so this costs ~one transfer per frame.

Inputs: `speaking` (Speaking) from TTS, `generating` (Bool) from the agent.
Output: `respeaker/led_command` (LedCommand) to respeaker_control. Static states
publish on change (plus a periodic refresh); THINKING publishes every frame.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from std_msgs.msg import Bool, Float32, String
from alfie_msgs.msg import LedCommand, Speaking

# LedCommand.effect values (see LedCommand.msg / xvf3800 LED_EFFECT).
EFFECT_OFF, EFFECT_BREATH, EFFECT_RAINBOW, EFFECT_SOLID, EFFECT_DOA, EFFECT_RING = range(6)

# ======================= LED appearance — tweak these =======================
# Colours are (R, G, B), 0-255. Brightness for the animated states caps at
# LED_MAX so the ring isn't harsh. Effect modes are the firmware values above.

FRAME_HZ = 30.0         # animation / update rate
IDLE_TIMEOUT_S = 30.0   # drop to IDLE after this long with no speaking/thinking
REFRESH_S = 5.0         # re-publish a static state periodically (driver re-sync)
LED_MAX = 191           # peak channel value for animated states (75% of 255)

# --- Wake / barge-in flash: a brief full-ring flash over the current state ---
FLASH_DURATION_S = 0.45           # how long the flash holds
WAKE_FLASH_COLOR = (0, 255, 0)    # green: a wake word was heard (hey alfie / alfie)
BARGE_FLASH_COLOR = (255, 0, 0)   # red: a cancel / barge-in (stop, energy barge-in)
CANCEL_KEYS = {'stop', 'bargein'}  # detection keys that use BARGE_FLASH_COLOR

# --- THINKING: whole ring crossfades between two colours (green reserved for wake) ---
THINK_PERIOD_S = 1.0              # seconds per full A -> B -> A cycle
THINK_COLOR_A = (0, 255, 255)     # cyan
THINK_COLOR_B = (128, 0, 255)     # purple
THINK_BRIGHTNESS = LED_MAX        # peak brightness of the crossfade (0-255)

# --- SPEAKING: ring holds a colour whose brightness tracks the live TTS amplitude ---
SPK_COLOR = (0, 255, 255)         # cyan base (brightness follows the voice envelope)
SPK_MAX_BRIGHT = LED_MAX          # brightest, on the loudest syllable
SPK_MIN_BRIGHT = 25               # floor so the ring never blacks out between words
SPK_GAIN = 4.0                    # RMS -> 0..1 (speech RMS is small)
SPK_RELEASE = 0.08                # envelope decay per frame (~0.4 s fall at 30 Hz)
SPK_LEVEL_TIMEOUT_S = 0.3         # treat level as 0 if no update for this long

# --- ESTOP: solid red while the command_mux is latched, above every other state ---
ESTOP_COLOR = (255, 0, 0)
ESTOP_BRIGHTNESS = 255
# ============================================================================


def _pack(rgb):
    """(R, G, B) 0-255 -> 0x00RRGGBB int for LedCommand.color."""
    r, g, b = rgb
    return (int(r) << 16) | (int(g) << 8) | int(b)

# Static looks: (effect, brightness, speed, color 0x00RRGGBB), published on change.
STATE_LED = {
    'ESTOP':     (EFFECT_SOLID, ESTOP_BRIGHTNESS, 0, _pack(ESTOP_COLOR)),
    'LISTENING': (EFFECT_DOA,   40, 0, 0x0000FF),   # ring follows the speaker
    'IDLE':      (EFFECT_SOLID,  10, 0, 0x101010),  # dim resting glow
}


class LedBehavior(Node):
    def __init__(self):
        super().__init__('led_behavior')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # respeaker_control subscribes RELIABLE, so the LED publisher must be
        # RELIABLE too (a BEST_EFFORT pub -> RELIABLE sub is QoS-incompatible and
        # delivers nothing). The speaking/generating subs stay BEST_EFFORT to
        # match the TTS/agent publishers.
        led_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.led_pub = self.create_publisher(LedCommand, 'respeaker/led_command', led_qos)
        self.create_subscription(Speaking, 'speaking', self.on_speaking, qos)
        self.create_subscription(Bool, 'generating', self.on_generating, qos)
        self.create_subscription(Float32, 'tts/level', self.on_level, qos)
        # wakeword_node publishes RELIABLE, so match it for the detection flash.
        self.create_subscription(String, 'wakeword/detection', self.on_detection, led_qos)
        # command_mux publishes estop_state latched, so a node started while the
        # robot is already stopped still picks up the red ring.
        estop_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Bool, 'estop_state', self.on_estop_state, estop_qos)

        self.estopped = False
        self.speaking = False
        self.generating = False
        self._last_state = None
        self._last_publish = None
        self._anim_t0 = time.monotonic()
        self._last_activity = self.get_clock().now()
        # Speaking-pulse envelope follower state.
        self._spk_raw = 0.0
        self._spk_ts = 0.0
        self._spk_env = 0.0
        # Wake/stop flash overlay state.
        self._flash_until = 0.0
        self._flash_color = WAKE_FLASH_COLOR
        self._flashing = False

        self.create_timer(1.0 / FRAME_HZ, self._tick)
        self.get_logger().info('LedBehavior initialized.')

    def on_speaking(self, msg):
        self.speaking = msg.is_speaking
        if msg.is_speaking:
            self._mark_active()

    def on_generating(self, msg):
        self.generating = msg.data
        if msg.data:
            self._mark_active()

    def on_level(self, msg):
        self._spk_raw = msg.data
        self._spk_ts = time.monotonic()

    def on_detection(self, msg):
        # Flash green on a wake word, red on a cancel / barge-in.
        key = (msg.data or '').strip()
        self._flash_color = BARGE_FLASH_COLOR if key in CANCEL_KEYS else WAKE_FLASH_COLOR
        self._flash_until = time.monotonic() + FLASH_DURATION_S
        self._mark_active()

    def on_estop_state(self, msg):
        if msg.data != self.estopped:
            self.get_logger().warn(
                f'e-stop {"engaged" if msg.data else "cleared"} — ring '
                f'{"-> red" if msg.data else "released"}')
        self.estopped = msg.data

    def _mark_active(self):
        self._last_activity = self.get_clock().now()

    def _desired_state(self):
        if self.estopped:
            return 'ESTOP'
        if self.speaking:
            return 'SPEAKING'
        if self.generating:
            return 'THINKING'
        idle_for = (self.get_clock().now() - self._last_activity).nanoseconds / 1e9
        if idle_for > IDLE_TIMEOUT_S:
            return 'IDLE'
        return 'LISTENING'

    def _tick(self):
        # Wake/stop flash overrides everything for its brief duration -- except
        # an e-stop, which must never be masked by a wake word landing while the
        # robot is latched.
        if not self.estopped and time.monotonic() < self._flash_until:
            self._publish(EFFECT_SOLID, 255, 0, _pack(self._flash_color))
            self._flashing = True
            return
        if self._flashing:
            self._flashing = False
            self._last_state = None   # force the underlying state to re-publish

        state = self._desired_state()
        changed = state != self._last_state
        if changed:
            self.get_logger().info(f'LED state -> {state}')
            self._last_state = state
            self._anim_t0 = time.monotonic()

        if state == 'THINKING':
            self._animate_thinking()
        elif state == 'SPEAKING':
            self._animate_speaking()
        else:
            # Static state: publish on change or periodic refresh only.
            stale = (self._last_publish is None or
                     (time.monotonic() - self._last_publish) >= REFRESH_S)
            if changed or stale:
                effect, brightness, speed, color = STATE_LED[state]
                self._publish(effect, brightness, speed, color)

    def _animate_thinking(self):
        # Crossfade A <-> B, scaled to THINK_BRIGHTNESS.
        t = time.monotonic() - self._anim_t0
        p = (1 - math.cos(2 * math.pi * t / THINK_PERIOD_S)) / 2  # 0=A .. 1=B .. 0
        s = THINK_BRIGHTNESS / 255.0
        rgb = tuple(int((a + (b - a) * p) * s)
                    for a, b in zip(THINK_COLOR_A, THINK_COLOR_B))
        self._publish(EFFECT_SOLID, 255, 0, _pack(rgb))

    def _animate_speaking(self):
        # Envelope follower over the live TTS level: instant attack, slow release.
        now = time.monotonic()
        target = self._spk_raw if (now - self._spk_ts) < SPK_LEVEL_TIMEOUT_S else 0.0
        scaled = min(1.0, target * SPK_GAIN)
        if scaled > self._spk_env:
            self._spk_env = scaled
        else:
            self._spk_env = max(scaled, self._spk_env - SPK_RELEASE)
        # Scale SPK_COLOR so its brightest channel rides SPK_MIN..SPK_MAX with the voice.
        bright = SPK_MIN_BRIGHT + (SPK_MAX_BRIGHT - SPK_MIN_BRIGHT) * self._spk_env
        peak = max(SPK_COLOR) or 1
        s = bright / peak
        rgb = tuple(int(c * s) for c in SPK_COLOR)
        self._publish(EFFECT_SOLID, 255, 0, _pack(rgb))

    def _publish(self, effect, brightness, speed, color):
        msg = LedCommand()
        msg.effect = effect
        msg.brightness = brightness
        msg.speed = speed
        msg.color = color
        self.led_pub.publish(msg)
        self._last_publish = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = LedBehavior()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
