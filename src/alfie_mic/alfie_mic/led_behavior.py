"""
led_behavior node — maps robot conversation state to the reSpeaker LED ring.

Keeps respeaker_control a pure USB driver: this node only decides *what* the
ring should show and publishes LedCommand; respeaker_control owns the USB writes.

State priority (highest first):
  SPEAKING  (tts is talking)      -> solid green (amplitude pulse TBD)
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
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float32
from alfie_msgs.msg import LedCommand, Speaking

# LedCommand.effect values (see LedCommand.msg / xvf3800 LED_EFFECT).
EFFECT_OFF, EFFECT_BREATH, EFFECT_RAINBOW, EFFECT_SOLID, EFFECT_DOA, EFFECT_RING = range(6)

FRAME_HZ = 30.0         # animation / update rate
IDLE_TIMEOUT_S = 30.0   # drop to IDLE after this long with no speaking/thinking
REFRESH_S = 5.0         # re-publish a static state periodically (driver re-sync)

# Peak channel value for the animated states (75% of full 255).
LED_MAX = 191

# THINKING crossfade: whole ring morphs cyan <-> green. The two colours differ
# only in the blue channel, so the fade is just blue 0..LED_MAX with G=LED_MAX —
# no muddy midpoints.
THINK_PERIOD_S = 1.0    # seconds per full cyan->green->cyan cycle

# SPEAKING pulse: cyan ring whose brightness tracks the live TTS amplitude
# (tts/level, RMS 0..1). Envelope follower: instant attack, slow release, so it
# punches with the voice without strobing. Brightness is applied via LED_COLOR
# magnitude because the firmware BRIGHTNESS param only affects breath/rainbow.
SPK_GAIN = 4.0          # RMS -> 0..1 (speech RMS is small)
SPK_MIN_BRIGHT = 25     # floor so the ring never fully blacks out between words
SPK_RELEASE = 0.08      # env decay per frame (~0.4 s fall at 30 Hz)
SPK_LEVEL_TIMEOUT_S = 0.3   # treat level as 0 if no update for this long

# Static looks: (effect, brightness, speed, color 0x00RRGGBB), published on change.
STATE_LED = {
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

    def _mark_active(self):
        self._last_activity = self.get_clock().now()

    def _desired_state(self):
        if self.speaking:
            return 'SPEAKING'
        if self.generating:
            return 'THINKING'
        idle_for = (self.get_clock().now() - self._last_activity).nanoseconds / 1e9
        if idle_for > IDLE_TIMEOUT_S:
            return 'IDLE'
        return 'LISTENING'

    def _tick(self):
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
        t = time.monotonic() - self._anim_t0
        p = (1 - math.cos(2 * math.pi * t / THINK_PERIOD_S)) / 2  # 0(green)..1(cyan)..0
        blue = int(LED_MAX * p)
        color = (LED_MAX << 8) | blue                             # R=0, G=LED_MAX, B=blue
        self._publish(EFFECT_SOLID, 255, 0, color)

    def _animate_speaking(self):
        # Envelope follower over the live TTS level: instant attack, slow release.
        now = time.monotonic()
        target = self._spk_raw if (now - self._spk_ts) < SPK_LEVEL_TIMEOUT_S else 0.0
        scaled = min(1.0, target * SPK_GAIN)
        if scaled > self._spk_env:
            self._spk_env = scaled
        else:
            self._spk_env = max(scaled, self._spk_env - SPK_RELEASE)
        bright = int(SPK_MIN_BRIGHT + (LED_MAX - SPK_MIN_BRIGHT) * self._spk_env)
        color = (bright << 8) | bright                            # cyan, R=0 G=B=bright
        self._publish(EFFECT_SOLID, 255, 0, color)

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
