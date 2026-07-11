"""
respeaker_control node — owns the reSpeaker XVF3800 USB control interface.

Responsibilities:
  * Apply DSP tuning (AGC / echo suppression) once at startup.
  * Poll direction-of-arrival and publish it on /respeaker/doa.
  * Drive the 12-LED ring from /respeaker/led_command.

This is the *single* owner of the pyusb control handle. Audio capture
(audio_publisher) goes through ALSA and does not touch USB control, so the two
nodes can run side by side without fighting over the device.
"""
import re
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Header
from alfie_msgs.msg import Doa, LedCommand

from alfie_mic.xvf3800 import respeaker as xvf3800

# DSP post-processing tuning applied at startup. See xvf3800/respeaker.py.
XVF3800_TUNING = [
    ("PP_AGCONOFF", [1]),
    ("PP_AGCMAXGAIN", [1000.0]),
    ("PP_AGCDESIREDLEVEL", [0.03]),
    ("PP_ECHOONOFF", [1]),
    ("PP_NLATTENONOFF", [1]),
    ("LED_GAMMIFY", [1]),   # gamma-correct the ring so breath/crossfade fade smoothly
]

# ALSA playback mixer on the reSpeaker's USB-audio interface. This is a
# USB-audio-class control, independent of the XVF3800 USB DSP control handle,
# and it resets on replug. The board ships the mono speaker path (PCM,1) at
# -20 dB, which makes the 5W speaker far too quiet, so we drive both playback
# controls to 0 dB (max scale = 60) at startup, the same way we apply DSP tuning.
RESPEAKER_ALSA_NAME_MATCH = ("respeaker", "xvf", "seeed")
XVF3800_PCM_CONTROLS = ("PCM,0", "PCM,1")
XVF3800_PCM_MAX = 60

DOA_POLL_HZ = 10.0


class ReSpeakerControl(Node):
    def __init__(self):
        super().__init__('respeaker_control')

        self.mic = xvf3800.find()
        if self.mic is None:
            self.get_logger().error(
                'reSpeaker XVF3800 not found on USB (2886:001a). '
                'DOA/LED control disabled; check the udev rule and connection.')
        else:
            try:
                version = '.'.join(str(v) for v in self.mic.read("VERSION"))
                self.get_logger().info(f'reSpeaker XVF3800 found, firmware {version}')
            except Exception as e:
                self.get_logger().info(f'reSpeaker XVF3800 found (version read failed: {e})')
            self._apply_tuning()

        # Playback gain is an ALSA control, so set it regardless of whether the
        # USB DSP handle came up.
        self._apply_playback_gain()

        # DOA publisher (best-effort, latest-sample telemetry).
        doa_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.doa_pub = self.create_publisher(Doa, 'respeaker/doa', doa_qos)

        # LED command subscriber (reliable — commands should not be dropped).
        self.led_sub = self.create_subscription(
            LedCommand, 'respeaker/led_command', self._on_led_command, 10)
        # Last values written to the ring, so streamed animations (e.g. the
        # THINKING crossfade at 30 Hz) only issue a USB write for the field that
        # actually changed — usually just LED_COLOR — keeping the DOA poll on the
        # same USB handle responsive.
        self._led_last = {}

        if self.mic is not None:
            self.create_timer(1.0 / DOA_POLL_HZ, self._poll_doa)

    def _apply_tuning(self):
        for name, value in XVF3800_TUNING:
            try:
                self.mic.write(name, value)
            except Exception as e:
                self.get_logger().warn(
                    f'Mic tuning {name}={value} failed ({type(e).__name__}: {e})')

    def _apply_playback_gain(self):
        card = self._find_alsa_card()
        if card is None:
            self.get_logger().warn(
                'reSpeaker ALSA card not found; leaving playback mixer unchanged.')
            return
        for ctrl in XVF3800_PCM_CONTROLS:
            try:
                subprocess.run(
                    ['amixer', '-c', str(card), 'sset', ctrl, str(XVF3800_PCM_MAX)],
                    check=True, capture_output=True, text=True)
            except Exception as e:
                self.get_logger().warn(
                    f'Playback gain {ctrl}={XVF3800_PCM_MAX} failed '
                    f'({type(e).__name__}: {e})')
        self.get_logger().info(
            f'reSpeaker playback mixer set to max on ALSA card {card}')

    @staticmethod
    def _find_alsa_card():
        """Return the ALSA card index for the reSpeaker, or None."""
        try:
            with open('/proc/asound/cards') as f:
                text = f.read()
        except OSError:
            return None
        for line in text.splitlines():
            m = re.match(r'\s*(\d+)\s', line)
            if m and any(k in line.lower() for k in RESPEAKER_ALSA_NAME_MATCH):
                return int(m.group(1))
        return None

    def _poll_doa(self):
        try:
            angle, speech = self.mic.read("DOA_VALUE")
        except Exception as e:
            self.get_logger().warn(f'DOA read failed ({type(e).__name__}: {e})',
                                   throttle_duration_sec=5.0)
            return

        msg = Doa()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'respeaker'
        msg.angle = int(angle)
        msg.speech_detected = bool(speech)
        self.doa_pub.publish(msg)

    def _on_led_command(self, msg: LedCommand):
        if self.mic is None:
            return
        # Set colour/brightness/speed first, then switch the effect mode. Skip any
        # field whose value is unchanged from the last command so a high-rate
        # colour animation costs a single USB write per frame.
        writes = [
            ("LED_COLOR", int(msg.color)),
            ("LED_BRIGHTNESS", int(msg.brightness)),
            ("LED_SPEED", int(msg.speed)),
            ("LED_EFFECT", int(msg.effect)),
        ]
        for name, value in writes:
            if self._led_last.get(name) == value:
                continue
            try:
                self.mic.write(name, [value])
                self._led_last[name] = value
            except Exception as e:
                self.get_logger().warn(
                    f'LED write {name}={value} failed ({type(e).__name__}: {e})')

    def destroy_node(self):
        if self.mic is not None:
            try:
                self.mic.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ReSpeakerControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
