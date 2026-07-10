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
]

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

        # DOA publisher (best-effort, latest-sample telemetry).
        doa_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.doa_pub = self.create_publisher(Doa, 'respeaker/doa', doa_qos)

        # LED command subscriber (reliable — commands should not be dropped).
        self.led_sub = self.create_subscription(
            LedCommand, 'respeaker/led_command', self._on_led_command, 10)

        if self.mic is not None:
            self.create_timer(1.0 / DOA_POLL_HZ, self._poll_doa)

    def _apply_tuning(self):
        for name, value in XVF3800_TUNING:
            try:
                self.mic.write(name, value)
            except Exception as e:
                self.get_logger().warn(
                    f'Mic tuning {name}={value} failed ({type(e).__name__}: {e})')

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
        # Set colour/brightness/speed first, then switch the effect mode.
        writes = [
            ("LED_COLOR", [int(msg.color)]),
            ("LED_BRIGHTNESS", [int(msg.brightness)]),
            ("LED_SPEED", [int(msg.speed)]),
            ("LED_EFFECT", [int(msg.effect)]),
        ]
        for name, value in writes:
            try:
                self.mic.write(name, value)
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
