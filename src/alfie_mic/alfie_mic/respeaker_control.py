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
import errno
import re
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Header
from alfie_msgs.msg import Doa, LedCommand

from alfie_mic.xvf3800 import respeaker as xvf3800

# Fixed DSP tuning applied once at startup. See xvf3800/respeaker.py.
XVF3800_TUNING = [
    ("PP_AGCMAXGAIN", [1000.0]),
    ("PP_AGCDESIREDLEVEL", [0.03]),
    ("LED_GAMMIFY", [1]),   # gamma-correct the ring so breath/crossfade fade smoothly
]

# Live-tunable AEC / post-processing params: ROS param -> (XVF3800 param, is_float).
# Tune at runtime to explore double-talk / barge-in behaviour, e.g.
#   ros2 param set /alfie/respeaker_control pp_nlatten 0
# A float param left at its sentinel default (-1) is not written (firmware default
# kept); the on/off ints are always written at startup.
AEC_TUNABLES = {
    'pp_agc_onoff': ('PP_AGCONOFF', False, 1),
    'pp_echo':      ('PP_ECHOONOFF', False, 1),
    'pp_nlatten':   ('PP_NLATTENONOFF', False, 1),
    'ref_gain':     ('AUDIO_MGR_REF_GAIN', True, -1.0),
    'mic_gain':     ('AUDIO_MGR_MIC_GAIN', True, -1.0),
    'pp_min_ns':    ('PP_MIN_NS', True, -1.0),
    'pp_min_nn':    ('PP_MIN_NN', True, -1.0),
}

# ALSA playback mixer on the reSpeaker's USB-audio interface. This is a
# USB-audio-class control, independent of the XVF3800 USB DSP control handle,
# and it resets on replug. The board ships the mono speaker path (PCM,1) at
# -20 dB, which makes the 5W speaker far too quiet, so we drive both playback
# controls to 0 dB (max scale = 60) at startup, the same way we apply DSP tuning.
RESPEAKER_ALSA_NAME_MATCH = ("respeaker", "xvf", "seeed")
XVF3800_PCM_CONTROLS = ("PCM,0", "PCM,1")
XVF3800_PCM_MAX = 60

DOA_POLL_HZ = 10.0

# The board re-enumerates on a power blip or replug, which invalidates the pyusb
# handle permanently: every transfer then fails with ENODEV and the node would
# sit there warning forever. Drop the handle and re-find() the device instead.
RECONNECT_PERIOD_S = 2.0
# errnos that mean "this handle is dead, stop using it".
USB_DEAD_ERRNOS = (errno.ENODEV, errno.ENXIO, errno.EPIPE)
# ...and a transient error (e.g. EIO) that keeps repeating means the same thing.
USB_FAIL_LIMIT = 5


class ReSpeakerControl(Node):
    def __init__(self):
        super().__init__('respeaker_control')

        self.mic = None
        self._usb_fails = 0
        # Last values written to the ring, so streamed animations (e.g. the
        # THINKING crossfade at 30 Hz) only issue a USB write for the field that
        # actually changed — usually just LED_COLOR — keeping the DOA poll on the
        # same USB handle responsive. Cleared on reconnect: a re-enumerated board
        # is back at its defaults, so every field must be rewritten.
        self._led_last = {}

        # Params are declared once, up front; their values are (re)written to the
        # device on every connect.
        self._setup_aec_params()

        # DOA publisher (best-effort, latest-sample telemetry).
        doa_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.doa_pub = self.create_publisher(Doa, 'respeaker/doa', doa_qos)

        # LED command subscriber (reliable — commands should not be dropped).
        self.led_sub = self.create_subscription(
            LedCommand, 'respeaker/led_command', self._on_led_command, 10)

        if not self._connect():
            self.get_logger().error(
                'reSpeaker XVF3800 not found on USB (2886:001a). '
                'DOA/LED control disabled until it appears; '
                'check the udev rule and connection.')
            # Playback gain is an ALSA control, so try it even without the USB
            # DSP handle — the card may still be enumerated.
            self._apply_playback_gain()

        self.create_timer(1.0 / DOA_POLL_HZ, self._poll_doa)
        self.create_timer(RECONNECT_PERIOD_S, self._reconnect)

    # ------------------------------------------------------------------
    # connection management
    # ------------------------------------------------------------------
    def _connect(self):
        """Open the USB control handle and push all device state to it.

        Also the post-replug recovery path, so everything the board forgets on
        re-enumeration (DSP tuning, AEC params, LED ring, ALSA playback mixer)
        gets reapplied here. Returns True if the device was found.
        """
        mic = xvf3800.find()
        if mic is None:
            return False

        self.mic = mic
        self._usb_fails = 0
        self._led_last = {}
        try:
            version = '.'.join(str(v) for v in self.mic.read("VERSION"))
            self.get_logger().info(f'reSpeaker XVF3800 found, firmware {version}')
        except Exception as e:
            self.get_logger().info(f'reSpeaker XVF3800 found (version read failed: {e})')
        self._apply_tuning()
        self._write_all_aec()
        self._apply_playback_gain()
        return True

    def _reconnect(self):
        if self.mic is not None:
            return
        if self._connect():
            self.get_logger().info('reSpeaker XVF3800 reconnected.')

    def _drop_device(self, reason):
        """Release a handle that can no longer be used; _reconnect picks it up."""
        if self.mic is None:
            return
        self.get_logger().warn(
            f'reSpeaker USB handle lost ({reason}); will retry every '
            f'{RECONNECT_PERIOD_S:g}s.')
        try:
            self.mic.close()
        except Exception:
            pass
        self.mic = None
        self._usb_fails = 0

    def _note_usb_error(self, what, exc):
        """Log a failed transfer and drop the handle if the device is gone."""
        self.get_logger().warn(f'{what} failed ({type(exc).__name__}: {exc})',
                               throttle_duration_sec=5.0)
        # USBError subclasses OSError, so a dead device shows up as errno.
        code = getattr(exc, 'errno', None)
        if code in USB_DEAD_ERRNOS:
            self._drop_device(f'errno {code}')
            return
        self._usb_fails += 1
        if self._usb_fails >= USB_FAIL_LIMIT:
            self._drop_device(f'{self._usb_fails} consecutive errors')

    def _apply_tuning(self):
        for name, value in XVF3800_TUNING:
            try:
                self.mic.write(name, value)
            except Exception as e:
                self._note_usb_error(f'Mic tuning {name}={value}', e)

    def _setup_aec_params(self):
        """Declare the AEC/post-processing params and register a live-update
        callback so they can be tuned at runtime. Values reach the device in
        _write_all_aec(), which runs on every connect."""
        for pname, (_xvf, _is_float, default) in AEC_TUNABLES.items():
            self.declare_parameter(pname, default)
        self.add_on_set_parameters_callback(self._on_set_params)

    def _write_all_aec(self):
        """Push every declared AEC param to the (re)connected device."""
        for pname in AEC_TUNABLES:
            self._write_aec(pname, self.get_parameter(pname).value)
        applied = {p: self.get_parameter(p).value for p in AEC_TUNABLES}
        self.get_logger().info(f'AEC params: {applied}')

    def _write_aec(self, pname, value):
        """Write one AEC param to the device (skips float sentinels < 0)."""
        if self.mic is None:
            return
        xvf, is_float, _ = AEC_TUNABLES[pname]
        value = float(value) if is_float else int(value)
        if is_float and value < 0:
            return   # sentinel: leave firmware default
        try:
            self.mic.write(xvf, [value])
        except Exception as e:
            self._note_usb_error(f'AEC write {xvf}={value}', e)

    def _on_set_params(self, params):
        for p in params:
            if p.name in AEC_TUNABLES:
                self._write_aec(p.name, p.value)
                self.get_logger().info(f'AEC {p.name} -> {p.value}')
        return SetParametersResult(successful=True)

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
        if self.mic is None:
            return
        try:
            angle, speech = self.mic.read("DOA_VALUE")
        except Exception as e:
            self._note_usb_error('DOA read', e)
            return

        self._usb_fails = 0
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
                self._note_usb_error(f'LED write {name}={value}', e)
                if self.mic is None:
                    return

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
