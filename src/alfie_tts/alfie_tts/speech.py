import sys
import threading
import time
import subprocess
import os

import numpy as np
import sounddevice as sd
from piper.voice import PiperVoice
from piper.config import SynthesisConfig

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Empty, Float32
from alfie_msgs.msg import SpeechRequest
from alfie_msgs.msg import Speaking

# Substring (case-insensitive) used to locate the reSpeaker's PulseAudio sink.
# The gen2 reSpeaker XVF3800 drives the 5W speaker AND provides the AEC far-end
# reference, so TTS must play through it. Match by name (the sink name carries a
# unit-specific serial and the ALSA card index is not stable across replugs).
RESPEAKER_SINK_MATCH = ('respeaker', 'seeed')

# Piper yields coarse chunks (often a whole sentence). To drive an amplitude-
# reactive LED, we write each chunk in small windows and publish an RMS level per
# window on `tts/level` — fine-grained and paced to playback by the blocking
# stream write. ~24 ms at the ~21 kHz output rate.
LEVEL_WINDOW_SAMPLES = 512


class AlfieTTS(Node):
    def __init__(self):

        super().__init__('alfie_speech')

        # Get package voices directory
        package_share_directory = get_package_share_directory('alfie_tts')
        voices_dir = os.path.join(package_share_directory, 'voices')

        self.model_path = os.path.join(voices_dir, 'en_US-libritts_r-medium.onnx')
        self.config_path = os.path.join(voices_dir, 'en_US-libritts_r-medium.onnx.json')
        self.speaker_id = 623
        #candidates = [65, 66, 67, 76, 515, 617, 623, 903 ]
        self.length_scale = 0.95  # >1.0 slows down speech (1.1 = 10% slower)
        self.noise_scale = 0.667  # controls pitch variation
        self.noise_w = 0.8  # controls phoneme duration variation
        self.playback_rate_scale = 0.95  # <1.0 lowers pitch, >1.0 raises pitch
        self.use_cuda = False
        self.latency = 0.15
        self.speaking = False

        # Interrupt event for barge-in: set() aborts the current utterance.
        self._interrupt = threading.Event()
        # Active output stream, shared so a barge-in on another thread can abort
        # it immediately (piper yields coarse chunks, so a flag checked between
        # chunks is not responsive enough on a long sentence).
        self._stream = None
        self._stream_lock = threading.Lock()

        # Resolve and select the reSpeaker sink as the default PulseAudio sink.
        # (PulseAudio resamples our 22.05 kHz synthesis to the reSpeaker's 16 kHz.)
        self.output_sink = self._resolve_respeaker_sink()
        if self.output_sink:
            try:
                subprocess.run(["pactl", "set-default-sink", self.output_sink], check=True)
                self.get_logger().info(f"Set default sink to {self.output_sink}")
            except Exception as e:
                self.get_logger().warn(f"Failed to set default sink: {e}")
        else:
            self.get_logger().error(
                "reSpeaker PulseAudio sink not found; TTS has no output device. "
                "Check the reSpeaker connection.")

        self.speaking_pub = self.create_publisher(
            Speaking,
            'speaking',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # Live speech amplitude (RMS 0..1) for the speaking-state LED pulse.
        self.level_pub = self.create_publisher(
            Float32,
            'tts/level',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.voice = PiperVoice.load(self.model_path, self.config_path, use_cuda=self.use_cuda)

        # Separate callback groups so a barge-in can be serviced while an
        # utterance is playing (playback blocks its own group's thread).
        speech_group = MutuallyExclusiveCallbackGroup()
        barge_group = MutuallyExclusiveCallbackGroup()

        self.speech_request_subscriber = self.create_subscription(
            SpeechRequest,
            'speechrequest',
            self.speech_request_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=speech_group,
        )

        self.barge_in_subscriber = self.create_subscription(
            Empty,
            'barge_in',
            self.barge_in_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=barge_group,
        )

        # Publish initial state (not speaking)
        self.publish_speaking(False)

    def _resolve_respeaker_sink(self):
        """Return the PulseAudio sink name for the reSpeaker, or None."""
        try:
            out = subprocess.run(
                ["pactl", "list", "sinks", "short"],
                check=True, capture_output=True, text=True).stdout
        except Exception as e:
            self.get_logger().warn(f"Could not list PulseAudio sinks: {e}")
            return None
        for line in out.splitlines():
            name = line.split('\t')[1] if '\t' in line else line
            low = name.lower()
            if any(m in low for m in RESPEAKER_SINK_MATCH):
                return name
        return None

    def publish_speaking(self, is_speaking):
        msg = Speaking()
        msg.is_speaking = is_speaking
        self.speaking = is_speaking
        self.speaking_pub.publish(msg)

    def publish_level(self, level):
        msg = Float32()
        msg.data = float(level)
        self.level_pub.publish(msg)

    def barge_in_callback(self, msg):
        # User started talking over the robot: abort the current utterance.
        if self.speaking:
            self.get_logger().info("Barge-in received; interrupting speech.")
            self._interrupt.set()
            # Abort the stream directly to flush buffered audio immediately.
            with self._stream_lock:
                if self._stream is not None:
                    try:
                        self._stream.abort()
                    except Exception:
                        pass

    def set_volume(self, volume):
        if not self.output_sink:
            return
        try:
            subprocess.run(
                ["pactl", "set-sink-volume", self.output_sink, f"{int(volume)}%"],
                check=True)
        except Exception as e:
            self.get_logger().warn(f"Failed to set sink volume: {e}")

    def speech_request_callback(self, msg):
        self.get_logger().info(f"Received speech request: {msg.text}")
        self._interrupt.clear()
        self.publish_speaking(True)
        self.set_volume(msg.volume)

        # Create synthesis config
        syn_config = SynthesisConfig(
            speaker_id=self.speaker_id,
            length_scale=self.length_scale,
            noise_scale=self.noise_scale,
            noise_w_scale=self.noise_w,
        )

        interrupted = False
        try:
            with sd.RawOutputStream(
                samplerate=int(self.voice.config.sample_rate * self.playback_rate_scale),
                channels=1,
                dtype='int16',
                latency=self.latency,
            ) as output_stream:
                with self._stream_lock:
                    self._stream = output_stream
                for audio_chunk in self.voice.synthesize(msg.text, syn_config):
                    if self._interrupt.is_set():
                        interrupted = True
                        break
                    # Write the chunk in small windows, publishing an RMS level
                    # per window so the LED pulse tracks the speech envelope.
                    data = np.frombuffer(audio_chunk.audio_int16_bytes, dtype=np.int16)
                    for i in range(0, len(data), LEVEL_WINDOW_SAMPLES):
                        if self._interrupt.is_set():
                            interrupted = True
                            break
                        win = data[i:i + LEVEL_WINDOW_SAMPLES]
                        if win.size:
                            rms = float(np.sqrt(np.mean(np.square(win.astype(np.float32))))) / 32768.0
                            self.publish_level(rms)
                            output_stream.write(win.tobytes())
                    if interrupted:
                        break
                if not interrupted:
                    # Completed without interruption: let the buffer drain.
                    time.sleep(0.25)
        except Exception as e:
            # A barge-in abort() unblocks write() with an error; that's expected.
            if not self._interrupt.is_set():
                self.get_logger().error(f"Playback error: {e}")
        finally:
            with self._stream_lock:
                self._stream = None
            self.publish_level(0.0)
            self.publish_speaking(False)



def main(args=None):
    rclpy.init(args=args)
    alfietts = AlfieTTS()

    executor = MultiThreadedExecutor()
    executor.add_node(alfietts)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        alfietts.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
