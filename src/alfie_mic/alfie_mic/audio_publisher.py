import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from alfie_msgs.msg import AudioFrame
import sounddevice as sd
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time

SAMPLE_RATE = 16000  # ReSpeaker default sample rate
BLOCKSIZE = 512  # 512 samples per frame to match AudioFrame message
CHANNELS = 1
STREAM_RESET_INTERVAL = 3600  # seconds, configurable
# A 512-sample block arrives every 32 ms. If none has arrived for this long the
# stream is dead: when the reSpeaker drops off USB, PortAudio keeps polling the
# deleted ALSA device (100% CPU) and the callback never fires again, so no
# overflow status ever triggers a reset.
STREAM_STALL_S = 2.0

# NOTE: XVF3800 DSP tuning, DOA and LED control live in the respeaker_control
# node, which owns the USB control interface. This node only captures audio
# over ALSA.

class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher_ = self.create_publisher(AudioFrame, 'audio_frames', qos)
        
        self.stream = None
        self.audio_device = None
        self._reset_requested = False
        self._last_audio = time.monotonic()
        self._stalled = False
        self._open_stream()
        self.last_stream_reset = time.time()

        # Timer to check if stream reset is needed (runs in main ROS thread)
        self.create_timer(1.0, self._check_stream_reset)

    def _find_respeaker(self):
        """Return the PortAudio index of the reSpeaker, or None.

        Looked up by name on every open: after the board re-enumerates, the old
        index can point at a different device or nothing at all.
        """
        # PortAudio caches the device list at init; re-init to see a replugged board.
        sd._terminate()
        sd._initialize()
        # Match case-insensitively on "respeaker" so this works across the
        # gen1 "ReSpeaker 4 Mic Array" and gen2 "reSpeaker XVF3800 4-Mic Array".
        for i, device in enumerate(sd.query_devices()):
            if device['max_input_channels'] > 0 and 'respeaker' in device['name'].lower():
                return i
        return None

    def _open_stream(self):
        """(Re)open the capture stream. Returns True on success."""
        if self.stream is not None:
            try:
                # abort, not stop: stop drains buffers, which can hang on a
                # device that has disappeared.
                self.stream.abort()
                self.stream.close()
            except Exception as e:
                self.get_logger().warn(f'Error closing stream: {e}')
            self.stream = None
            # Small delay to let the device release
            time.sleep(0.1)

        try:
            device = self._find_respeaker()
        except Exception as e:
            self.get_logger().error(f'Could not query audio devices: {e}')
            return False
        if device is None:
            self.get_logger().warn('ReSpeaker 4 Mic Array not found; retrying every second',
                                   throttle_duration_sec=30.0)
            return False
        if device != self.audio_device:
            self.get_logger().info(
                f'Found ReSpeaker device at index {device}: {sd.query_devices(device)["name"]}')
            self.audio_device = device

        try:
            self.stream = sd.InputStream(
                device=self.audio_device,
                samplerate=SAMPLE_RATE,
                blocksize=BLOCKSIZE,
                channels=CHANNELS,
                dtype='int16',
                callback=self.audio_callback
            )
            self.stream.start()
        except Exception as e:
            self.get_logger().error(f'Error opening stream: {e}')
            self.stream = None
            return False
        self._last_audio = time.monotonic()
        return True

    def _check_stream_reset(self):
        """Check and perform stream reset from main thread (not from callback)."""
        now = time.time()
        stalled = time.monotonic() - self._last_audio > STREAM_STALL_S
        if stalled and not self._stalled and self.stream is not None:
            self.get_logger().warn(
                f'No audio for {STREAM_STALL_S:g}s (reSpeaker lost?); reopening stream.')
        self._stalled = stalled

        if (self.stream is None or stalled or self._reset_requested
                or now - self.last_stream_reset > STREAM_RESET_INTERVAL):
            if not stalled and self.stream is not None:
                self.get_logger().info('Resetting audio input stream to prevent overflow.')
            if self._open_stream():
                if stalled:
                    self.get_logger().info('Audio stream reopened.')
                self._stalled = False
                self.last_stream_reset = now
                self._reset_requested = False

    def audio_callback(self, indata, frames, time_info, status):
        self._last_audio = time.monotonic()
        if status:
            self.get_logger().warn(f'Stream status: {status}')
            # If there's an input overflow, request a reset from the main thread
            if status.input_overflow:
                self._reset_requested = True
        
        msg = AudioFrame()
        # Convert and ensure we have exactly 512 samples
        audio_data = indata.copy().flatten().astype(np.int16)
        
        # Debug: log audio data info occasionally
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        #if self._debug_counter % 100 == 0:  # Log every 100th callback
        #    self.get_logger().info(f'Audio data: shape={audio_data.shape}, min={audio_data.min()}, max={audio_data.max()}, mean={audio_data.mean():.2f}')
        
        if len(audio_data) >= 512:
            msg.audioframe = audio_data[:512].tolist()
        else:
            # Pad with zeros if we have fewer samples
            padded_data = np.zeros(512, dtype=np.int16)
            padded_data[:len(audio_data)] = audio_data
            msg.audioframe = padded_data.tolist()
            
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.stream is not None:
            node.stream.stop()
            node.stream.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
