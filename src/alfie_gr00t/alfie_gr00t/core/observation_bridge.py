"""Observation bridge for collecting and synchronizing ROS2 sensor data."""

import io
import os
import threading
from dataclasses import dataclass, field
from typing import Callable, Optional

import av
import cv2
import message_filters
import numpy as np
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from alfie_msgs.msg import RobotLowState


@dataclass
class Observation:
    """Synchronized observation from all sensors."""

    # Camera images as JPEG bytes (for ZMQ transmission)
    images: dict[str, bytes] = field(default_factory=dict)

    # Camera images as numpy arrays (for local processing)
    images_array: dict[str, np.ndarray] = field(default_factory=dict)

    # State vector (22D, raw physical units)
    state: np.ndarray = field(default_factory=lambda: np.zeros(22, dtype=np.float32))

    # Timestamp (ROS time in seconds)
    timestamp: float = 0.0

    # Whether observation is valid (all sensors received)
    valid: bool = False


class ObservationBridge:
    """Bridges ROS2 observations to GR00T format.

    Subscribes to:
    - 4 camera topics (CompressedImage)
    - Robot state topic (RobotLowState)

    Synchronizes observations using ApproximateTimeSynchronizer
    and packages them for GR00T inference.
    """

    # Camera topic names (in order matching modality config)
    CAMERA_NAMES = ['left_wide', 'right_wide', 'left_center', 'right_center']

    # Target image size for inference (must match training data MP4s: 320x240)
    IMAGE_WIDTH = 320
    IMAGE_HEIGHT = 240

    # State vector dimension
    STATE_DIM = 22

    def __init__(
        self,
        node: Node,
        camera_topic_prefix: str = '/alfie/stereo_camera',
        state_topic: str = '/alfie/robotlowstate',
        sync_slop: float = 0.05,
        jpeg_quality: int = 95,
        debug_save_images: bool = False,
        h264_conditioning: bool = False,
    ):
        """Initialize observation bridge.

        Args:
            node: ROS2 node for creating subscriptions.
            camera_topic_prefix: Prefix for camera topics.
            state_topic: Topic for robot state.
            sync_slop: Time synchronization tolerance in seconds.
            jpeg_quality: JPEG compression quality (0-100).
            debug_save_images: Save first N frames to /tmp/groot_debug_images/ for comparison.
            h264_conditioning: Apply H.264 yuv420p encode/decode round-trip to match
                training data pipeline (rosbag_to_groot.py uses libx264 CRF=23).
        """
        self.node = node
        self.jpeg_quality = jpeg_quality
        self._debug_save_images = debug_save_images
        self._h264_conditioning = h264_conditioning

        if self._h264_conditioning:
            node.get_logger().info('H.264 conditioning enabled: live frames will be '
                                  'encoded/decoded through libx264 yuv420p CRF=23')

        # Create debug image directory if needed
        if self._debug_save_images:
            os.makedirs('/tmp/groot_debug_images', exist_ok=True)
            node.get_logger().info('Debug image saving enabled: /tmp/groot_debug_images/')

        # Build camera topic names
        self.camera_topics = [
            f'{camera_topic_prefix}/{name}/image_raw/compressed'
            for name in self.CAMERA_NAMES
        ]

        # QoS profile for sensor data (best effort for real-time)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create camera subscribers using message_filters
        self.camera_subs = [
            message_filters.Subscriber(
                node, CompressedImage, topic, qos_profile=qos_sensor
            )
            for topic in self.camera_topics
        ]

        # Create state subscriber
        self.state_sub = message_filters.Subscriber(
            node, RobotLowState, state_topic, qos_profile=qos_sensor
        )

        # Create approximate time synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [*self.camera_subs, self.state_sub],
            queue_size=10,
            slop=sync_slop,
        )
        self.ts.registerCallback(self._sync_callback)

        # Latest synchronized observation
        self._latest_observation: Optional[Observation] = None
        self._observation_lock = threading.Lock()

        # Callback for new observations
        self._observation_callback: Optional[Callable[[Observation], None]] = None

        # Statistics
        self._observation_count = 0
        self._last_observation_time: Optional[float] = None

    def set_observation_callback(self, callback: Callable[[Observation], None]):
        """Set callback for new synchronized observations.

        Args:
            callback: Function called with new Observation when available.
        """
        self._observation_callback = callback

    def get_latest_observation(self) -> Optional[Observation]:
        """Get the most recent synchronized observation.

        Returns:
            Latest Observation or None if no observation available.
        """
        with self._observation_lock:
            return self._latest_observation

    def _sync_callback(
        self,
        left_wide: CompressedImage,
        right_wide: CompressedImage,
        left_center: CompressedImage,
        right_center: CompressedImage,
        state: RobotLowState,
    ):
        """Callback for synchronized sensor messages.

        Args:
            left_wide: Left wide camera image.
            right_wide: Right wide camera image.
            left_center: Left center camera image.
            right_center: Right center camera image.
            state: Robot state message.
        """
        obs = Observation()

        # Process camera images
        camera_msgs = [left_wide, right_wide, left_center, right_center]
        for name, msg in zip(self.CAMERA_NAMES, camera_msgs):
            # Decompress and resize
            img_array = self._decompress_and_resize(msg)
            if img_array is not None:
                obs.images_array[name] = img_array
                # Re-compress to JPEG for transmission
                obs.images[name] = self._compress_jpeg(img_array)

        # Extract state vector (raw physical units — server normalizes internally)
        obs.state = self._extract_state(state)

        # Set timestamp from state message (most reliable)
        obs.timestamp = (
            state.header.stamp.sec +
            state.header.stamp.nanosec * 1e-9
        )

        # Check validity (all cameras received)
        obs.valid = len(obs.images) == len(self.CAMERA_NAMES)

        # Save debug images for visual comparison with training data
        if self._debug_save_images and self._observation_count < 5:
            for name, img_array in obs.images_array.items():
                path = f'/tmp/groot_debug_images/live_{self._observation_count}_{name}.png'
                cv2.imwrite(path, cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR))
            # Also save the exact JPEG bytes that would be sent to the server
            for name, jpeg_bytes in obs.images.items():
                path = f'/tmp/groot_debug_images/live_{self._observation_count}_{name}.jpg'
                with open(path, 'wb') as f:
                    f.write(jpeg_bytes)
            # Also save raw camera JPEG (before decompress/resize/recompress)
            for name, msg in zip(self.CAMERA_NAMES, camera_msgs):
                path = f'/tmp/groot_debug_images/raw_{self._observation_count}_{name}.jpg'
                with open(path, 'wb') as f:
                    f.write(bytes(msg.data))
            # Save state vector for hybrid_image_test.py Test 11
            np.save(
                f'/tmp/groot_debug_images/live_state_{self._observation_count}.npy',
                obs.state,
            )
            if self._observation_count == 0:
                self.node.get_logger().info(
                    f'Saved debug images to /tmp/groot_debug_images/ '
                    f'(frame {self._observation_count}, {len(obs.images_array)} cameras, '
                    f'+ JPEG bytes + raw camera JPEG)'
                )

        # Update latest observation
        with self._observation_lock:
            self._latest_observation = obs
            self._observation_count += 1
            self._last_observation_time = obs.timestamp

        # Call user callback if set
        if self._observation_callback is not None and obs.valid:
            self._observation_callback(obs)

    def _decompress_and_resize(self, msg: CompressedImage) -> Optional[np.ndarray]:
        """Decompress JPEG and resize to target size.

        Args:
            msg: CompressedImage message.

        Returns:
            Resized RGB numpy array or None on failure.
        """
        try:
            # Decode JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if img is None:
                return None

            # Resize if needed
            if img.shape[1] != self.IMAGE_WIDTH or img.shape[0] != self.IMAGE_HEIGHT:
                img = cv2.resize(
                    img,
                    (self.IMAGE_WIDTH, self.IMAGE_HEIGHT),
                    interpolation=cv2.INTER_LINEAR,
                )

            # Convert BGR to RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            # Apply H.264 conditioning to match training data pipeline
            if self._h264_conditioning:
                img = self._h264_condition_frame(img)

            return img

        except Exception:
            return None

    def _compress_jpeg(self, img: np.ndarray) -> bytes:
        """Compress image to JPEG bytes.

        Args:
            img: RGB numpy array.

        Returns:
            JPEG bytes.
        """
        # Convert RGB to BGR for OpenCV
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # Encode to JPEG
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        _, encoded = cv2.imencode('.jpg', img_bgr, encode_params)

        return encoded.tobytes()

    def _h264_condition_frame(self, img: np.ndarray) -> np.ndarray:
        """Apply H.264 encode/decode round-trip to match training data pipeline.

        Training data in rosbag_to_groot.py goes through libx264 yuv420p CRF=23
        encoding into MP4 files. This introduces chroma subsampling and DCT
        compression artifacts. This method replicates that pipeline on live frames
        so the model sees the same pixel distribution it was trained on.

        Args:
            img: RGB numpy array (H, W, 3), uint8.

        Returns:
            H.264-conditioned RGB numpy array, same shape.
        """
        h, w = img.shape[:2]
        buf = io.BytesIO()

        # Encode: RGB → H.264 yuv420p (same params as rosbag_to_groot.py lines 504-521)
        output = av.open(buf, mode='w', format='mp4')
        stream = output.add_stream('libx264', rate=1)
        stream.width = w
        stream.height = h
        stream.pix_fmt = 'yuv420p'
        stream.options = {'crf': '23', 'preset': 'fast'}

        frame = av.VideoFrame.from_ndarray(img, format='rgb24')
        frame = frame.reformat(format='yuv420p')
        for packet in stream.encode(frame):
            output.mux(packet)
        for packet in stream.encode():
            output.mux(packet)
        output.close()

        # Decode: H.264 → RGB
        buf.seek(0)
        container = av.open(buf, mode='r', format='mp4')
        for frame in container.decode(video=0):
            result = frame.to_ndarray(format='rgb24')
            container.close()
            return result

        # Fallback: return original if decode fails
        container.close()
        return img

    def _extract_state(self, msg: RobotLowState) -> np.ndarray:
        """Extract 22-dimensional state vector from RobotLowState message.

        State vector layout (matches modality.json):
        [0-2]:   base linear velocity (x, y, z) from current_cmd_vel
        [3-5]:   base angular velocity (x, y, z) from current_cmd_vel
        [6]:     back joint (from back_state.current_position)
        [7-11]:  left arm joints (servos 0-4 current_location)
        [12]:    left gripper (servo 5 current_location)
        [13-17]: right arm joints (servos 6-10 current_location)
        [18]:    right gripper (servo 11 current_location)
        [19-21]: head joints (servos 12-14 current_location)

        Args:
            msg: RobotLowState message.

        Returns:
            22D state vector.
        """
        state = np.zeros(self.STATE_DIM, dtype=np.float32)

        # Base velocity from current_cmd_vel (rate-limited velocity)
        state[0] = msg.current_cmd_vel.linear.x
        state[1] = msg.current_cmd_vel.linear.y
        state[2] = msg.current_cmd_vel.linear.z
        state[3] = msg.current_cmd_vel.angular.x
        state[4] = msg.current_cmd_vel.angular.y
        state[5] = msg.current_cmd_vel.angular.z

        # Back joint (from back_state)
        state[6] = msg.back_state.current_position

        # Left arm (servos 0-4)
        for i in range(5):
            state[7 + i] = msg.servo_state[i].current_location

        # Left gripper (servo 5)
        state[12] = msg.servo_state[5].current_location

        # Right arm (servos 6-10)
        for i in range(5):
            state[13 + i] = msg.servo_state[6 + i].current_location

        # Right gripper (servo 11)
        state[18] = msg.servo_state[11].current_location

        # Head (servos 12-14)
        for i in range(3):
            state[19 + i] = msg.servo_state[12 + i].current_location

        return state

    def get_stats(self) -> dict:
        """Get observation statistics.

        Returns:
            Dictionary with observation stats.
        """
        return {
            'observation_count': self._observation_count,
            'last_observation_time': self._last_observation_time,
            'has_observation': self._latest_observation is not None,
            'observation_valid': (
                self._latest_observation.valid
                if self._latest_observation else False
            ),
        }
