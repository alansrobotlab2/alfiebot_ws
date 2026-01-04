"""Observation bridge for collecting and synchronizing ROS2 sensor data."""

import threading
from dataclasses import dataclass, field
from typing import Callable, Optional

import cv2
import message_filters
import numpy as np
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from alfie_msgs.msg import RobotLowState

from .normalization import Normalizer


@dataclass
class Observation:
    """Synchronized observation from all sensors."""

    # Camera images as JPEG bytes (for ZMQ transmission)
    images: dict[str, bytes] = field(default_factory=dict)

    # Camera images as numpy arrays (for local processing)
    images_array: dict[str, np.ndarray] = field(default_factory=dict)

    # State vector (21D, raw)
    state: np.ndarray = field(default_factory=lambda: np.zeros(21, dtype=np.float32))

    # State vector (21D, normalized)
    state_normalized: np.ndarray = field(default_factory=lambda: np.zeros(21, dtype=np.float32))

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

    # Target image size for inference
    IMAGE_WIDTH = 320
    IMAGE_HEIGHT = 280

    # State vector dimension
    STATE_DIM = 21

    def __init__(
        self,
        node: Node,
        camera_topic_prefix: str = '/alfie/stereo_camera',
        state_topic: str = '/alfie/robotlowstate',
        sync_slop: float = 0.05,
        normalizer: Optional[Normalizer] = None,
        jpeg_quality: int = 80,
    ):
        """Initialize observation bridge.

        Args:
            node: ROS2 node for creating subscriptions.
            camera_topic_prefix: Prefix for camera topics.
            state_topic: Topic for robot state.
            sync_slop: Time synchronization tolerance in seconds.
            normalizer: Optional normalizer for state vectors.
            jpeg_quality: JPEG compression quality (0-100).
        """
        self.node = node
        self.normalizer = normalizer or Normalizer()
        self.jpeg_quality = jpeg_quality

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

        # Extract state vector
        obs.state = self._extract_state(state)
        obs.state_normalized = self.normalizer.normalize_state(obs.state)

        # Set timestamp from state message (most reliable)
        obs.timestamp = (
            state.header.stamp.sec +
            state.header.stamp.nanosec * 1e-9
        )

        # Check validity (all cameras received)
        obs.valid = len(obs.images) == len(self.CAMERA_NAMES)

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

    def _extract_state(self, msg: RobotLowState) -> np.ndarray:
        """Extract 21-dimensional state vector from RobotLowState message.

        State vector layout (matches modality.json):
        [0-2]:   base linear velocity (x, y, z) from current_cmd_vel
        [3-5]:   base angular velocity (x, y, z) from current_cmd_vel
        [6-11]:  left arm joints (servos 0-5 current_location)
        [12-17]: right arm joints (servos 6-11 current_location)
        [18-20]: head joints (servos 12-14 current_location)

        Args:
            msg: RobotLowState message.

        Returns:
            21D state vector.
        """
        state = np.zeros(self.STATE_DIM, dtype=np.float32)

        # Base velocity from current_cmd_vel (rate-limited velocity)
        state[0] = msg.current_cmd_vel.linear.x
        state[1] = msg.current_cmd_vel.linear.y
        state[2] = msg.current_cmd_vel.linear.z
        state[3] = msg.current_cmd_vel.angular.x
        state[4] = msg.current_cmd_vel.angular.y
        state[5] = msg.current_cmd_vel.angular.z

        # Left arm (servos 0-5)
        for i in range(6):
            state[6 + i] = msg.servo_state[i].current_location

        # Right arm (servos 6-11)
        for i in range(6):
            state[12 + i] = msg.servo_state[6 + i].current_location

        # Head (servos 12-14)
        for i in range(3):
            state[18 + i] = msg.servo_state[12 + i].current_location

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
