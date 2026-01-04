"""ZeroMQ client for communication with GR00T inference server."""

import time
from collections import deque
from typing import Any, Callable, Optional

import msgpack
import numpy as np
import zmq


# Default IPC socket path for on-device inference
DEFAULT_IPC_PATH = "/tmp/groot_inference.sock"


def build_server_address(
    transport: str = "tcp",
    host: str = "192.168.1.100",
    port: int = 5555,
    ipc_path: str = DEFAULT_IPC_PATH,
) -> str:
    """Build ZeroMQ server address from components.

    Args:
        transport: Transport type - "tcp" for remote, "ipc" for on-device.
        host: Host address for TCP transport.
        port: Port number for TCP transport.
        ipc_path: Socket path for IPC transport.

    Returns:
        Full ZeroMQ address string.
    """
    if transport == "ipc":
        return f"ipc://{ipc_path}"
    elif transport == "tcp":
        return f"tcp://{host}:{port}"
    else:
        raise ValueError(f"Unknown transport: {transport}. Use 'tcp' or 'ipc'.")


class ZMQClient:
    """ZeroMQ client for GR00T inference server communication.

    Uses REQ/REP pattern for synchronous request-response inference.
    Observations are sent as msgpack-serialized dictionaries with
    JPEG-compressed images for efficient bandwidth usage.

    Supports two transport modes:
    - TCP: For remote inference server (e.g., tcp://192.168.1.100:5555)
    - IPC: For on-device inference (e.g., ipc:///tmp/groot_inference.sock)

    IPC is ~5x faster than TCP localhost and recommended for on-device use.
    """

    def __init__(
        self,
        server_address: str = 'tcp://192.168.1.100:5555',
        timeout_ms: int = 100,
        reconnect_delay_ms: int = 1000,
        max_reconnect_attempts: int = 5,
        logger: Optional[Callable[[str], None]] = None,
    ):
        """Initialize ZMQ client.

        Args:
            server_address: ZMQ server address (e.g., 'tcp://192.168.1.100:5555').
            timeout_ms: Receive timeout in milliseconds.
            reconnect_delay_ms: Delay between reconnection attempts.
            max_reconnect_attempts: Maximum consecutive reconnection attempts.
            logger: Optional logging function.
        """
        self.server_address = server_address
        self.timeout_ms = timeout_ms
        self.reconnect_delay_ms = reconnect_delay_ms
        self.max_reconnect_attempts = max_reconnect_attempts
        self._log = logger or (lambda msg: None)

        # ZeroMQ context and socket
        self._context: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None

        # Connection state
        self._connected = False
        self._consecutive_failures = 0

        # Statistics
        self._latency_history: deque = deque(maxlen=100)
        self._frame_id = 0

    @property
    def connected(self) -> bool:
        """Check if client is connected to server."""
        return self._connected

    @property
    def average_latency_ms(self) -> float:
        """Get average round-trip latency in milliseconds."""
        if not self._latency_history:
            return 0.0
        return sum(self._latency_history) / len(self._latency_history)

    @property
    def frame_id(self) -> int:
        """Get current frame ID counter."""
        return self._frame_id

    def connect(self) -> bool:
        """Establish connection to inference server.

        Returns:
            True if connection successful, False otherwise.
        """
        try:
            # Create context if needed
            if self._context is None:
                self._context = zmq.Context()

            # Close existing socket if any
            if self._socket is not None:
                self._socket.close(linger=0)

            # Create new REQ socket
            self._socket = self._context.socket(zmq.REQ)

            # Configure socket options
            self._socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self._socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
            self._socket.setsockopt(zmq.LINGER, 0)

            # Enable TCP keepalive
            self._socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
            self._socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 60)

            # Connect to server
            self._socket.connect(self.server_address)

            self._connected = True
            self._consecutive_failures = 0
            self._log(f'Connected to server: {self.server_address}')
            return True

        except zmq.ZMQError as e:
            self._log(f'Failed to connect: {e}')
            self._connected = False
            return False

    def disconnect(self):
        """Disconnect from server and cleanup resources."""
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None

        self._connected = False

    def close(self):
        """Close client and cleanup all resources."""
        self.disconnect()

        if self._context is not None:
            self._context.term()
            self._context = None

    def _reset_socket(self):
        """Reset socket after failure (for REQ/REP recovery)."""
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
            self._connected = False

        # Delay before reconnection
        time.sleep(self.reconnect_delay_ms / 1000.0)
        self.connect()

    def send_observation(
        self,
        images: dict[str, bytes],
        state: np.ndarray,
        language: str,
    ) -> Optional[dict[str, Any]]:
        """Send observation and receive action prediction.

        Args:
            images: Dictionary mapping camera names to JPEG bytes.
                   Keys: 'left_wide', 'right_wide', 'left_center', 'right_center'
            state: Normalized state vector (21D).
            language: Task description string.

        Returns:
            Action response dictionary with 'actions' key containing
            16x21 action horizon, or None on failure.
        """
        if not self._connected:
            if not self.connect():
                return None

        # Build observation message
        observation = {
            'timestamp': time.time(),
            'frame_id': self._frame_id,
            'images': images,
            'state': state.tolist() if isinstance(state, np.ndarray) else state,
            'language': language,
        }

        try:
            start_time = time.monotonic()

            # Serialize and send
            packed = msgpack.packb(observation, use_bin_type=True)
            self._socket.send(packed)

            # Receive response
            response_packed = self._socket.recv()
            response = msgpack.unpackb(response_packed, raw=False)

            # Track latency
            latency_ms = (time.monotonic() - start_time) * 1000
            self._latency_history.append(latency_ms)

            # Success - reset failure count and increment frame
            self._consecutive_failures = 0
            self._frame_id += 1

            return response

        except zmq.Again:
            # Timeout
            self._consecutive_failures += 1
            self._log(f'Inference timeout ({self.timeout_ms}ms)')

            # Reset socket on timeout (REQ/REP requires strict send-recv order)
            self._reset_socket()
            return None

        except zmq.ZMQError as e:
            self._consecutive_failures += 1
            self._log(f'ZMQ error: {e}')
            self._reset_socket()
            return None

        except msgpack.exceptions.UnpackException as e:
            self._consecutive_failures += 1
            self._log(f'Msgpack decode error: {e}')
            return None

    def should_reconnect(self) -> bool:
        """Check if reconnection should be attempted.

        Returns:
            True if reconnection should be attempted.
        """
        return (
            not self._connected and
            self._consecutive_failures < self.max_reconnect_attempts
        )

    def get_stats(self) -> dict:
        """Get client statistics.

        Returns:
            Dictionary with connection and performance stats.
        """
        return {
            'connected': self._connected,
            'server_address': self.server_address,
            'frame_id': self._frame_id,
            'consecutive_failures': self._consecutive_failures,
            'average_latency_ms': self.average_latency_ms,
            'latency_samples': len(self._latency_history),
        }
