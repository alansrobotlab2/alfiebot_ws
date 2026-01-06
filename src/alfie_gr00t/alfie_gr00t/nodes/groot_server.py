#!/usr/bin/env python3
"""GR00T N1.6 inference server node for Alfiebot.

This node runs the NVIDIA GR00T N1.6 model inference using TensorRT
and serves action predictions via ZeroMQ REP/REQ pattern.

The server:
- Loads a trained GR00T model checkpoint
- Receives observations (4 images + 22D state + language) via ZeroMQ
- Runs TensorRT inference at ~15-20 FPS
- Returns 16-step action horizon (16 x 22D)

For on-device deployment, uses IPC (Unix domain sockets) for ~5x faster
communication than TCP localhost.
"""

import os
import time
from pathlib import Path
from typing import Optional

import cv2
import msgpack
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import zmq

# GR00T imports (conditional - may not be available during setup)
try:
    from gr00t.core.inference import GrootInferenceEngine
    from gr00t.core.policy import GrootPolicy
    GROOT_AVAILABLE = True
except ImportError:
    GROOT_AVAILABLE = False
    print("Warning: GR00T SDK not available. Server will run in mock mode.")


class GrootServerNode(Node):
    """GR00T N1.6 inference server ROS2 node.

    Runs a ZeroMQ REP server that:
    1. Receives observation messages (images, state, language)
    2. Runs GR00T TensorRT inference
    3. Returns action predictions (16-step horizon)

    The server can run in two modes:
    - Production: Loads actual GR00T model and runs TensorRT inference
    - Mock: Returns random actions for testing without GPU/model
    """

    def __init__(self):
        super().__init__('groot_server')

        # Declare parameters
        self._declare_parameters()

        # Get parameters
        self.transport = self.get_parameter('transport').value
        self.bind_host = self.get_parameter('bind_host').value
        self.bind_port = self.get_parameter('bind_port').value
        self.ipc_path = self.get_parameter('ipc_path').value
        self.model_checkpoint = self.get_parameter('model_checkpoint').value
        self.use_tensorrt = self.get_parameter('use_tensorrt').value
        self.mock_mode = self.get_parameter('mock_mode').value
        self.action_horizon = self.get_parameter('action_horizon').value
        self.device = self.get_parameter('device').value

        # Build bind address
        self.bind_address = self._build_bind_address()

        # Initialize ZeroMQ
        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REP)

        # Cleanup IPC socket file if it exists
        if self.transport == 'ipc' and os.path.exists(self.ipc_path):
            os.unlink(self.ipc_path)
            self.get_logger().info(f'Cleaned up existing IPC socket: {self.ipc_path}')

        try:
            self._socket.bind(self.bind_address)
            self.get_logger().info(f'GR00T server bound to: {self.bind_address}')
        except zmq.ZMQError as e:
            self.get_logger().error(f'Failed to bind socket: {e}')
            raise

        # Initialize model
        self._policy: Optional[GrootPolicy] = None
        self._inference_engine: Optional[GrootInferenceEngine] = None

        if not self.mock_mode:
            self._load_model()
        else:
            self.get_logger().warn('Running in MOCK mode - will return random actions')

        # Statistics
        self._total_requests = 0
        self._total_inference_time_ms = 0.0
        self._last_inference_time_ms = 0.0

        # Status publisher
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        # Start inference loop in main thread
        self.get_logger().info(
            f'GR00T Server ready. transport={self.transport}, '
            f'mock={self.mock_mode}, tensorrt={self.use_tensorrt}'
        )

        # Run server loop
        self.create_timer(0.001, self._server_loop)

    def _declare_parameters(self):
        """Declare all ROS2 parameters."""
        # Transport configuration
        self.declare_parameter('transport', 'ipc')
        self.declare_parameter('bind_host', '*')  # Bind to all interfaces
        self.declare_parameter('bind_port', 5555)
        self.declare_parameter('ipc_path', '/tmp/groot_inference.sock')

        # Model configuration
        self.declare_parameter('model_checkpoint', '')
        self.declare_parameter('use_tensorrt', True)
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('action_horizon', 16)
        self.declare_parameter('device', 'cuda:0')

    def _build_bind_address(self) -> str:
        """Build ZeroMQ bind address from parameters."""
        if self.transport == 'ipc':
            return f'ipc://{self.ipc_path}'
        elif self.transport == 'tcp':
            return f'tcp://{self.bind_host}:{self.bind_port}'
        else:
            raise ValueError(f'Unknown transport: {self.transport}')

    def _load_model(self):
        """Load GR00T model checkpoint."""
        if not GROOT_AVAILABLE:
            self.get_logger().error('GR00T SDK not available. Cannot load model.')
            self.mock_mode = True
            return

        if not self.model_checkpoint or not Path(self.model_checkpoint).exists():
            self.get_logger().error(
                f'Model checkpoint not found: {self.model_checkpoint}. '
                'Running in mock mode.'
            )
            self.mock_mode = True
            return

        try:
            self.get_logger().info(f'Loading GR00T model from: {self.model_checkpoint}')

            # Load policy
            self._policy = GrootPolicy.from_checkpoint(
                checkpoint_path=self.model_checkpoint,
                device=self.device,
            )

            # Initialize TensorRT engine if enabled
            if self.use_tensorrt:
                self.get_logger().info('Building TensorRT engine...')
                self._inference_engine = GrootInferenceEngine(
                    policy=self._policy,
                    device=self.device,
                    use_fp16=True,
                )
                self.get_logger().info('TensorRT engine ready')

            self.get_logger().info('Model loaded successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.get_logger().warn('Falling back to mock mode')
            self.mock_mode = True

    def _server_loop(self):
        """Main server loop - process one request per timer callback."""
        try:
            # Non-blocking check for messages
            if self._socket.poll(timeout=1, flags=zmq.POLLIN):
                self._handle_request()
        except Exception as e:
            self.get_logger().error(f'Error in server loop: {e}')

    def _handle_request(self):
        """Handle a single inference request."""
        try:
            # Receive observation
            data = self._socket.recv(flags=zmq.NOBLOCK)
            obs = msgpack.unpackb(data, raw=False)

            # Extract observation data
            images = obs.get('images', {})
            state = np.array(obs.get('state', []), dtype=np.float32)
            language = obs.get('language', '')

            # Run inference
            start_time = time.monotonic()

            if self.mock_mode:
                actions = self._mock_inference(state)
            else:
                actions = self._run_inference(images, state, language)

            inference_time_ms = (time.monotonic() - start_time) * 1000

            # Build response
            response = {
                'actions': actions.tolist() if isinstance(actions, np.ndarray) else actions,
                'inference_time_ms': inference_time_ms,
                'status': 'ok',
            }

            # Send response
            packed = msgpack.packb(response, use_bin_type=True)
            self._socket.send(packed)

            # Update statistics
            self._total_requests += 1
            self._total_inference_time_ms += inference_time_ms
            self._last_inference_time_ms = inference_time_ms

        except zmq.Again:
            # No message available
            pass
        except Exception as e:
            self.get_logger().error(f'Error handling request: {e}')
            # Send error response
            error_response = {
                'actions': [],
                'inference_time_ms': 0.0,
                'status': 'error',
                'error_message': str(e),
            }
            packed = msgpack.packb(error_response, use_bin_type=True)
            self._socket.send(packed)

    def _mock_inference(self, state: np.ndarray) -> np.ndarray:
        """Generate mock actions for testing.

        Args:
            state: Current state vector (22D).

        Returns:
            Mock action horizon (16 x 22D).
        """
        # Return state as first action, then gradually return to zero
        # This creates a "hold current position" behavior
        actions = np.zeros((self.action_horizon, 22), dtype=np.float32)

        for i in range(self.action_horizon):
            # Exponential decay toward zero
            decay = np.exp(-i / 4.0)
            actions[i] = state * decay

        return actions

    def _run_inference(
        self,
        images: dict[str, bytes],
        state: np.ndarray,
        language: str,
    ) -> np.ndarray:
        """Run GR00T model inference.

        Args:
            images: Dictionary of JPEG-compressed images.
            state: Normalized state vector (22D).
            language: Task description string.

        Returns:
            Action horizon (16 x 22D).
        """
        # Decode images from JPEG
        image_arrays = {}
        for key, jpeg_bytes in images.items():
            img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            # Convert BGR to RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            image_arrays[key] = img

        # Prepare observation dict for GR00T
        observation = {
            'video': image_arrays,  # dict of RGB images
            'state': state,
            'language': language,
        }

        # Run inference
        if self._inference_engine is not None:
            # Use TensorRT engine
            actions = self._inference_engine.predict(observation)
        else:
            # Use PyTorch model
            actions = self._policy.predict(observation)

        return actions

    def _publish_status(self):
        """Publish server status."""
        avg_inference_ms = (
            self._total_inference_time_ms / self._total_requests
            if self._total_requests > 0
            else 0.0
        )

        status = {
            'total_requests': self._total_requests,
            'average_inference_ms': avg_inference_ms,
            'last_inference_ms': self._last_inference_time_ms,
            'mock_mode': self.mock_mode,
            'tensorrt': self.use_tensorrt and not self.mock_mode,
            'bind_address': self.bind_address,
        }

        msg = String()
        msg.data = str(status)
        self.status_pub.publish(msg)

    def destroy_node(self):
        """Clean up resources on shutdown."""
        self.get_logger().info('Shutting down GR00T server...')

        # Close socket
        if self._socket is not None:
            self._socket.close()

        # Terminate context
        if self._context is not None:
            self._context.term()

        # Cleanup IPC socket file
        if self.transport == 'ipc' and os.path.exists(self.ipc_path):
            try:
                os.unlink(self.ipc_path)
                self.get_logger().info(f'Cleaned up IPC socket: {self.ipc_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to cleanup IPC socket: {e}')

        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = GrootServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
