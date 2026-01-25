#!/usr/bin/env python3
"""Standalone GR00T N1.6 inference server.

This script runs the NVIDIA GR00T N1.6 model inference using TensorRT
and serves action predictions via ZeroMQ REP/REQ pattern.

Can be run standalone or launched from the ROS2 groot_server node.

Usage:
    # Standalone with defaults
    python groot_inference_server.py

    # With custom checkpoint
    python groot_inference_server.py --checkpoint /path/to/model

    # TCP transport instead of IPC
    python groot_inference_server.py --transport tcp --port 5555

    # Mock mode for testing without GPU
    python groot_inference_server.py --mock
"""

import argparse
import logging
import os
import signal
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import msgpack
import numpy as np
import zmq

# GR00T imports (conditional - may not be available during setup)
try:
    from gr00t.policy.gr00t_policy import Gr00tPolicy
    from gr00t.data.embodiment_tags import EmbodimentTag
    GROOT_AVAILABLE = True
except ImportError as e:
    GROOT_AVAILABLE = False
    print(f"Warning: GR00T SDK not available ({e}). Server will run in mock mode.")


class GrootInferenceServer:
    """GR00T N1.6 inference server.

    Runs a ZeroMQ REP server that:
    1. Receives observation messages (images, state, language)
    2. Runs GR00T TensorRT inference
    3. Returns action predictions (16-step horizon)

    The server can run in two modes:
    - Production: Loads actual GR00T model and runs TensorRT inference
    - Mock: Returns random actions for testing without GPU/model
    """

    def __init__(
        self,
        transport: str = 'ipc',
        bind_host: str = '*',
        bind_port: int = 5555,
        ipc_path: str = '/tmp/groot_inference.sock',
        model_checkpoint: str = '',
        embodiment_tag: str = 'new_embodiment',
        mock_mode: bool = False,
        action_horizon: int = 16,
        device: str = 'cuda:0',
        logger: Optional[logging.Logger] = None,
    ):
        """Initialize the inference server.

        Args:
            transport: 'ipc' or 'tcp'
            bind_host: Host to bind for TCP (default '*' for all interfaces)
            bind_port: Port to bind for TCP
            ipc_path: Path for IPC Unix socket
            model_checkpoint: Path to GR00T model checkpoint
            embodiment_tag: Embodiment tag string
            mock_mode: If True, return mock actions without model
            action_horizon: Number of action steps to predict
            device: CUDA device string
            logger: Optional logger instance
        """
        self.transport = transport
        self.bind_host = bind_host
        self.bind_port = bind_port
        self.ipc_path = ipc_path
        self.model_checkpoint = model_checkpoint
        self.embodiment_tag_str = embodiment_tag
        self.mock_mode = mock_mode
        self.action_horizon = action_horizon
        self.device = device

        # Setup logging
        self.logger = logger or logging.getLogger(__name__)

        # Build bind address
        self.bind_address = self._build_bind_address()

        # Initialize ZeroMQ
        self._zmq_context: Optional[zmq.Context] = None
        self._socket: Optional[zmq.Socket] = None

        # Initialize model
        self._policy: Optional[Gr00tPolicy] = None
        self._embodiment_tag: Optional[EmbodimentTag] = None

        # Statistics
        self._total_requests = 0
        self._total_inference_time_ms = 0.0
        self._last_inference_time_ms = 0.0

        # Running state
        self._running = False

    def _build_bind_address(self) -> str:
        """Build ZeroMQ bind address from parameters."""
        if self.transport == 'ipc':
            return f'ipc://{self.ipc_path}'
        elif self.transport == 'tcp':
            return f'tcp://{self.bind_host}:{self.bind_port}'
        else:
            raise ValueError(f'Unknown transport: {self.transport}')

    def _setup_socket(self):
        """Setup ZeroMQ socket."""
        self._zmq_context = zmq.Context()
        self._socket = self._zmq_context.socket(zmq.REP)

        # Cleanup IPC socket file if it exists
        if self.transport == 'ipc' and os.path.exists(self.ipc_path):
            os.unlink(self.ipc_path)
            self.logger.info(f'Cleaned up existing IPC socket: {self.ipc_path}')

        try:
            self._socket.bind(self.bind_address)
            self.logger.info(f'GR00T server bound to: {self.bind_address}')
        except zmq.ZMQError as e:
            self.logger.error(f'Failed to bind socket: {e}')
            raise

    def _load_model(self):
        """Load GR00T model checkpoint."""
        if not GROOT_AVAILABLE:
            self.logger.error('GR00T SDK not available. Cannot load model.')
            self.mock_mode = True
            return

        if not self.model_checkpoint or not Path(self.model_checkpoint).exists():
            self.logger.error(
                f'Model checkpoint not found: {self.model_checkpoint}. '
                'Running in mock mode.'
            )
            self.mock_mode = True
            return

        try:
            self.logger.info(f'Loading GR00T model from: {self.model_checkpoint}')

            # Parse embodiment tag
            try:
                self._embodiment_tag = EmbodimentTag(self.embodiment_tag_str)
            except ValueError:
                self.logger.warning(
                    f'Unknown embodiment tag: {self.embodiment_tag_str}. '
                    f'Using NEW_EMBODIMENT.'
                )
                self._embodiment_tag = EmbodimentTag.NEW_EMBODIMENT

            # Load policy
            self._policy = Gr00tPolicy(
                embodiment_tag=self._embodiment_tag,
                model_path=self.model_checkpoint,
                device=self.device,
            )

            self.logger.info(
                f'Model loaded successfully (embodiment={self._embodiment_tag.value})'
            )

        except Exception as e:
            self.logger.error(f'Failed to load model: {e}')
            import traceback
            self.logger.error(traceback.format_exc())
            self.logger.warning('Falling back to mock mode')
            self.mock_mode = True

    def _handle_request(self) -> bool:
        """Handle a single inference request.

        Returns:
            True if a request was handled, False if no request available.
        """
        try:
            # Non-blocking check for messages
            if not self._socket.poll(timeout=100, flags=zmq.POLLIN):
                return False

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

            return True

        except zmq.Again:
            # No message available
            return False
        except Exception as e:
            import traceback
            import sys
            tb = traceback.format_exc()
            # Force output to stderr
            print(f'='*60, file=sys.stderr, flush=True)
            print(f'ERROR handling request: {e}', file=sys.stderr, flush=True)
            print(f'Full traceback:', file=sys.stderr, flush=True)
            print(tb, file=sys.stderr, flush=True)
            print(f'='*60, file=sys.stderr, flush=True)
            # Also log
            self.logger.error(f'Error handling request: {e}')
            self.logger.error(f'Full traceback:\n{tb}')
            # Log observation details if available
            try:
                if images:
                    msg = f'Received image keys: {list(images.keys())}'
                    print(msg, file=sys.stderr, flush=True)
                    self.logger.error(msg)
                if state is not None:
                    msg = f'Received state shape: {state.shape}'
                    print(msg, file=sys.stderr, flush=True)
                    self.logger.error(msg)
                if language:
                    msg = f'Received language: {language}'
                    print(msg, file=sys.stderr, flush=True)
                    self.logger.error(msg)
            except Exception:
                pass
            # Send error response
            error_response = {
                'actions': [],
                'inference_time_ms': 0.0,
                'status': 'error',
                'error_message': str(e),
            }
            try:
                packed = msgpack.packb(error_response, use_bin_type=True)
                self._socket.send(packed)
            except Exception:
                pass
            return True

    def _mock_inference(self, state: np.ndarray) -> np.ndarray:
        """Generate mock actions for testing.

        Args:
            state: Current state vector (22D).

        Returns:
            Mock action horizon (16 x 22D).
        """
        # Return state as first action, then gradually return to zero
        # This creates a "hold current position" behavior
        state_dim = len(state) if len(state) > 0 else 22
        actions = np.zeros((self.action_horizon, state_dim), dtype=np.float32)

        for i in range(self.action_horizon):
            # Exponential decay toward zero
            decay = np.exp(-i / 4.0)
            if len(state) > 0:
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
        # Decode images from JPEG and format for Gr00tPolicy
        # Expected format: video[key] = np.ndarray[np.uint8, (B, T, H, W, C)]
        video_dict = {}
        for key, jpeg_bytes in images.items():
            img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            # Convert BGR to RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            # Add batch and temporal dimensions: (H, W, C) -> (1, 1, H, W, C)
            video_dict[key] = img[np.newaxis, np.newaxis, ...].astype(np.uint8)

        # Format state for Gr00tPolicy
        # Expected format: state[key] = np.ndarray[np.float32, (B, T, D)]
        # State must be split according to modality config:
        #   base: [0:6], back: [6:7], left_arm: [7:12], left_hand: [12:13],
        #   right_arm: [13:18], right_hand: [18:19], head: [19:22]
        state_dict = {
            'base': state[np.newaxis, np.newaxis, 0:6].astype(np.float32),
            'back': state[np.newaxis, np.newaxis, 6:7].astype(np.float32),
            'left_arm': state[np.newaxis, np.newaxis, 7:12].astype(np.float32),
            'left_hand': state[np.newaxis, np.newaxis, 12:13].astype(np.float32),
            'right_arm': state[np.newaxis, np.newaxis, 13:18].astype(np.float32),
            'right_hand': state[np.newaxis, np.newaxis, 18:19].astype(np.float32),
            'head': state[np.newaxis, np.newaxis, 19:22].astype(np.float32),
        }

        # Prepare observation dict for GR00T
        # GR00T expects flat keys with dot notation for nested structures
        observation = {
            'video': video_dict,
            'state': state_dict,
            # Flat key for annotation - GR00T looks for 'annotation.human.task_description'
            'annotation.human.task_description': [[language]],  # (B=1, T=1)
        }

        # Log observation structure for debugging
        self.logger.info(f"Observation keys: {list(observation.keys())}")
        self.logger.info(f"  video keys: {list(video_dict.keys())}")
        self.logger.info(f"  state keys: {list(state_dict.keys())}")
        for k, v in video_dict.items():
            self.logger.info(f"    video[{k}] shape: {v.shape}")
        for k, v in state_dict.items():
            self.logger.info(f"    state[{k}] shape: {v.shape}")
        self.logger.info(f"  annotation key: 'annotation.human.task_description' = {observation.get('annotation.human.task_description')}")

        # Log policy expected keys if available
        if hasattr(self._policy, 'config'):
            self.logger.info(f"Policy config: {self._policy.config}")
        if hasattr(self._policy, 'modality_config'):
            self.logger.info(f"Policy modality_config: {self._policy.modality_config}")
        if hasattr(self._policy, '_modality_config'):
            self.logger.info(f"Policy _modality_config: {self._policy._modality_config}")

        # Run inference using get_action()
        try:
            action_dict, info = self._policy.get_action(observation)
        except KeyError as e:
            missing_key = str(e)
            self.logger.error(f"KeyError during inference: {missing_key}")
            self.logger.error(f"Observation structure provided:")
            self.logger.error(f"  Top-level keys: {list(observation.keys())}")
            self.logger.error(f"  video keys: {list(video_dict.keys())}")
            self.logger.error(f"  state keys: {list(state_dict.keys())}")
            for k, v in video_dict.items():
                self.logger.error(f"    video[{k}] shape: {v.shape}")
            for k, v in state_dict.items():
                self.logger.error(f"    state[{k}] shape: {v.shape}")
            self.logger.error(f"  annotation.human.task_description: {observation.get('annotation.human.task_description')}")
            # Try to get policy's expected keys
            if hasattr(self._policy, 'config'):
                self.logger.error(f"Policy config: {self._policy.config}")
            if hasattr(self._policy, 'modality_config'):
                self.logger.error(f"Policy modality_config: {self._policy.modality_config}")
            if hasattr(self._policy, '_modality_config'):
                self.logger.error(f"Policy _modality_config: {self._policy._modality_config}")
            raise

        # Reassemble actions from split body parts into 22D vector
        # Action dict contains keys: base, back, left_arm, left_hand, right_arm, right_hand, head
        # Each has shape (B, T, D) where D varies per body part
        # Output should be (T, 22) with parts concatenated in order

        # Define expected action parts and their dimensions (must match modality config)
        action_parts = [
            ('base', 6),       # [0:6]
            ('back', 1),       # [6:7]
            ('left_arm', 5),   # [7:12]
            ('left_hand', 1),  # [12:13]
            ('right_arm', 5),  # [13:18]
            ('right_hand', 1), # [18:19]
            ('head', 3),       # [19:22]
        ]

        # Get temporal dimension from first available action
        temporal_dim = self.action_horizon
        for key, _ in action_parts:
            if key in action_dict:
                temporal_dim = action_dict[key].shape[1]
                break

        # Assemble full action array
        actions = np.zeros((temporal_dim, 22), dtype=np.float32)
        offset = 0
        for key, dim in action_parts:
            if key in action_dict:
                # action_dict[key] has shape (B, T, D), squeeze batch dim
                part_actions = action_dict[key][0]  # (T, D)
                actions[:, offset:offset + dim] = part_actions
            offset += dim

        return actions

    def get_stats(self) -> dict:
        """Get server statistics.

        Returns:
            Dictionary with server stats.
        """
        avg_inference_ms = (
            self._total_inference_time_ms / self._total_requests
            if self._total_requests > 0
            else 0.0
        )

        return {
            'total_requests': self._total_requests,
            'average_inference_ms': avg_inference_ms,
            'last_inference_ms': self._last_inference_time_ms,
            'mock_mode': self.mock_mode,
            'embodiment': self.embodiment_tag_str,
            'bind_address': self.bind_address,
            'running': self._running,
        }

    def start(self):
        """Start the server (non-blocking setup)."""
        self._setup_socket()

        if not self.mock_mode:
            self._load_model()
        else:
            self.logger.warning('Running in MOCK mode - will return random actions')

        self._running = True
        self.logger.info(
            f'GR00T Server ready. transport={self.transport}, '
            f'mock={self.mock_mode}, embodiment={self.embodiment_tag_str}'
        )

    def spin_once(self) -> bool:
        """Process one request if available.

        Returns:
            True if a request was processed.
        """
        if not self._running:
            return False
        return self._handle_request()

    def spin(self):
        """Run the server loop until stopped."""
        self.logger.info('Starting server loop...')
        while self._running:
            self._handle_request()

    def stop(self):
        """Stop the server and cleanup resources."""
        self.logger.info('Shutting down GR00T server...')
        self._running = False

        # Close socket
        if self._socket is not None:
            self._socket.close()
            self._socket = None

        # Terminate ZMQ context
        if self._zmq_context is not None:
            self._zmq_context.term()
            self._zmq_context = None

        # Cleanup IPC socket file
        if self.transport == 'ipc' and os.path.exists(self.ipc_path):
            try:
                os.unlink(self.ipc_path)
                self.logger.info(f'Cleaned up IPC socket: {self.ipc_path}')
            except Exception as e:
                self.logger.warning(f'Failed to cleanup IPC socket: {e}')


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='GR00T N1.6 Inference Server',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # Transport options
    parser.add_argument(
        '--transport', '-t',
        choices=['ipc', 'tcp'],
        default='ipc',
        help='ZeroMQ transport type'
    )
    parser.add_argument(
        '--host',
        default='*',
        help='Host to bind for TCP transport'
    )
    parser.add_argument(
        '--port', '-p',
        type=int,
        default=5555,
        help='Port to bind for TCP transport'
    )
    parser.add_argument(
        '--ipc-path',
        default='/tmp/groot_inference.sock',
        help='Path for IPC Unix socket'
    )

    # Model options
    parser.add_argument(
        '--checkpoint', '-c',
        default='',
        help='Path to GR00T model checkpoint'
    )
    parser.add_argument(
        '--embodiment', '-e',
        default='new_embodiment',
        help='Embodiment tag string'
    )
    parser.add_argument(
        '--device', '-d',
        default='cuda:0',
        help='CUDA device string'
    )

    # Runtime options
    parser.add_argument(
        '--mock', '-m',
        action='store_true',
        help='Run in mock mode (no model, random actions)'
    )
    parser.add_argument(
        '--action-horizon',
        type=int,
        default=16,
        help='Number of action steps to predict'
    )
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable verbose logging'
    )

    return parser.parse_args()


def main():
    """Main entry point for standalone execution."""
    args = parse_args()

    # Setup logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
    )
    logger = logging.getLogger('groot_inference_server')

    # Create server
    server = GrootInferenceServer(
        transport=args.transport,
        bind_host=args.host,
        bind_port=args.port,
        ipc_path=args.ipc_path,
        model_checkpoint=args.checkpoint,
        embodiment_tag=args.embodiment,
        mock_mode=args.mock,
        action_horizon=args.action_horizon,
        device=args.device,
        logger=logger,
    )

    # Setup signal handlers
    def signal_handler(signum, frame):
        logger.info(f'Received signal {signum}, shutting down...')
        server.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Start and run server
    try:
        server.start()
        server.spin()
    except Exception as e:
        logger.error(f'Server error: {e}')
        import traceback
        logger.error(traceback.format_exc())
        sys.exit(1)
    finally:
        server.stop()


if __name__ == '__main__':
    main()
