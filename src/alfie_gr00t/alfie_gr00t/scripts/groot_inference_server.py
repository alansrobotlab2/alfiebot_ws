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
import torch
import zmq

# GR00T imports (conditional - may not be available during setup)
try:
    from gr00t.policy.gr00t_policy import Gr00tPolicy
    from gr00t.data.embodiment_tags import EmbodimentTag
    GROOT_AVAILABLE = True
except ImportError as e:
    GROOT_AVAILABLE = False
    print(f"Warning: GR00T SDK not available ({e}). Server will run in mock mode.")


class TensorRTDiTWrapper:
    """Wrapper for TensorRT DiT engine."""

    def __init__(self, engine_path: str, device: int = 0):
        import tensorrt as trt

        self.device = device

        if torch.cuda.is_available():
            torch.cuda.init()
            torch.cuda.set_device(device)
        else:
            raise RuntimeError("CUDA not available for TensorRT")

        self.trt_logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.trt_logger)

        with open(engine_path, "rb") as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())

        if self.engine is None:
            raise RuntimeError(f"Failed to load TensorRT engine from {engine_path}")

        self.context = self.engine.create_execution_context()
        logging.info(f"TensorRT engine loaded: {engine_path}")

    def __call__(self, sa_embs, vl_embs, timestep, image_mask=None, backbone_attention_mask=None):
        """Forward pass through TensorRT DiT."""
        sa_embs = sa_embs.to(f"cuda:{self.device}").contiguous()
        vl_embs = vl_embs.to(f"cuda:{self.device}").contiguous()
        timestep = timestep.to(f"cuda:{self.device}").contiguous()

        if image_mask is not None:
            image_mask = image_mask.to(f"cuda:{self.device}").contiguous()
        if backbone_attention_mask is not None:
            backbone_attention_mask = backbone_attention_mask.to(f"cuda:{self.device}").contiguous()

        self.context.set_input_shape("sa_embs", sa_embs.shape)
        self.context.set_input_shape("vl_embs", vl_embs.shape)
        self.context.set_input_shape("timestep", timestep.shape)
        if image_mask is not None:
            self.context.set_input_shape("image_mask", image_mask.shape)
        if backbone_attention_mask is not None:
            self.context.set_input_shape("backbone_attention_mask", backbone_attention_mask.shape)

        self.context.set_tensor_address("sa_embs", sa_embs.data_ptr())
        self.context.set_tensor_address("vl_embs", vl_embs.data_ptr())
        self.context.set_tensor_address("timestep", timestep.data_ptr())
        if image_mask is not None:
            self.context.set_tensor_address("image_mask", image_mask.data_ptr())
        if backbone_attention_mask is not None:
            self.context.set_tensor_address("backbone_attention_mask", backbone_attention_mask.data_ptr())

        # Output in FP16 (SM87 / Orin has native FP16 tensor cores, not BF16)
        output_shape = self.context.get_tensor_shape("output")
        output = torch.empty(
            tuple(output_shape), dtype=torch.float16, device=f"cuda:{self.device}"
        )
        self.context.set_tensor_address("output", output.data_ptr())

        success = self.context.execute_async_v3(torch.cuda.current_stream().cuda_stream)
        if not success:
            raise RuntimeError("TensorRT inference failed")

        return output


def replace_dit_with_tensorrt(policy, trt_engine_path: str, device: int = 0):
    """Replace the DiT forward method with TensorRT inference."""
    trt_dit = TensorRTDiTWrapper(trt_engine_path, device=device)

    def trt_forward(
        hidden_states,
        encoder_hidden_states,
        timestep,
        encoder_attention_mask=None,
        return_all_hidden_states=False,
        image_mask=None,
        backbone_attention_mask=None,
    ):
        output = trt_dit(
            sa_embs=hidden_states,
            vl_embs=encoder_hidden_states,
            timestep=timestep,
            image_mask=image_mask,
            backbone_attention_mask=backbone_attention_mask,
        )
        # TRT engine outputs FP16 (Orin lacks BF16 tensor cores), but the
        # action decoder weights are BF16 from the checkpoint. Cast to match.
        output = output.to(dtype=hidden_states.dtype)

        if return_all_hidden_states:
            raise RuntimeError("TensorRT only returns the final output. Check inference config")
        return output

    policy.model.action_head.model.forward = trt_forward
    logging.info("DiT replaced with TensorRT engine")


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
        dataset_path: str = '',
        episode_index: int = 0,
        stats_path: str = '',
        trt_engine_path: str = '',
        denoising_steps: int = 0,
        enable_viz: bool = False,
        viz_port: int = 7860,
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
            dataset_path: Path to LeRobot-format dataset for replay mode
            episode_index: Episode index to replay
            stats_path: Path to stats.json for normalizing replay actions
            trt_engine_path: Path to TensorRT engine file (.trt). Enables TRT mode.
            denoising_steps: Override number of denoising steps (0 = use model default).
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
        self.dataset_path = dataset_path
        self.episode_index = episode_index
        self.stats_path = stats_path
        self.trt_engine_path = trt_engine_path
        self.denoising_steps = denoising_steps

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

        # Visualizer
        self._enable_viz = enable_viz
        self._viz = None
        self._viz_port = viz_port
        if enable_viz:
            try:
                from alfie_gr00t.viz import GrootVisualizer
            except ModuleNotFoundError:
                # Standalone execution — add package root to sys.path
                import sys
                _pkg_root = str(Path(__file__).resolve().parents[2])
                if _pkg_root not in sys.path:
                    sys.path.insert(0, _pkg_root)
                from alfie_gr00t.viz import GrootVisualizer
            self._viz = GrootVisualizer(enable=True, port=viz_port)

        # Replay state
        self.replay_mode = bool(self.dataset_path)
        self._replay_actions: Optional[np.ndarray] = None
        self._replay_step = 0
        self._replay_total_steps = 0
        self._replay_done = False

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

            # Override denoising steps if requested
            if self.denoising_steps > 0:
                self._policy.model.action_head.num_inference_timesteps = (
                    self.denoising_steps
                )
                self.logger.info(
                    f'Denoising steps set to {self.denoising_steps}'
                )

            # Replace DiT with TensorRT engine if path provided
            if self.trt_engine_path:
                device_idx = int(self.device.split(':')[-1]) if ':' in self.device else 0
                replace_dit_with_tensorrt(
                    self._policy, self.trt_engine_path, device=device_idx
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

            if self.replay_mode:
                # Log replay progress every 50 steps, plus first and last active step
                if (
                    self._replay_step == 0
                    or self._replay_step % 50 == 0
                    or self._replay_step == self._replay_total_steps - 1
                ) and not self._replay_done:
                    self.logger.info(
                        f'Replay inference — step {self._replay_step}/'
                        f'{self._replay_total_steps} (episode {self.episode_index})'
                    )
                actions = self._replay_inference(state)
            elif self.mock_mode:
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

            # Update visualizer
            if self._viz is not None:
                viz_start = time.monotonic()
                self._viz.update(observation=obs, response=response)
                viz_ms = (time.monotonic() - viz_start) * 1000
                if self._total_requests % 50 == 0:
                    self.logger.info(
                        f'[perf] inference={inference_time_ms:.1f}ms, '
                        f'viz_update={viz_ms:.1f}ms'
                    )

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

    def _load_replay_data(self):
        """Load episode actions from parquet and normalize for replay."""
        import json
        import pandas as pd

        dataset_path = Path(self.dataset_path)

        # Try flat layout first, then chunked layout
        parquet_path = (
            dataset_path / f'data/episode_{self.episode_index:06d}.parquet'
        )
        if not parquet_path.exists():
            chunk_idx = self.episode_index // 1000
            parquet_path = (
                dataset_path
                / f'data/chunk-{chunk_idx:03d}/episode_{self.episode_index:06d}.parquet'
            )

        if not parquet_path.exists():
            self.logger.error(f'Parquet not found: {parquet_path}')
            self.logger.warning('Falling back to mock mode')
            self.replay_mode = False
            self.mock_mode = True
            return

        df = pd.read_parquet(parquet_path)
        raw_actions = np.array(df['action'].tolist(), dtype=np.float32)

        # Use raw actions directly — production mode returns unnormalized
        # (raw physical units) actions, so replay mode must match.
        # The Gr00tPolicy internally handles normalization/denormalization;
        # the client expects raw values (velocity m/s, joint positions rad).
        self._replay_actions = raw_actions
        self.logger.info(f'Replay actions loaded as raw values (no normalization)')

        self._replay_step = 0
        self._replay_total_steps = len(self._replay_actions)
        self._replay_done = False

        self.logger.info(
            f'Loaded episode {self.episode_index}: '
            f'{self._replay_total_steps} steps from {parquet_path}'
        )

    def _replay_inference(self, state: np.ndarray) -> np.ndarray:
        """Return next action chunk from pre-recorded episode.

        Args:
            state: Current state vector (ignored, actions come from dataset).

        Returns:
            Action horizon (action_horizon x 22D), normalized.
        """
        state_dim = 22
        actions = np.zeros((self.action_horizon, state_dim), dtype=np.float32)

        for i in range(self.action_horizon):
            step = self._replay_step + i
            if step < self._replay_total_steps:
                actions[i] = self._replay_actions[step]
            else:
                # Pad with last valid action but zero out base velocity
                # so the robot stops driving. Indices 0:6 are base twist
                # (linear x/y/z, angular x/y/z). Keep joint positions
                # (indices 6+) at their last values to hold pose.
                actions[i] = self._replay_actions[-1].copy()
                actions[i, 0:6] = 0.0
                if not self._replay_done:
                    self._replay_done = True
                    self.logger.info(
                        f'Episode {self.episode_index} replay complete '
                        f'at step {self._replay_step} — base velocity zeroed'
                    )

        # Advance 1 step per request (matches 15 FPS client rate)
        # Stop advancing once we've exhausted the episode
        if self._replay_step < self._replay_total_steps:
            self._replay_step += 1

        return actions

    def set_episode(self, episode_index: int):
        """Switch to a different episode for replay."""
        if not self.replay_mode:
            self.logger.warning('set_episode only works in replay mode')
            return
        self.episode_index = episode_index
        self._load_replay_data()

    def reset_replay(self):
        """Reset replay to beginning of current episode."""
        self._replay_step = 0
        self._replay_done = False

    def _run_inference(
        self,
        images: dict[str, bytes],
        state: np.ndarray,
        language: str,
    ) -> np.ndarray:
        """Run GR00T model inference.

        Args:
            images: Dictionary of JPEG-compressed images.
            state: Raw state vector (22D) — Gr00tPolicy normalizes internally.
            language: Task description string.

        Returns:
            Action horizon (16 x 22D), unnormalized.
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

        # Format language for Gr00tPolicy
        # Expected format: language[key] = list[list[str]] with shape (B, T)
        # Key must match modality config from fine-tuning
        language_dict = {
            'annotation.human.action.task_description': [[language]]  # (1, 1) - batch size 1, temporal 1
        }

        # Prepare observation dict for GR00T
        # GR00T expects language as a dict: {"annotation.human.task_description": [[str]]}
        observation = {
            'video': video_dict,
            'state': state_dict,
            'language': language_dict,
        }

        # Log model input state every 10 requests for debugging
        if self._total_requests % 10 == 0:
            self.logger.info(
                f"[input] head=({state[19]:.3f},{state[20]:.3f},{state[21]:.3f}) "
                f"r_arm=({state[13]:.3f},{state[14]:.3f},{state[15]:.3f},{state[16]:.3f},{state[17]:.3f}) "
                f"r_grip={state[18]:.3f} back={state[6]:.3f} fwd={state[0]:.3f}"
            )

        # Log observation structure periodically (every 50 requests to avoid spam)
        if self._total_requests % 50 == 0:
            self.logger.info(f"Observation: video={list(video_dict.keys())}, state={list(state_dict.keys())}")
            for k, v in state_dict.items():
                self.logger.info(f"  state[{k}]={v[0, 0, :].tolist()}")

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

        # Log action_dict diagnostics periodically
        if self._total_requests % 50 == 0:
            self.logger.info(f"action_dict keys: {list(action_dict.keys())}")
            for key, val in action_dict.items():
                self.logger.info(
                    f"  {key}: shape={val.shape}, "
                    f"mean={val.mean():.4f}, range=[{val.min():.4f}, {val.max():.4f}]"
                )

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

        # Log model output actions every 10 requests for debugging
        if self._total_requests % 10 == 0:
            a = actions[0]
            self.logger.info(
                f"[output] head=({a[19]:.3f},{a[20]:.3f},{a[21]:.3f}) "
                f"r_arm=({a[13]:.3f},{a[14]:.3f},{a[15]:.3f},{a[16]:.3f},{a[17]:.3f}) "
                f"r_grip={a[18]:.3f} back={a[6]:.3f} fwd={a[0]:.3f}"
            )

        # Log assembled action diagnostics periodically
        if self._total_requests % 50 == 0:
            self.logger.info(
                f"Assembled actions[0]: base={actions[0, 0:6]}, "
                f"right_arm={actions[0, 13:18]}, head={actions[0, 19:22]}"
            )

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

        stats = {
            'total_requests': self._total_requests,
            'average_inference_ms': avg_inference_ms,
            'last_inference_ms': self._last_inference_time_ms,
            'mock_mode': self.mock_mode,
            'replay_mode': self.replay_mode,
            'embodiment': self.embodiment_tag_str,
            'bind_address': self.bind_address,
            'running': self._running,
        }

        if self.replay_mode:
            stats.update({
                'replay_episode': self.episode_index,
                'replay_step': self._replay_step,
                'replay_total_steps': self._replay_total_steps,
                'replay_done': self._replay_done,
            })

        return stats

    def start(self):
        """Start the server (non-blocking setup)."""
        self._setup_socket()

        if self.replay_mode:
            self._load_replay_data()
            self.logger.info(
                f'Running in REPLAY mode - episode {self.episode_index}, '
                f'{self._replay_total_steps} steps'
            )
        elif not self.mock_mode:
            self._load_model()
        else:
            self.logger.warning('Running in MOCK mode - will return random actions')

        self._running = True
        self.logger.info(
            f'GR00T Server ready. transport={self.transport}, '
            f'replay={self.replay_mode}, mock={self.mock_mode}, '
            f'embodiment={self.embodiment_tag_str}'
        )
        if self._enable_viz:
            self.logger.info(f'Gradio visualization available at http://localhost:{self._viz_port}')


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

        # Close visualizer
        if self._viz is not None:
            self._viz.close()
            self._viz = None

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
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
modes:
  The server runs in one of three modes (checked in this priority order):

  1. Replay mode   --dataset-path is set
                   Serves pre-recorded actions from a LeRobot-format dataset.
                   No GPU or model required.

  2. Mock mode     --mock flag is set
                   Returns dummy actions for testing without GPU/model.

  3. Production    (default) --checkpoint is set
                   Loads a GR00T model and runs TensorRT inference.

examples:
  # Replay episode 3 over TCP
  %(prog)s --dataset-path /home/alfie/Isaac-GR00T/alfiebot.CanDoChallenge \\
           --episode-index 3 --transport tcp --port 5555

  # Mock mode for testing
  %(prog)s --mock --transport tcp --port 5555

  # Production inference
  %(prog)s --checkpoint /home/alfie/cando --transport tcp --port 5555
""",
    )

    # Transport options
    transport_group = parser.add_argument_group('transport')
    transport_group.add_argument(
        '--transport', '-t',
        choices=['ipc', 'tcp'],
        default='ipc',
        help='ZeroMQ transport type (default: %(default)s)'
    )
    transport_group.add_argument(
        '--host',
        default='*',
        help='Host to bind for TCP transport (default: %(default)s)'
    )
    transport_group.add_argument(
        '--port', '-p',
        type=int,
        default=5555,
        help='Port to bind for TCP transport (default: %(default)s)'
    )
    transport_group.add_argument(
        '--ipc-path',
        default='/tmp/groot_inference.sock',
        help='Path for IPC Unix socket (default: %(default)s)'
    )

    # Model options
    model_group = parser.add_argument_group('model')
    model_group.add_argument(
        '--checkpoint', '-c',
        default='',
        help='Path to GR00T model checkpoint'
    )
    model_group.add_argument(
        '--embodiment', '-e',
        default='new_embodiment',
        help='Embodiment tag string (default: %(default)s)'
    )
    model_group.add_argument(
        '--device', '-d',
        default='cuda:0',
        help='CUDA device string (default: %(default)s)'
    )
    model_group.add_argument(
        '--trt-engine-path',
        default='',
        help='Path to TensorRT engine file (.trt). Enables TRT inference.'
    )
    model_group.add_argument(
        '--denoising-steps',
        type=int,
        default=0,
        help='Override number of denoising steps (0 = use model default of 4)'
    )

    # Replay mode options
    replay_group = parser.add_argument_group('replay mode')
    replay_group.add_argument(
        '--dataset-path',
        default='',
        help='Path to LeRobot-format dataset (enables replay mode)'
    )
    replay_group.add_argument(
        '--episode-index',
        type=int,
        default=0,
        help='Episode index to replay (default: %(default)s)'
    )
    replay_group.add_argument(
        '--stats-path',
        default='',
        help='Path to stats.json for normalizing replay actions '
             '(default: {dataset_path}/meta/stats.json)'
    )

    # Runtime options
    runtime_group = parser.add_argument_group('runtime')
    runtime_group.add_argument(
        '--mock', '-m',
        action='store_true',
        help='Run in mock mode (no model, random actions)'
    )
    runtime_group.add_argument(
        '--action-horizon',
        type=int,
        default=16,
        help='Number of action steps to predict (default: %(default)s)'
    )
    runtime_group.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable verbose logging'
    )
    runtime_group.add_argument(
        '--enable-viz',
        action='store_true',
        help='Enable Gradio visualizer for inference I/O'
    )
    runtime_group.add_argument(
        '--viz-port',
        type=int,
        default=7860,
        help='Port for Gradio visualizer (default: %(default)s)'
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

    # Silence noisy third-party loggers even in verbose mode
    for name in ('matplotlib', 'PIL', 'httpcore', 'httpx', 'urllib3', 'asyncio'):
        logging.getLogger(name).setLevel(logging.WARNING)

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
        dataset_path=args.dataset_path,
        episode_index=args.episode_index,
        stats_path=args.stats_path,
        trt_engine_path=args.trt_engine_path,
        denoising_steps=args.denoising_steps,
        enable_viz=args.enable_viz,
        viz_port=args.viz_port,
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
