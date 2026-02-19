#!/usr/bin/env python3
"""GR00T N1.6 inference server.

Thin wrapper around NVIDIA's PolicyServer + Gr00tPolicy. Adds a
JpegPolicyWrapper that translates JPEG-over-WiFi observations from the
Jetson client into the native Gr00tPolicy observation format.

Usage:
    # Production inference
    python groot_inference_server.py --checkpoint /path/to/model --host 0.0.0.0

    # Replay mode (no GPU needed)
    python groot_inference_server.py --dataset-path /path/to/dataset --episode-index 3

    # With torch.compile optimization
    python groot_inference_server.py --checkpoint /path/to/model --torch-compile
"""

import argparse
import gc
import logging
import os
import signal
import sys
from pathlib import Path
from typing import Any

# Jetson unified memory optimization: configure PyTorch CUDA allocator before any CUDA ops.
# NOTE: expandable_segments:True is BROKEN on Jetson (PyTorch 2.8 + CUDA 12.6 + r36.5).
os.environ.setdefault(
    "PYTORCH_CUDA_ALLOC_CONF",
    "garbage_collection_threshold:0.6",
)

import cv2
import numpy as np
import torch
import zmq

# Enable TF32 tensor cores for FP32 matmuls (~2x speedup, negligible precision loss).
# Orin SM87 supports TF32 but PyTorch doesn't enable it by default.
# NOTE: Must use legacy API — new fp32_precision API conflicts with torch.compile's
# inductor cache hashing (RuntimeError: mix of legacy and new APIs).
torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True

from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.policy.gr00t_policy import Gr00tPolicy
from gr00t.policy.policy import BasePolicy
from gr00t.policy.replay_policy import ReplayPolicy
from gr00t.policy.server_client import PolicyServer


# Body-part slicing for flat 22D state → Gr00tPolicy's split format
STATE_PARTS = [
    ('base', 0, 6),
    ('back', 6, 7),
    ('left_arm', 7, 12),
    ('left_hand', 12, 13),
    ('right_arm', 13, 18),
    ('right_hand', 18, 19),
    ('head', 19, 22),
]


###############################################################################
# TensorRT DiT Wrapper
###############################################################################


class TensorRTDiTWrapper:
    """Wrapper for TensorRT DiT engine.

    Optimized for Orin AGX (SM87) with Jetson unified memory:
    - Dedicated CUDA stream for TRT execution (avoids default stream sync overhead)
    - Auto-detects engine input/output dtypes (no hardcoded assumptions)
    - Pre-allocated output buffer reused across diffusion steps (avoids alloc/free churn)
    - Event-based synchronization (non-blocking)
    - Engine file buffer freed immediately after deserialization (unified memory)
    """

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

        # Read and deserialize separately so we can free the file buffer immediately.
        # On Jetson unified memory, the file buffer competes with GPU allocation.
        with open(engine_path, "rb") as f:
            engine_data = f.read()
        self.engine = self.runtime.deserialize_cuda_engine(engine_data)
        del engine_data
        gc.collect()

        if self.engine is None:
            raise RuntimeError(f"Failed to load TensorRT engine from {engine_path}")

        self.context = self.engine.create_execution_context()

        # Dedicated CUDA stream for TRT execution (avoids default stream sync overhead)
        self.stream = torch.cuda.Stream(device=device)

        # Detect input dtypes from engine so we can cast inputs to match
        self.input_dtypes = {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.input_dtypes[name] = self._trt_dtype_to_torch(self.engine.get_tensor_dtype(name))

        # Detect output dtype from engine
        output_dtype = self.engine.get_tensor_dtype("output")
        self.engine_output_dtype = self._trt_dtype_to_torch(output_dtype)

        # When model runs BF16 (flash_attention_2) and TRT outputs FP16, convert to match.
        self.convert_to_bf16 = (self.engine_output_dtype == torch.float16)
        if self.convert_to_bf16:
            logging.info(f"TRT output dtype: {output_dtype} -> will convert FP16 to BF16 for action decoder")
        else:
            logging.info(f"TRT output dtype: {output_dtype} -> torch {self.engine_output_dtype}")

        # Pre-allocated output buffer — reused across diffusion steps to avoid
        # repeated alloc/free on Jetson unified memory.
        self._output_buf = None

        # Cache for TRT context shape setup — shapes are constant at batch=1
        # with fixed input, so set_input_shape only needs to run once.
        self._shapes_configured = False

        logging.info(f"TensorRT engine loaded: {engine_path}")

    def _trt_dtype_to_torch(self, trt_dtype):
        """Convert TensorRT dtype to PyTorch dtype."""
        import tensorrt as trt

        dtype_map = {
            trt.float32: torch.float32,
            trt.float16: torch.float16,
            trt.bfloat16: torch.bfloat16,
            trt.int8: torch.int8,
            trt.int32: torch.int32,
            trt.int64: torch.int64,
            trt.bool: torch.bool,
        }
        return dtype_map.get(trt_dtype, torch.float32)  # Default to fp32 (safe fallback)

    def __call__(self, sa_embs, vl_embs, timestep, image_mask=None, backbone_attention_mask=None):
        """Forward pass through TensorRT DiT."""
        # Ensure default stream ops (backbone, action_encoder, etc.) complete
        self.stream.wait_stream(torch.cuda.current_stream())

        with torch.cuda.stream(self.stream):
            # Cast to engine's expected dtypes (no device move — Jetson unified memory)
            sa_embs = sa_embs.to(dtype=self.input_dtypes.get("sa_embs", sa_embs.dtype))
            if not sa_embs.is_contiguous():
                sa_embs = sa_embs.contiguous()
            vl_embs = vl_embs.to(dtype=self.input_dtypes.get("vl_embs", vl_embs.dtype))
            if not vl_embs.is_contiguous():
                vl_embs = vl_embs.contiguous()
            timestep = timestep.to(dtype=self.input_dtypes.get("timestep", timestep.dtype))
            if not timestep.is_contiguous():
                timestep = timestep.contiguous()

            if image_mask is not None:
                image_mask = image_mask.to(dtype=self.input_dtypes.get("image_mask", image_mask.dtype))
                if not image_mask.is_contiguous():
                    image_mask = image_mask.contiguous()
            if backbone_attention_mask is not None:
                backbone_attention_mask = backbone_attention_mask.to(dtype=self.input_dtypes.get("backbone_attention_mask", backbone_attention_mask.dtype))
                if not backbone_attention_mask.is_contiguous():
                    backbone_attention_mask = backbone_attention_mask.contiguous()

            # Set input shapes only once — constant at batch=1 with fixed inputs.
            # Tensor addresses must be set every call (different tensor pointers).
            if not self._shapes_configured:
                self.context.set_input_shape("sa_embs", sa_embs.shape)
                self.context.set_input_shape("vl_embs", vl_embs.shape)
                self.context.set_input_shape("timestep", timestep.shape)
                if image_mask is not None:
                    self.context.set_input_shape("image_mask", image_mask.shape)
                if backbone_attention_mask is not None:
                    self.context.set_input_shape("backbone_attention_mask", backbone_attention_mask.shape)
                self._shapes_configured = True

            self.context.set_tensor_address("sa_embs", sa_embs.data_ptr())
            self.context.set_tensor_address("vl_embs", vl_embs.data_ptr())
            self.context.set_tensor_address("timestep", timestep.data_ptr())
            if image_mask is not None:
                self.context.set_tensor_address("image_mask", image_mask.data_ptr())
            if backbone_attention_mask is not None:
                self.context.set_tensor_address(
                    "backbone_attention_mask", backbone_attention_mask.data_ptr()
                )

            # Reuse output buffer across diffusion steps (avoids alloc/free churn)
            if self._output_buf is None:
                output_shape = tuple(self.context.get_tensor_shape("output"))
                self._output_buf = torch.empty(
                    output_shape, dtype=self.engine_output_dtype, device=f"cuda:{self.device}"
                )
            self.context.set_tensor_address("output", self._output_buf.data_ptr())

            # Execute on dedicated stream
            success = self.context.execute_async_v3(self.stream.cuda_stream)
            if not success:
                raise RuntimeError("TensorRT inference failed")

            # Convert FP16 -> BF16 if needed (action decoder requires BF16)
            if self.convert_to_bf16:
                output = self._output_buf.to(torch.bfloat16)
            else:
                # Safe to return buffer directly (no clone needed): action_decoder
                # creates new tensors via torch.bmm before the next TRT call
                # overwrites _output_buf in the next denoising step.
                output = self._output_buf

        # Record event on TRT stream and make default stream wait for it
        event = self.stream.record_event()
        torch.cuda.current_stream().wait_event(event)

        return output


def replace_dit_with_tensorrt(policy, trt_engine_path: str, device: int = 0, preloaded_trt: TensorRTDiTWrapper | None = None):
    """Replace the DiT forward method with TensorRT inference.

    Args:
        policy: The Gr00tPolicy instance
        trt_engine_path: Path to the TensorRT engine file
        device: CUDA device index
        preloaded_trt: Optional pre-loaded TensorRT wrapper (for memory-constrained systems)
    """
    # Free the PyTorch DiT weights to reclaim memory before TRT load
    if hasattr(policy.model.action_head, 'model') and policy.model.action_head.model is not None:
        if torch.cuda.is_available():
            mem_before = torch.cuda.memory_allocated() / 1024**3
            logging.info(f"GPU memory before DiT deletion: {mem_before:.2f} GB")

        del policy.model.action_head.model
        gc.collect()
        torch.cuda.empty_cache()
        if torch.cuda.is_available():
            torch.cuda.synchronize()
            mem_after = torch.cuda.memory_allocated() / 1024**3
            logging.info(f"GPU memory after DiT deletion: {mem_after:.2f} GB (freed {mem_before - mem_after:.2f} GB)")

    # Use preloaded TRT engine if provided, otherwise load now
    if preloaded_trt is not None:
        trt_dit = preloaded_trt
        logging.info("Using pre-loaded TensorRT engine")
    else:
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

        if return_all_hidden_states:
            raise RuntimeError("TensorRT only returns the final output. Check inference config")
        return output

    # Wrap in a proper class so policy.model.action_head.model is a callable object
    class TRTModel:
        def __init__(self, forward_fn):
            self._forward = forward_fn

        def forward(self, *args, **kwargs):
            return self._forward(*args, **kwargs)

        def __call__(self, *args, **kwargs):
            return self._forward(*args, **kwargs)

    policy.model.action_head.model = TRTModel(trt_forward)
    logging.info("DiT replaced with TensorRT engine")


###############################################################################
# JpegPolicyWrapper
###############################################################################


class JpegPolicyWrapper(BasePolicy):
    """Wraps any BasePolicy to accept JPEG-encoded observations over network.

    Translates from the Jetson client wire format:
        {left_wide: jpeg_bytes, ..., state: [22D flat], language: str}
    To Gr00tPolicy native format:
        {video: {key: (1,1,H,W,3) uint8}, state: {key: (1,1,D) float32},
         language: {key: [[str]]}}

    Returns flat 22D action arrays (reassembled from body-part dicts) so the
    existing client can consume them without changes.
    """

    def __init__(self, policy: BasePolicy, language_key: str):
        super().__init__(strict=False)
        self.policy = policy
        self.language_key = language_key
        self._request_count = 0

    def _decode_images(self, observation: dict) -> dict[str, np.ndarray]:
        """Decode JPEG bytes or raw RGB arrays into (1,1,H,W,3) uint8."""
        video_dict = {}

        # Check for raw images first (used by open-loop eval)
        raw_images = observation.get('raw_images', {})
        if raw_images:
            for key, meta in raw_images.items():
                arr = np.frombuffer(meta['data'], dtype=np.uint8)
                img = arr.reshape(meta['shape'])
                video_dict[key] = img[np.newaxis, np.newaxis, ...]
            return video_dict

        # JPEG decode path (normal inference)
        image_keys = ['left_wide', 'right_wide', 'left_center', 'right_center']
        for key in image_keys:
            if key not in observation:
                continue
            jpeg_bytes = observation[key]
            img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            video_dict[key] = img[np.newaxis, np.newaxis, ...]  # already uint8 from cv2

        return video_dict

    def _split_state(self, flat_state: list | np.ndarray) -> dict[str, np.ndarray]:
        """Split flat 22D state into body-part dicts with shape (1,1,D)."""
        state = np.array(flat_state, dtype=np.float32)
        return {
            name: state[np.newaxis, np.newaxis, start:end]
            for name, start, end in STATE_PARTS
        }

    def _reassemble_actions(self, action_dict: dict[str, np.ndarray]) -> list:
        """Reassemble body-part action dicts into flat 22D action arrays.

        Returns list of lists for msgpack serialization.
        """
        # Get temporal dimension from first key
        first_key = next(iter(action_dict))
        temporal_dim = action_dict[first_key].shape[1]

        actions = np.zeros((temporal_dim, 22), dtype=np.float32)
        for name, start, end in STATE_PARTS:
            if name in action_dict:
                # action_dict[key] shape is (B, T, D), squeeze batch
                actions[:, start:end] = action_dict[name][0]

        return actions.tolist()

    def _get_action(
        self, observation: dict[str, Any], options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        """Translate wire format → native format → run inference → flat 22D."""
        # Decode images
        video_dict = self._decode_images(observation)

        # Split state
        state_dict = self._split_state(observation.get('state', []))

        # Format language
        language = observation.get('language', '')
        language_dict = {self.language_key: [[language]]}

        # Build native observation
        native_obs = {
            'video': video_dict,
            'state': state_dict,
            'language': language_dict,
        }

        # Log periodically
        if self._request_count % 10 == 0:
            state = np.array(observation.get('state', []), dtype=np.float32)
            if len(state) >= 22:
                msg = (
                    f"[input] head=({state[19]:.3f},{state[20]:.3f},{state[21]:.3f}) "
                    f"r_arm=({state[13]:.3f},{state[14]:.3f},{state[15]:.3f},{state[16]:.3f},{state[17]:.3f}) "
                    f"r_grip={state[18]:.3f} back={state[6]:.3f} fwd={state[0]:.3f}"
                )
                logging.info(msg)
                print(msg, flush=True)

        # Run inference on the wrapped policy
        action_dict, info = self.policy.get_action(native_obs, options)

        # Reassemble to flat 22D for the client
        flat_actions = self._reassemble_actions(action_dict)

        # Log periodically
        if self._request_count % 10 == 0 and flat_actions:
            a = flat_actions[0]
            msg = (
                f"[output] head=({a[19]:.3f},{a[20]:.3f},{a[21]:.3f}) "
                f"r_arm=({a[13]:.3f},{a[14]:.3f},{a[15]:.3f},{a[16]:.3f},{a[17]:.3f}) "
                f"r_grip={a[18]:.3f} back={a[6]:.3f} fwd={a[0]:.3f}"
            )
            logging.info(msg)
            print(msg, flush=True)

        self._request_count += 1

        # Return as a dict with 'actions' key — PolicyServer serializes via MsgSerializer.
        # The client expects the response to contain an 'actions' key with flat 22D arrays.
        # However, PolicyServer returns whatever _get_action returns as a tuple (action, info).
        # We embed the flat actions in the action dict so the client can extract them.
        return {'actions': flat_actions}, info

    def check_observation(self, observation: dict[str, Any]) -> None:
        pass  # Validation done by inner policy

    def check_action(self, action: dict[str, Any]) -> None:
        pass  # We return custom format

    def reset(self, options: dict[str, Any] | None = None) -> dict[str, Any]:
        return self.policy.reset(options)

    def get_modality_config(self):
        if hasattr(self.policy, 'get_modality_config'):
            return self.policy.get_modality_config()
        return {}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='GR00T N1.6 Inference Server',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
modes:
  1. Production   --checkpoint is set (default)
  2. Replay       --dataset-path is set (no GPU needed)

examples:
  %(prog)s --checkpoint /home/alfie/cando --host 0.0.0.0
  %(prog)s --dataset-path /path/to/dataset --episode-index 3
  %(prog)s --checkpoint /home/alfie/cando --torch-compile
""",
    )

    parser.add_argument('--checkpoint', '-c', default='',
                        help='Path to GR00T model checkpoint')
    parser.add_argument('--embodiment', '-e', default='new_embodiment',
                        help='Embodiment tag (default: %(default)s)')
    parser.add_argument('--device', '-d', default='cuda:0',
                        help='CUDA device (default: %(default)s)')
    parser.add_argument('--host', default='0.0.0.0',
                        help='Bind host (default: %(default)s)')
    parser.add_argument('--port', '-p', type=int, default=5555,
                        help='Bind port (default: %(default)s)')
    parser.add_argument('--dataset-path', default='',
                        help='LeRobot dataset path (enables replay mode)')
    parser.add_argument('--episode-index', type=int, default=0,
                        help='Episode index for replay (default: %(default)s)')
    parser.add_argument('--execution-horizon', type=int, default=16,
                        help='Actions per replay step (default: %(default)s)')
    parser.add_argument('--denoising-steps', type=int, default=0,
                        help='Override denoising steps (0=model default)')
    parser.add_argument('--trt-engine-path', default='',
                        help='Path to TensorRT engine file (.trt) for DiT. Enables TRT mode.')
    parser.add_argument('--compile-backbone', action='store_true',
                        help='Apply torch.compile to backbone for kernel fusion (~10%% speedup)')
    parser.add_argument('--compile-backbone-mode', default='default',
                        help='torch.compile mode (default: %(default)s). '
                             'Only "default" works on Orin.')
    parser.add_argument('--compile-action-head', action='store_true',
                        help='Apply torch.compile to action encoder/decoder MLPs')
    parser.add_argument('--torch-compile', action='store_true',
                        help='Apply torch.compile to DiT (PyTorch mode only)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Verbose logging')

    return parser.parse_args()


def main():
    args = parse_args()

    # Setup logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
    )
    for name in ('matplotlib', 'PIL', 'httpcore', 'httpx', 'urllib3', 'asyncio'):
        logging.getLogger(name).setLevel(logging.WARNING)

    logger = logging.getLogger('groot_server')

    # Parse embodiment tag
    try:
        embodiment_tag = EmbodimentTag(args.embodiment)
    except ValueError:
        logger.warning(f'Unknown embodiment: {args.embodiment}, using NEW_EMBODIMENT')
        embodiment_tag = EmbodimentTag.NEW_EMBODIMENT

    # Determine language key from modality config
    # For alfiebot: 'annotation.human.action.task_description'
    language_key = 'annotation.human.action.task_description'

    # Create policy
    if args.dataset_path:
        # Replay mode
        logger.info(f'Replay mode: {args.dataset_path} episode {args.episode_index}')

        from gr00t.configs.data.embodiment_configs import MODALITY_CONFIGS
        modality_configs = MODALITY_CONFIGS[embodiment_tag.value]

        # Get language key from modality config
        language_key = modality_configs['language'].modality_keys[0]

        policy = ReplayPolicy(
            dataset_path=args.dataset_path,
            modality_configs=modality_configs,
            execution_horizon=args.execution_horizon,
            strict=False,
        )
        logger.info(f'ReplayPolicy loaded: {policy.episode_length} steps, '
                     f'execution_horizon={args.execution_horizon}')

    elif args.checkpoint:
        # Production inference
        checkpoint = Path(args.checkpoint)
        if not checkpoint.exists():
            logger.error(f'Checkpoint not found: {checkpoint}')
            sys.exit(1)

        device_idx = int(args.device.split(':')[-1]) if ':' in args.device else 0

        if args.trt_engine_path and torch.cuda.is_available():
            # TRT-first loading order: load TRT engine while GPU is empty to avoid
            # memory fragmentation on Jetson unified memory.
            logger.info('TensorRT mode: loading TRT engine first while GPU is empty...')

            trt_dit = TensorRTDiTWrapper(args.trt_engine_path, device=device_idx)
            gc.collect()
            torch.cuda.empty_cache()
            mem_used = torch.cuda.memory_allocated() / 1024**3
            logger.info(f'GPU memory after TRT engine load: {mem_used:.2f} GB')

            # Load PyTorch model WITHOUT DiT weights (skip_dit=True saves ~2GB)
            logger.info(f'Loading Gr00tPolicy from {checkpoint} (skip_dit=True)...')
            policy = Gr00tPolicy(
                embodiment_tag=embodiment_tag,
                model_path=str(checkpoint),
                device=args.device,
                skip_dit=True,
                strict=False,
            )
            gc.collect()
            torch.cuda.empty_cache()
            mem_used = torch.cuda.memory_allocated() / 1024**3
            logger.info(f'GPU memory after PyTorch model load: {mem_used:.2f} GB')

            # Wire up pre-loaded TRT engine
            replace_dit_with_tensorrt(
                policy, args.trt_engine_path, device=device_idx,
                preloaded_trt=trt_dit,
            )
            gc.collect()
            torch.cuda.empty_cache()
            mem_used = torch.cuda.memory_allocated() / 1024**3
            mem_reserved = torch.cuda.memory_reserved() / 1024**3
            logger.info(
                f'GPU memory after TRT wiring: allocated={mem_used:.2f} GB, '
                f'reserved={mem_reserved:.2f} GB'
            )
        else:
            # PyTorch-only mode (no TRT)
            logger.info(f'Loading Gr00tPolicy from {checkpoint}...')
            policy = Gr00tPolicy(
                embodiment_tag=embodiment_tag,
                model_path=str(checkpoint),
                device=args.device,
                strict=False,
            )

        # Get language key from loaded policy
        language_key = policy.modality_configs['language'].modality_keys[0]

        # Override denoising steps
        if args.denoising_steps > 0:
            model_denoise = policy.model.action_head.num_inference_timesteps
            policy.model.action_head.num_inference_timesteps = args.denoising_steps
            logger.info(f'Denoising steps: {model_denoise} -> {args.denoising_steps}')

        # torch.compile on backbone for kernel fusion
        if args.compile_backbone:
            logger.info(
                f'Compiling backbone with torch.compile(mode={args.compile_backbone_mode!r})...'
            )
            policy.model.backbone.forward = torch.compile(
                policy.model.backbone.forward,
                mode=args.compile_backbone_mode,
            )
            logger.info('Backbone compiled (will warmup on first inference)')

        # torch.compile on action encoder/decoder MLPs
        if args.compile_action_head:
            logger.info('Compiling action encoder/decoder with torch.compile...')
            ah = policy.model.action_head
            if hasattr(ah, 'action_encoder'):
                ah.action_encoder.forward = torch.compile(ah.action_encoder.forward)
            if hasattr(ah, 'action_decoder'):
                ah.action_decoder.forward = torch.compile(ah.action_decoder.forward)
            logger.info('Action head compiled (will warmup on first inference)')

        # Apply torch.compile to DiT (PyTorch mode only, not with TRT)
        if args.torch_compile and not args.trt_engine_path:
            logger.info('Applying torch.compile to DiT (first inference will be slow)...')
            policy.model.action_head.model.forward = torch.compile(
                policy.model.action_head.model.forward, mode="max-autotune"
            )

        # Runtime CUDA optimizations
        if torch.cuda.is_available():
            torch.backends.cudnn.benchmark = True
            # Cap PyTorch's CUDA cache at 60% of total memory to leave headroom for
            # TensorRT internal buffers, numpy, OS, and other allocations on unified memory
            torch.cuda.set_per_process_memory_fraction(0.6)
            logger.info('CUDA optimizations: cudnn.benchmark=True, memory cap=60%')

        logger.info(f'Model loaded (embodiment={embodiment_tag.value}, '
                     f'language_key={language_key})')
    else:
        logger.error('Either --checkpoint or --dataset-path must be provided')
        sys.exit(1)

    # Wrap with JPEG translation layer
    wrapped = JpegPolicyWrapper(policy, language_key=language_key)

    # Warmup inference to trigger torch.compile and other first-call overhead
    if not args.dataset_path:
        import time
        logger.info('Running warmup inference (triggers torch.compile if enabled)...')
        black_img = np.zeros((240, 320, 3), dtype=np.uint8)
        _, jpeg_bytes = cv2.imencode('.jpg', black_img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        jpeg_bytes = jpeg_bytes.tobytes()
        warmup_obs = {
            'left_wide': jpeg_bytes,
            'right_wide': jpeg_bytes,
            'left_center': jpeg_bytes,
            'right_center': jpeg_bytes,
            'state': [0.0] * 22,
            'language': '',
        }
        t0 = time.monotonic()
        wrapped._get_action(warmup_obs)
        elapsed = time.monotonic() - t0
        logger.info(f'Warmup complete in {elapsed:.1f}s')

    # Create server using NVIDIA's PolicyServer (sets up socket + endpoints)
    logger.info(f'Starting PolicyServer on {args.host}:{args.port}')
    server = PolicyServer(
        policy=wrapped,
        host=args.host,
        port=args.port,
    )

    # Run our own loop instead of server.run() so we can handle Ctrl-C.
    # PolicyServer.run() blocks on socket.recv() with no timeout, making
    # it impossible to shut down cleanly via signal.
    server.socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1s timeout for clean shutdown
    running = True

    def signal_handler(signum, frame):
        nonlocal running
        logger.info(f'Received signal {signum}, shutting down...')
        running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    from gr00t.policy.server_client import MsgSerializer

    addr = server.socket.getsockopt_string(zmq.LAST_ENDPOINT)
    logger.info(f'Server is ready and listening on {addr}')

    while running:
        try:
            message = server.socket.recv()
        except zmq.Again:
            continue  # Timeout — check running flag and loop
        except zmq.ZMQError:
            if not running:
                break
            raise

        try:
            request = MsgSerializer.from_bytes(message)
            endpoint = request.get("endpoint", "get_action")

            if endpoint not in server._endpoints:
                raise ValueError(f"Unknown endpoint: {endpoint}")

            handler = server._endpoints[endpoint]
            result = (
                handler.handler(**request.get("data", {}))
                if handler.requires_input
                else handler.handler()
            )
            server.socket.send(MsgSerializer.to_bytes(result))
        except Exception as e:
            logger.error(f'Error processing request: {e}')
            import traceback
            traceback.print_exc()
            server.socket.send(MsgSerializer.to_bytes({"error": str(e)}))

    server.socket.close()
    server.context.term()
    logger.info('Server stopped.')


if __name__ == '__main__':
    main()
