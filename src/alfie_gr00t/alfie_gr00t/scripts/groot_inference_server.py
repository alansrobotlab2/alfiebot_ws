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
import logging
import signal
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
import zmq

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
            video_dict[key] = img[np.newaxis, np.newaxis, ...].astype(np.uint8)

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
    parser.add_argument('--torch-compile', action='store_true',
                        help='Apply torch.compile to DiT for speedup')
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
            policy.model.action_head.num_inference_timesteps = args.denoising_steps
            logger.info(f'Denoising steps: {args.denoising_steps}')

        # Apply torch.compile for inference speedup
        if args.torch_compile:
            logger.info('Applying torch.compile to DiT (first inference will be slow)...')
            policy.model.action_head.model.forward = torch.compile(
                policy.model.action_head.model.forward, mode="max-autotune"
            )

        logger.info(f'Model loaded (embodiment={embodiment_tag.value}, '
                     f'language_key={language_key})')
    else:
        logger.error('Either --checkpoint or --dataset-path must be provided')
        sys.exit(1)

    # Wrap with JPEG translation layer
    wrapped = JpegPolicyWrapper(policy, language_key=language_key)

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
