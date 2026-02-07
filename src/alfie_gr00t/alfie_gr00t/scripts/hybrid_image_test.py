#!/usr/bin/env python3
"""Hybrid image test: send live camera images with GT state through inference server.

This script isolates whether the visual domain gap alone causes model failure by:
1. Loading saved live debug images from /tmp/groot_debug_images/
2. Loading GT state from training episode 0
3. Sending them together through the ZMQ inference server
4. Comparing model predictions vs what open-loop eval produced with GT images

If predictions are garbage → images alone are the problem
If predictions are reasonable → something else in the live client pipeline is wrong

Usage:
    python hybrid_image_test.py --host 192.168.50.108 --port 5555
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd

# ROS2 imports for rosbag reading
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

try:
    import zstandard as zstd
    HAS_ZSTD = True
except ImportError:
    HAS_ZSTD = False

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from alfie_gr00t.core.zmq_client import ZMQClient, build_server_address

CAMERA_NAMES = ['left_wide', 'right_wide', 'left_center', 'right_center']
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
JPEG_QUALITY = 95  # Match live pipeline

# Rosbag topic to camera name mapping (from rosbag_to_groot.py)
CAMERA_TOPICS = {
    '/alfie/stereo_camera/left_wide/image_raw/compressed': 'left_wide',
    '/alfie/stereo_camera/right_wide/image_raw/compressed': 'right_wide',
    '/alfie/stereo_camera/left_center/image_raw/compressed': 'left_center',
    '/alfie/stereo_camera/right_center/image_raw/compressed': 'right_center',
}


def load_live_images(debug_dir: str, frame_idx: int = 0) -> dict[str, bytes]:
    """Load saved live debug images and compress to JPEG."""
    images = {}
    for cam in CAMERA_NAMES:
        path = Path(debug_dir) / f'live_{frame_idx}_{cam}.png'
        if not path.exists():
            print(f'WARNING: Missing {path}')
            continue
        img = cv2.imread(str(path))  # BGR
        # Convert to RGB then back to BGR for JPEG (matching live pipeline)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        _, encoded = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        images[cam] = encoded.tobytes()
        print(f'  Loaded {cam}: {img.shape}')
    return images


def load_training_images(debug_dir: str) -> dict[str, bytes]:
    """Load saved training debug images and compress to JPEG."""
    images = {}
    for cam in CAMERA_NAMES:
        path = Path(debug_dir) / f'train_0_{cam}.png'
        if not path.exists():
            print(f'WARNING: Missing {path}')
            continue
        img = cv2.imread(str(path))  # BGR
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        _, encoded = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])  # Match eval quality
        images[cam] = encoded.tobytes()
        print(f'  Loaded {cam}: {img.shape}')
    return images


def load_rosbag_images(mcap_path: str) -> dict[str, bytes]:
    """Extract first frame for all 4 cameras from a rosbag (pre-H.264).

    Applies the same processing as rosbag_to_groot.py:
      imdecode(BGR) -> resize(320,240) -> cvtColor(RGB)
    Then JPEG-compresses at quality 95 (matching live pipeline).

    This gives us images that went through the same JPEG decode path as live,
    but from the exact same recording session that produced the training data.
    """
    mcap_path = Path(mcap_path)

    # Handle zstd decompression
    if mcap_path.suffix == '.zstd':
        decompressed = mcap_path.with_suffix('')
        if not decompressed.exists():
            if not HAS_ZSTD:
                raise RuntimeError('zstandard package needed for .mcap.zstd files')
            print(f'  Decompressing {mcap_path.name}...')
            dctx = zstd.ZstdDecompressor()
            with open(mcap_path, 'rb') as ifh:
                with open(decompressed, 'wb') as ofh:
                    dctx.copy_stream(ifh, ofh)
        mcap_path = decompressed

    storage_options = StorageOptions(uri=str(mcap_path), storage_id='mcap')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Collect first image from each camera
    found = {}
    while reader.has_next() and len(found) < 4:
        topic, data, timestamp_ns = reader.read_next()
        if topic in CAMERA_TOPICS:
            cam_name = CAMERA_TOPICS[topic]
            if cam_name in found:
                continue
            msg = deserialize_message(data, CompressedImage)
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # BGR
            if img is not None:
                img = cv2.resize(img, (IMAGE_WIDTH, IMAGE_HEIGHT))
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                # JPEG compress matching live pipeline (quality 95)
                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                _, encoded = cv2.imencode('.jpg', img_bgr,
                                         [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                found[cam_name] = encoded.tobytes()
                print(f'  Loaded {cam_name}: {img.shape} ts={timestamp_ns}')

    if len(found) < 4:
        missing = set(CAMERA_NAMES) - set(found.keys())
        print(f'  WARNING: Missing cameras: {missing}')

    return found


def load_gt_state(dataset_path: str, episode_index: int = 0) -> np.ndarray:
    """Load GT state from training episode."""
    # LeRobot v2 format: data/chunk-000/episode_NNNNNN.parquet
    parquet_path = Path(dataset_path) / 'data' / 'chunk-000' / f'episode_{episode_index:06d}.parquet'
    if not parquet_path.exists():
        raise RuntimeError(f'Episode {episode_index} not found at {parquet_path}')

    df = pd.read_parquet(parquet_path)
    # State is stored as a single list/array column 'observation.state'
    state = np.array(df.iloc[0]['observation.state'], dtype=np.float32)
    print(f'  GT state (22D): head=({state[19]:.3f},{state[20]:.3f},{state[21]:.3f})')
    print(f'    r_arm=({state[13]:.3f},{state[14]:.3f},{state[15]:.3f},{state[16]:.3f},{state[17]:.3f})')
    print(f'    r_grip={state[18]:.3f} back={state[6]:.3f}')
    return state


def format_action(a: np.ndarray, label: str):
    """Pretty-print an action vector."""
    print(f'  {label}:')
    print(f'    head=({a[19]:.3f}, {a[20]:.3f}, {a[21]:.3f})')
    print(f'    r_arm=({a[13]:.3f}, {a[14]:.3f}, {a[15]:.3f}, {a[16]:.3f}, {a[17]:.3f})')
    print(f'    r_grip={a[18]:.3f}  back={a[6]:.3f}  fwd={a[0]:.3f}')


def main():
    parser = argparse.ArgumentParser(description='Hybrid image test')
    parser.add_argument('--host', default='192.168.50.108')
    parser.add_argument('--port', type=int, default=5555)
    parser.add_argument('--dataset-path', default='/home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge')
    parser.add_argument('--debug-dir', default='/tmp/groot_debug_images')
    parser.add_argument('--task', default='find the can and pick it up')
    parser.add_argument('--episode', type=int, default=0)
    parser.add_argument('--rosbag',
                        default='/home/alfie/alfiebot_ws/data/demonstrations/demo_20260102_195919/demo_20260102_195919_0.mcap',
                        help='Path to .mcap or .mcap.zstd rosbag for episode 0 source demo')
    args = parser.parse_args()

    server_address = build_server_address(transport='tcp', host=args.host, port=args.port)
    print(f'Server: {server_address}')
    print()

    # Load GT state
    print('Loading GT state from training episode...')
    gt_state = load_gt_state(args.dataset_path, args.episode)
    print()

    # Connect to server
    client = ZMQClient(
        server_address=server_address,
        timeout_ms=5000,
        logger=lambda msg: print(f'  [zmq] {msg}'),
    )
    client.connect()
    if not client.ping():
        print('ERROR: Server not reachable')
        return

    print()
    print('=' * 60)
    print('TEST 1: GT training images + GT state (baseline)')
    print('=' * 60)
    print('Loading training images...')
    train_images = load_training_images(args.debug_dir)
    if len(train_images) == 4:
        response = client.send_observation(
            images=train_images,
            state=gt_state,
            language=args.task,
        )
        if response and 'actions' in response:
            actions = np.array(response['actions'])
            print(f'  Got {actions.shape[0]} actions')
            format_action(actions[0], 'Action[0]')
            format_action(actions[7], 'Action[7]')
            format_action(actions[15], 'Action[15]')
        else:
            print(f'  ERROR: {response}')
    else:
        print('  SKIPPED: Not all 4 training images available')

    print()
    print('=' * 60)
    print('TEST 2: LIVE images + GT state (hybrid test)')
    print('=' * 60)
    print('Loading live images...')
    live_images = load_live_images(args.debug_dir, frame_idx=0)
    if len(live_images) == 4:
        response = client.send_observation(
            images=live_images,
            state=gt_state,
            language=args.task,
        )
        if response and 'actions' in response:
            actions = np.array(response['actions'])
            print(f'  Got {actions.shape[0]} actions')
            format_action(actions[0], 'Action[0]')
            format_action(actions[7], 'Action[7]')
            format_action(actions[15], 'Action[15]')
        else:
            print(f'  ERROR: {response}')
    else:
        print('  SKIPPED: Not all 4 live images available')

    # Also test with multiple live frames to see if predictions are consistent
    print()
    print('=' * 60)
    print('TEST 3: LIVE images (frames 0-4) + GT state (consistency)')
    print('=' * 60)
    for frame_idx in range(5):
        live_imgs = load_live_images(args.debug_dir, frame_idx)
        if len(live_imgs) < 4:
            print(f'  Frame {frame_idx}: SKIPPED')
            continue
        response = client.send_observation(
            images=live_imgs,
            state=gt_state,
            language=args.task,
        )
        if response and 'actions' in response:
            a = np.array(response['actions'])[0]
            print(f'  Frame {frame_idx}: head=({a[19]:.3f},{a[20]:.3f},{a[21]:.3f}) '
                  f'r_grip={a[18]:.3f} fwd={a[0]:.3f}')
        else:
            print(f'  Frame {frame_idx}: ERROR')

    # Test 4: Most visually similar training episode images + GT state
    print()
    print('=' * 60)
    print('TEST 4: Similar training ep (ep_031) images + GT state')
    print('=' * 60)
    similar_images = {}
    for cam in CAMERA_NAMES:
        path = Path(args.debug_dir) / f'similar_ep031_{cam}.png'
        if path.exists():
            img = cv2.imread(str(path))  # BGR (from MP4, already BGR in cv2)
            # MP4 stores RGB, cv2.VideoCapture reads as BGR — same as training path
            img_bgr = img  # already BGR from cv2.imread
            _, encoded = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            similar_images[cam] = encoded.tobytes()
            print(f'  Loaded {cam}: {img.shape}')
    if len(similar_images) == 4:
        response = client.send_observation(
            images=similar_images,
            state=gt_state,
            language=args.task,
        )
        if response and 'actions' in response:
            actions = np.array(response['actions'])
            print(f'  Got {actions.shape[0]} actions')
            format_action(actions[0], 'Action[0]')
            format_action(actions[7], 'Action[7]')
            format_action(actions[15], 'Action[15]')
        else:
            print(f'  ERROR: {response}')
    else:
        print('  SKIPPED: Not all 4 similar episode images available')

    # Test 5: Live images after H.264 roundtrip + GT state
    # If training images always go through H.264 encode/decode (yuv420p),
    # the model may rely on those color subsampling artifacts.
    print()
    print('=' * 60)
    print('TEST 5: H.264-conditioned LIVE images + GT state')
    print('=' * 60)
    h264_images = {}
    for cam in CAMERA_NAMES:
        path = Path(args.debug_dir) / f'live_h264_{cam}.png'
        if path.exists():
            img = cv2.imread(str(path))  # BGR
            _, encoded = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            h264_images[cam] = encoded.tobytes()
            print(f'  Loaded {cam}: {img.shape}')
    if len(h264_images) == 4:
        response = client.send_observation(
            images=h264_images,
            state=gt_state,
            language=args.task,
        )
        if response and 'actions' in response:
            actions = np.array(response['actions'])
            print(f'  Got {actions.shape[0]} actions')
            format_action(actions[0], 'Action[0]')
            format_action(actions[7], 'Action[7]')
            format_action(actions[15], 'Action[15]')
        else:
            print(f'  ERROR: {response}')
    else:
        print('  SKIPPED: Not all 4 h264-conditioned images available')

    # Test 6: Blended images (train→live) to find sensitivity threshold
    print()
    print('=' * 60)
    print('TEST 6: Blended images (0%=train, 100%=live) + GT state')
    print('=' * 60)
    for blend_pct in [0, 25, 50, 75, 100]:
        blend_images = {}
        for cam in CAMERA_NAMES:
            path = Path(args.debug_dir) / f'blend_{blend_pct}_{cam}.png'
            if path.exists():
                img = cv2.imread(str(path))
                _, encoded = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                blend_images[cam] = encoded.tobytes()
        if len(blend_images) == 4:
            response = client.send_observation(
                images=blend_images,
                state=gt_state,
                language=args.task,
            )
            if response and 'actions' in response:
                a = np.array(response['actions'])
                # Show head_pitch trajectory across the 16-action horizon
                pitches = [f'{a[i,20]:.2f}' for i in [0, 4, 8, 12, 15]]
                print(f'  {blend_pct:3d}% live: head_pitch=[{", ".join(pitches)}]  '
                      f'r_grip[0]={a[0,18]:.3f}')

    # Test 7: Rosbag raw images (pre-H.264) + GT state
    # These images went through JPEG decode + resize but NOT H.264 encode/decode.
    # If this fails like live → H.264 is confirmed as the cause.
    # If this works like training → something else changed between capture and live.
    print()
    print('=' * 60)
    print('TEST 7: Rosbag raw images (pre-H.264) + GT state')
    print('=' * 60)
    rosbag_path = Path(args.rosbag)
    if not rosbag_path.exists():
        # Try .zstd variant
        zstd_path = Path(str(rosbag_path) + '.zstd')
        if zstd_path.exists():
            rosbag_path = zstd_path
        else:
            print(f'  SKIPPED: Rosbag not found: {args.rosbag}')
            rosbag_path = None

    if rosbag_path is not None:
        print(f'  Loading from: {rosbag_path.name}')
        rosbag_images = load_rosbag_images(str(rosbag_path))
        if len(rosbag_images) == 4:
            response = client.send_observation(
                images=rosbag_images,
                state=gt_state,
                language=args.task,
            )
            if response and 'actions' in response:
                actions = np.array(response['actions'])
                print(f'  Got {actions.shape[0]} actions')
                format_action(actions[0], 'Action[0]')
                format_action(actions[7], 'Action[7]')
                format_action(actions[15], 'Action[15]')
            else:
                print(f'  ERROR: {response}')
        else:
            print('  SKIPPED: Not all 4 camera images found in rosbag')

    client.close()
    print()
    print('Done.')


if __name__ == '__main__':
    main()
