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
import io
import json
import sys
import time
from pathlib import Path

import av
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
IMAGE_HEIGHT = 240  # Must match training data MP4s (320x240)
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


def extract_mp4_frame0(dataset_path: str, episode_index: int = 0) -> dict[str, bytes]:
    """Extract first frame from training MP4 videos (post-H.264).

    These are the exact images the model was trained on — they've been through
    the full rosbag_to_groot.py pipeline including H.264 yuv420p encode/decode.
    """
    videos_dir = Path(dataset_path) / 'videos' / 'chunk-000'
    images = {}
    for cam in CAMERA_NAMES:
        mp4_path = videos_dir / f'observation.images.{cam}' / f'episode_{episode_index:06d}.mp4'
        if not mp4_path.exists():
            print(f'  WARNING: Missing {mp4_path}')
            continue
        container = av.open(str(mp4_path))
        for frame in container.decode(video=0):
            img_rgb = frame.to_ndarray(format='rgb24')
            # Convert to BGR for JPEG encoding (server decodes JPEG → BGR → RGB)
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            _, encoded = cv2.imencode('.jpg', img_bgr,
                                     [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            images[cam] = encoded.tobytes()
            print(f'  Extracted {cam}: {img_rgb.shape} from MP4')
            break  # only first frame
        container.close()
    return images


def h264_condition_frame(img_rgb: np.ndarray) -> np.ndarray:
    """Apply H.264 encode/decode round-trip to match training data pipeline.

    Replicates the exact encoding used in rosbag_to_groot.py:
    libx264, yuv420p, CRF=23, preset=fast.

    Args:
        img_rgb: RGB numpy array (H, W, 3), uint8.

    Returns:
        H.264-conditioned RGB numpy array, same shape.
    """
    h, w = img_rgb.shape[:2]
    buf = io.BytesIO()

    # Encode: RGB → H.264 yuv420p
    output = av.open(buf, mode='w', format='mp4')
    stream = output.add_stream('libx264', rate=1)
    stream.width = w
    stream.height = h
    stream.pix_fmt = 'yuv420p'
    stream.options = {'crf': '23', 'preset': 'fast'}

    frame = av.VideoFrame.from_ndarray(img_rgb, format='rgb24')
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

    container.close()
    return img_rgb


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
    print('TEST 1a: GT training images from MP4 (post-H.264) + GT state')
    print('=' * 60)
    print('Extracting frame 0 from training MP4 videos...')
    mp4_images = extract_mp4_frame0(args.dataset_path, args.episode)
    if len(mp4_images) == 4:
        response = client.send_observation(
            images=mp4_images,
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
        print('  SKIPPED: Not all 4 MP4 videos found')

    print()
    print('=' * 60)
    print('TEST 1b: GT training images from rosbag (pre-H.264) + GT state')
    print('=' * 60)
    rosbag_path = Path(args.rosbag)
    if not rosbag_path.exists():
        zstd_path = Path(str(rosbag_path) + '.zstd')
        if zstd_path.exists():
            rosbag_path = zstd_path
        else:
            rosbag_path = None
    if rosbag_path is not None:
        print(f'  Extracting frame 0 from rosbag: {rosbag_path.name}')
        rosbag_raw_images = load_rosbag_images(str(rosbag_path))
        if len(rosbag_raw_images) == 4:
            response = client.send_observation(
                images=rosbag_raw_images,
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
            print('  SKIPPED: Not all 4 cameras found in rosbag')
    else:
        print(f'  SKIPPED: Rosbag not found: {args.rosbag}')

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
    # Training images go through libx264 yuv420p CRF=23 encode/decode in MP4.
    # This test applies the same round-trip to live images on-the-fly.
    print()
    print('=' * 60)
    print('TEST 5: H.264-conditioned LIVE images + GT state (on-the-fly)')
    print('=' * 60)
    h264_images = {}
    for cam in CAMERA_NAMES:
        path = Path(args.debug_dir) / f'live_0_{cam}.png'
        if not path.exists():
            print(f'  WARNING: Missing {path}')
            continue
        img = cv2.imread(str(path))  # BGR
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # Apply H.264 encode/decode round-trip (libx264 yuv420p CRF=23)
        img_conditioned = h264_condition_frame(img_rgb)
        # Convert back to BGR for JPEG encoding
        img_bgr = cv2.cvtColor(img_conditioned, cv2.COLOR_RGB2BGR)
        _, encoded = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        h264_images[cam] = encoded.tobytes()
        print(f'  Conditioned {cam}: {img.shape}')
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
        print('  SKIPPED: Not all 4 live images available for H.264 conditioning')

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

    # Test 7: Consistency test — same input 5 times (detects stochastic noise)
    # If head_yaw varies by more than ±0.05 across identical inputs,
    # something is wrong (TRT bf16, wrong backend, etc.)
    print()
    print('=' * 60)
    print('TEST 7: Consistency — same GT images + GT state sent 5 times')
    print('=' * 60)
    if len(mp4_images) == 4:
        head_yaws = []
        for trial in range(5):
            response = client.send_observation(
                images=mp4_images,
                state=gt_state,
                language=args.task,
            )
            if response and 'actions' in response:
                a = np.array(response['actions'])[0]
                head_yaws.append(a[19])
                print(f'  Trial {trial}: head_yaw={a[19]:+.4f}  head_pitch={a[20]:+.4f}  '
                      f'r_arm[0]={a[13]:+.4f}  fwd={a[0]:+.4f}')
        if len(head_yaws) > 1:
            spread = max(head_yaws) - min(head_yaws)
            print(f'  HEAD_YAW SPREAD: {spread:.4f}  '
                  f'(OK if <0.05, BAD if >0.2 — indicates TRT/backend issue)')
    else:
        print('  SKIPPED: No MP4 images')

    # Test 8: Raw RGB path — match eval pipeline exactly (no JPEG)
    print()
    print('=' * 60)
    print('TEST 8: GT training images as RAW RGB (no JPEG) + GT state')
    print('       (matches groot_open_loop_eval.py send_raw_observation path)')
    print('=' * 60)
    raw_images = {}
    try:
        import torchcodec.decoders
        for cam in CAMERA_NAMES:
            mp4_path = (Path(args.dataset_path) / 'videos' / 'chunk-000'
                        / f'observation.images.{cam}' / f'episode_{args.episode:06d}.mp4')
            if mp4_path.exists():
                decoder = torchcodec.decoders.VideoDecoder(
                    str(mp4_path), device="cpu", dimension_order="NHWC", num_ffmpeg_threads=0
                )
                frame_tensor = decoder.get_frames_at(indices=[0]).data  # (1, H, W, C)
                raw_images[cam] = frame_tensor[0].numpy()  # (H, W, C) uint8 RGB
                print(f'  Loaded {cam}: {raw_images[cam].shape} via torchcodec')
    except ImportError:
        print('  torchcodec not available — falling back to PyAV for MP4 decode')
        for cam in CAMERA_NAMES:
            mp4_path = (Path(args.dataset_path) / 'videos' / 'chunk-000'
                        / f'observation.images.{cam}' / f'episode_{args.episode:06d}.mp4')
            if mp4_path.exists():
                container = av.open(str(mp4_path))
                for frame in container.decode(video=0):
                    raw_images[cam] = frame.to_ndarray(format='rgb24')
                    print(f'  Loaded {cam}: {raw_images[cam].shape} via PyAV')
                    break
                container.close()
    if len(raw_images) == 4:
        response = client.send_raw_observation(
            raw_images=raw_images,
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
        print('  SKIPPED: Not all 4 MP4 videos found for raw path')

    # Test 9: Live images as RAW RGB (no JPEG) + GT state
    # If Test 2 (live JPEG) ≠ Test 1a (GT JPEG) but Test 9 ≈ Test 8,
    # then JPEG compression of live images is the issue.
    # If Test 9 also differs from Test 8, then the images themselves are OOD.
    print()
    print('=' * 60)
    print('TEST 9: LIVE images as RAW RGB (no JPEG) + GT state')
    print('=' * 60)
    raw_live_images = {}
    for cam in CAMERA_NAMES:
        path = Path(args.debug_dir) / f'live_0_{cam}.png'
        if path.exists():
            img_bgr = cv2.imread(str(path))
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            raw_live_images[cam] = img_rgb
            print(f'  Loaded {cam}: {img_rgb.shape}')
    if len(raw_live_images) == 4:
        response = client.send_raw_observation(
            raw_images=raw_live_images,
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

    # Test 10: Consistency with LIVE images (detects if live input causes noise)
    print()
    print('=' * 60)
    print('TEST 10: Consistency — same LIVE images + GT state sent 5 times')
    print('=' * 60)
    if len(raw_live_images) == 4:
        head_yaws = []
        for trial in range(5):
            response = client.send_raw_observation(
                raw_images=raw_live_images,
                state=gt_state,
                language=args.task,
            )
            if response and 'actions' in response:
                a = np.array(response['actions'])[0]
                head_yaws.append(a[19])
                print(f'  Trial {trial}: head_yaw={a[19]:+.4f}  head_pitch={a[20]:+.4f}  '
                      f'r_arm[0]={a[13]:+.4f}  fwd={a[0]:+.4f}')
        if len(head_yaws) > 1:
            spread = max(head_yaws) - min(head_yaws)
            print(f'  HEAD_YAW SPREAD: {spread:.4f}  '
                  f'(OK if <0.05, BAD if >0.2)')
    else:
        print('  SKIPPED: No live images')

    # Test 11: Live state from debug — check if live state values are OOD
    print()
    print('=' * 60)
    print('TEST 11: GT training images + LIVE state (isolate state effect)')
    print('=' * 60)
    # Try to load the live state from the latest observation bridge output
    live_state_path = Path(args.debug_dir) / 'live_state_0.npy'
    if not live_state_path.exists():
        # No saved state — construct from the observation log
        # The user can save it by adding a line to observation_bridge
        print('  SKIPPED: No saved live state at /tmp/groot_debug_images/live_state_0.npy')
        print('  (To enable: save obs.state in observation_bridge debug_save_images)')
    else:
        live_state = np.load(str(live_state_path))
        print(f'  Live state head=({live_state[19]:.3f},{live_state[20]:.3f},{live_state[21]:.3f})')
        print(f'  GT state   head=({gt_state[19]:.3f},{gt_state[20]:.3f},{gt_state[21]:.3f})')
        if len(mp4_images) == 4:
            response = client.send_observation(
                images=mp4_images,
                state=live_state,
                language=args.task,
            )
            if response and 'actions' in response:
                actions = np.array(response['actions'])
                format_action(actions[0], 'Action[0]')
            else:
                print(f'  ERROR: {response}')

    client.close()
    print()
    print('Done.')


if __name__ == '__main__':
    main()
