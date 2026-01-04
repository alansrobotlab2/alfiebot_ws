#!/usr/bin/env python3
"""
ROS2 Bag to GR00T N1.6 Training Data Converter

Converts ROS2 bag demonstrations (MCAP format) into the GR00T training format:
- Parquet files for state/action data
- MP4 videos for camera observations
- Updated meta files (info.jsonl, episodes.jsonl, stats.jsonl)

Usage:
    python3 rosbag_to_groot.py [--demos-dir PATH] [--output-dir PATH] [--task-index N] [--fps N]
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile
import zstandard as zstd
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from concurrent.futures import ThreadPoolExecutor, as_completed
from threading import Lock
import numpy as np
import cv2

# ROS2 imports
try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    from geometry_msgs.msg import Twist
    # Import custom message types
    import importlib
    alfie_msgs = importlib.import_module('alfie_msgs.msg')
    RobotLowState = alfie_msgs.RobotLowState
    RobotLowCmd = alfie_msgs.RobotLowCmd
except ImportError as e:
    print(f"Error importing ROS2 modules: {e}")
    print("Make sure to source your ROS2 workspace: source install/setup.bash")
    sys.exit(1)

# Data processing imports
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

# Video processing
import av


@dataclass
class EpisodeData:
    """Container for a single episode's data."""
    timestamps: List[float] = field(default_factory=list)
    states: List[np.ndarray] = field(default_factory=list)
    actions: List[np.ndarray] = field(default_factory=list)
    images: Dict[str, List[Tuple[float, np.ndarray]]] = field(default_factory=dict)

    def __post_init__(self):
        # Initialize image lists for each camera
        for cam in ['left_wide', 'right_wide', 'left_center', 'right_center']:
            self.images[cam] = []


class RosbagToGrootConverter:
    """Converts ROS2 bag files to GR00T training format."""

    # Topic to camera key mapping
    CAMERA_TOPICS = {
        '/alfie/stereo_camera/left_wide/image_raw/compressed': 'left_wide',
        '/alfie/stereo_camera/right_wide/image_raw/compressed': 'right_wide',
        '/alfie/stereo_camera/left_center/image_raw/compressed': 'left_center',
        '/alfie/stereo_camera/right_center/image_raw/compressed': 'right_center',
    }

    # Target video dimensions
    VIDEO_WIDTH = 320
    VIDEO_HEIGHT = 280

    def __init__(
        self,
        demos_dir: str,
        output_dir: str,
        fps: int = 15,
        chunk_size: int = 1000,
        num_threads: int = 1,
        use_gpu: bool = False,
    ):
        self.demos_dir = Path(demos_dir)
        self.output_dir = Path(output_dir)
        self.fps = fps
        self.chunk_size = chunk_size
        self.target_dt = 1.0 / fps
        self.num_threads = num_threads
        self.use_gpu = use_gpu

        # Ensure output directories exist
        self.data_dir = self.output_dir / 'data'
        self.videos_dir = self.output_dir / 'videos'
        self.meta_dir = self.output_dir / 'meta'

        for d in [self.data_dir, self.videos_dir, self.meta_dir]:
            d.mkdir(parents=True, exist_ok=True)

        # Statistics accumulators (thread-safe)
        self.all_states = []
        self.all_actions = []
        self.episode_metadata = []
        self.stats_lock = Lock()
        self.print_lock = Lock()

        # Detect available GPU encoder
        self.gpu_encoder = None
        if self.use_gpu:
            self.gpu_encoder = self._detect_gpu_encoder()

    def _detect_gpu_encoder(self) -> Optional[str]:
        """Detect available GPU video encoder."""
        # Try to detect NVENC (NVIDIA)
        try:
            result = subprocess.run(
                ['ffmpeg', '-hide_banner', '-encoders'],
                capture_output=True,
                text=True,
                timeout=5
            )
            output = result.stdout

            # Check for various NVIDIA encoders in order of preference
            if 'h264_nvenc' in output:
                print("Detected NVIDIA NVENC H.264 encoder")
                return 'h264_nvenc'
            elif 'hevc_nvenc' in output:
                print("Detected NVIDIA NVENC HEVC encoder")
                return 'hevc_nvenc'
            elif 'av1_nvenc' in output:
                print("Detected NVIDIA NVENC AV1 encoder")
                return 'av1_nvenc'
            # Check for AMD
            elif 'h264_amf' in output:
                print("Detected AMD AMF H.264 encoder")
                return 'h264_amf'
            # Check for Intel
            elif 'h264_qsv' in output:
                print("Detected Intel QuickSync H.264 encoder")
                return 'h264_qsv'
        except Exception as e:
            print(f"GPU encoder detection failed: {e}")

        print("No GPU encoder detected, falling back to CPU encoding")
        return None

    def find_demonstrations(self) -> List[Path]:
        """Find all demonstration directories."""
        demos = sorted([
            d for d in self.demos_dir.iterdir()
            if d.is_dir() and d.name.startswith('demo_')
        ])
        print(f"Found {len(demos)} demonstrations")
        return demos

    def decompress_mcap(self, mcap_zstd_path: Path) -> Path:
        """Decompress a zstd-compressed MCAP file."""
        mcap_path = mcap_zstd_path.with_suffix('')  # Remove .zstd

        if mcap_path.exists():
            return mcap_path

        print(f"  Decompressing {mcap_zstd_path.name}...")
        dctx = zstd.ZstdDecompressor()
        with open(mcap_zstd_path, 'rb') as ifh:
            with open(mcap_path, 'wb') as ofh:
                dctx.copy_stream(ifh, ofh)
        return mcap_path

    def read_rosbag(self, demo_dir: Path) -> Optional[EpisodeData]:
        """Read a ROS2 bag and extract all relevant data."""
        # Find the MCAP file
        mcap_files = list(demo_dir.glob('*.mcap.zstd'))
        if not mcap_files:
            mcap_files = list(demo_dir.glob('*.mcap'))
        if not mcap_files:
            print(f"  Warning: No MCAP file found in {demo_dir}")
            return None

        mcap_file = mcap_files[0]

        # Decompress if needed
        if mcap_file.suffix == '.zstd':
            mcap_file = self.decompress_mcap(mcap_file)

        episode = EpisodeData()

        # Raw data storage with timestamps
        state_msgs = []  # (timestamp_ns, RobotLowState)
        cmd_msgs = []    # (timestamp_ns, RobotLowCmd)
        image_msgs = {k: [] for k in self.CAMERA_TOPICS.values()}  # camera_key -> [(ts_ns, image)]

        # Read the bag
        storage_options = StorageOptions(uri=str(mcap_file), storage_id='mcap')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        reader = SequentialReader()
        reader.open(storage_options, converter_options)

        # Get topic types
        topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

        while reader.has_next():
            topic, data, timestamp_ns = reader.read_next()

            if topic == '/alfie/robotlowstate':
                msg = deserialize_message(data, RobotLowState)
                state_msgs.append((timestamp_ns, msg))
            elif topic == '/alfie/robotlowcmd':
                msg = deserialize_message(data, RobotLowCmd)
                cmd_msgs.append((timestamp_ns, msg))
            elif topic in self.CAMERA_TOPICS:
                msg = deserialize_message(data, CompressedImage)
                cam_key = self.CAMERA_TOPICS[topic]
                # Decode image
                np_arr = np.frombuffer(msg.data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if img is not None:
                    # Resize to target dimensions
                    img = cv2.resize(img, (self.VIDEO_WIDTH, self.VIDEO_HEIGHT))
                    # Convert BGR to RGB
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    image_msgs[cam_key].append((timestamp_ns, img))

        if not state_msgs or not cmd_msgs:
            print(f"  Warning: Missing state or command data in {demo_dir}")
            return None

        # Sort by timestamp
        state_msgs.sort(key=lambda x: x[0])
        cmd_msgs.sort(key=lambda x: x[0])
        for cam_key in image_msgs:
            image_msgs[cam_key].sort(key=lambda x: x[0])

        # Determine time range (use overlap of state and cmd)
        start_time_ns = max(state_msgs[0][0], cmd_msgs[0][0])
        end_time_ns = min(state_msgs[-1][0], cmd_msgs[-1][0])

        # Generate target timestamps at desired FPS
        target_times_ns = []
        t = start_time_ns
        dt_ns = int(self.target_dt * 1e9)
        while t <= end_time_ns:
            target_times_ns.append(t)
            t += dt_ns

        if len(target_times_ns) < 5:
            print(f"  Warning: Episode too short ({len(target_times_ns)} frames)")
            return None

        # Helper to find nearest message
        def find_nearest(msgs, target_ts):
            if not msgs:
                return None
            idx = np.searchsorted([m[0] for m in msgs], target_ts)
            if idx == 0:
                return msgs[0][1]
            if idx >= len(msgs):
                return msgs[-1][1]
            # Return closer one
            if abs(msgs[idx-1][0] - target_ts) < abs(msgs[idx][0] - target_ts):
                return msgs[idx-1][1]
            return msgs[idx][1]

        # Interpolate data to target timestamps
        reference_time_ns = target_times_ns[0]

        for ts_ns in target_times_ns:
            # Get nearest state and command
            state_msg = find_nearest(state_msgs, ts_ns)
            cmd_msg = find_nearest(cmd_msgs, ts_ns)

            if state_msg is None or cmd_msg is None:
                continue

            # Extract state vector (21 dimensions)
            state = self.extract_state(state_msg)
            action = self.extract_action(state_msg, cmd_msg)

            episode.states.append(state)
            episode.actions.append(action)
            episode.timestamps.append((ts_ns - reference_time_ns) / 1e9)

            # Get nearest images for each camera
            for cam_key, cam_msgs in image_msgs.items():
                img = find_nearest([(m[0], m[1]) for m in cam_msgs], ts_ns)
                if img is not None:
                    episode.images[cam_key].append(((ts_ns - reference_time_ns) / 1e9, img))

        return episode

    def extract_state(self, msg: 'RobotLowState') -> np.ndarray:
        """Extract 22-dimensional state vector from RobotLowState message.

        State vector layout (matches modality.json):
        [0-2]:   base linear velocity (x, y, z) from current_cmd_vel
        [3-5]:   base angular velocity (x, y, z) from current_cmd_vel
        [6]:     back joint (from back_state.current_position)
        [7-11]:  left arm joints (shoulder_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll)
        [12]:    left gripper
        [13-17]: right arm joints
        [18]:    right gripper
        [19-21]: head joints (yaw, pitch, roll)
        """
        state = np.zeros(22, dtype=np.float32)

        # Base velocity from current_cmd_vel (rate-limited velocity actually being sent)
        state[0] = msg.current_cmd_vel.linear.x
        state[1] = msg.current_cmd_vel.linear.y
        state[2] = msg.current_cmd_vel.linear.z
        state[3] = msg.current_cmd_vel.angular.x
        state[4] = msg.current_cmd_vel.angular.y
        state[5] = msg.current_cmd_vel.angular.z

        # Back joint (from back_state)
        state[6] = msg.back_state.current_position

        # Left arm (servos 0-4): shoulder_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll
        for i in range(5):
            state[7 + i] = msg.servo_state[i].current_location

        # Left gripper (servo 5)
        state[12] = msg.servo_state[5].current_location

        # Right arm (servos 6-10)
        for i in range(5):
            state[13 + i] = msg.servo_state[6 + i].current_location

        # Right gripper (servo 11)
        state[18] = msg.servo_state[11].current_location

        # Head (servos 12-14): yaw, pitch, roll
        for i in range(3):
            state[19 + i] = msg.servo_state[12 + i].current_location

        return state

    def extract_action(self, state_msg: 'RobotLowState', cmd_msg: 'RobotLowCmd') -> np.ndarray:
        """Extract 22-dimensional action vector from RobotLowState and RobotLowCmd messages.

        Action vector layout (matches modality.json):
        [0-2]:   base linear velocity command (x, y, z) from command_cmd_vel in state
        [3-5]:   base angular velocity command (x, y, z) from command_cmd_vel in state
        [6]:     back joint command (from back_state.command_position)
        [7-11]:  left arm joint commands from servo_cmd
        [12]:    left gripper command
        [13-17]: right arm joint commands from servo_cmd
        [18]:    right gripper command
        [19-21]: head joint commands from servo_cmd
        """
        action = np.zeros(22, dtype=np.float32)

        # Base velocity from command_cmd_vel (original commanded velocity before rate limiting)
        action[0] = state_msg.command_cmd_vel.linear.x
        action[1] = state_msg.command_cmd_vel.linear.y
        action[2] = state_msg.command_cmd_vel.linear.z
        action[3] = state_msg.command_cmd_vel.angular.x
        action[4] = state_msg.command_cmd_vel.angular.y
        action[5] = state_msg.command_cmd_vel.angular.z

        # Back joint command (from back_state)
        action[6] = state_msg.back_state.command_position

        # Left arm servo commands (target_location)
        for i in range(5):
            action[7 + i] = cmd_msg.servo_cmd[i].target_location

        # Left gripper command
        action[12] = cmd_msg.servo_cmd[5].target_location

        # Right arm servo commands
        for i in range(5):
            action[13 + i] = cmd_msg.servo_cmd[6 + i].target_location

        # Right gripper command
        action[18] = cmd_msg.servo_cmd[11].target_location

        # Head servo commands
        for i in range(3):
            action[19 + i] = cmd_msg.servo_cmd[12 + i].target_location

        return action

    def save_episode_parquet(
        self,
        episode: EpisodeData,
        episode_index: int,
        task_index: int,
    ) -> Tuple[str, int]:
        """Save episode data to parquet file."""
        chunk_index = episode_index // self.chunk_size
        chunk_dir = self.data_dir / f'chunk-{chunk_index:03d}'
        chunk_dir.mkdir(parents=True, exist_ok=True)

        parquet_path = chunk_dir / f'episode_{episode_index:06d}.parquet'

        num_frames = len(episode.states)

        # Build dataframe
        data = {
            'timestamp': np.array(episode.timestamps, dtype=np.float32),
            'frame_index': np.arange(num_frames, dtype=np.int64),
            'episode_index': np.full(num_frames, episode_index, dtype=np.int64),
            'index': np.arange(num_frames, dtype=np.int64),  # Global index (will update later)
            'task_index': np.full(num_frames, task_index, dtype=np.int64),
            'state': [s.tolist() for s in episode.states],
            'action': [a.tolist() for a in episode.actions],
        }

        df = pd.DataFrame(data)

        # Convert to pyarrow table with proper schema
        table = pa.Table.from_pandas(df)
        pq.write_table(table, parquet_path)

        return str(parquet_path), num_frames

    def save_episode_videos(
        self,
        episode: EpisodeData,
        episode_index: int,
    ) -> Dict[str, str]:
        """Save episode videos using GPU acceleration if available."""
        chunk_index = episode_index // self.chunk_size
        video_paths = {}

        for cam_key in ['left_wide', 'right_wide', 'left_center', 'right_center']:
            cam_dir = self.videos_dir / f'chunk-{chunk_index:03d}' / cam_key
            cam_dir.mkdir(parents=True, exist_ok=True)

            video_path = cam_dir / f'episode_{episode_index:06d}.mp4'

            if not episode.images.get(cam_key):
                print(f"    Warning: No images for {cam_key}")
                continue

            # Get just the images (without timestamps)
            frames = [img for _, img in episode.images[cam_key]]

            if len(frames) == 0:
                continue

            # Use GPU-accelerated encoding if available
            if self.gpu_encoder:
                success = self._encode_video_gpu(frames, video_path)
                if success:
                    video_paths[cam_key] = str(video_path)
                    continue

            # Fallback to CPU encoding with PyAV
            try:
                container = av.open(str(video_path), mode='w')
                stream = container.add_stream('libx264', rate=self.fps)
                stream.width = self.VIDEO_WIDTH
                stream.height = self.VIDEO_HEIGHT
                stream.pix_fmt = 'yuv420p'
                stream.options = {'crf': '23', 'preset': 'fast'}

                for frame_data in frames:
                    frame = av.VideoFrame.from_ndarray(frame_data, format='rgb24')
                    frame = frame.reformat(format='yuv420p')
                    for packet in stream.encode(frame):
                        container.mux(packet)

                for packet in stream.encode():
                    container.mux(packet)

                container.close()
                video_paths[cam_key] = str(video_path)

            except Exception as e:
                print(f"    Failed to save video: {e}")

        return video_paths

    def _encode_video_gpu(self, frames: List[np.ndarray], output_path: Path) -> bool:
        """Encode video using GPU acceleration via ffmpeg."""
        try:
            # Create a temporary directory for raw frames
            with tempfile.TemporaryDirectory() as tmpdir:
                tmpdir_path = Path(tmpdir)

                # Save frames as individual images
                for i, frame in enumerate(frames):
                    frame_path = tmpdir_path / f'frame_{i:06d}.png'
                    cv2.imwrite(str(frame_path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

                # Build ffmpeg command with GPU encoder
                encoder_opts = self._get_encoder_options()

                cmd = [
                    'ffmpeg',
                    '-y',  # Overwrite output
                    '-framerate', str(self.fps),
                    '-i', str(tmpdir_path / 'frame_%06d.png'),
                    '-c:v', self.gpu_encoder,
                ] + encoder_opts + [
                    '-pix_fmt', 'yuv420p',
                    str(output_path)
                ]

                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=60
                )

                if result.returncode == 0:
                    return True
                else:
                    print(f"    GPU encoding failed: {result.stderr}")
                    return False

        except Exception as e:
            print(f"    GPU encoding error: {e}")
            return False

    def _get_encoder_options(self) -> List[str]:
        """Get encoder-specific options for quality/performance."""
        if not self.gpu_encoder:
            return []

        # NVIDIA NVENC options
        if 'nvenc' in self.gpu_encoder:
            return [
                '-preset', 'p4',  # Performance preset (p1-p7, p4 is balanced)
                '-tune', 'hq',    # High quality tuning
                '-rc', 'vbr',     # Variable bitrate
                '-cq', '23',      # Constant quality (lower is better, 0-51)
                '-b:v', '0',      # Let CQ control bitrate
            ]
        # AMD AMF options
        elif 'amf' in self.gpu_encoder:
            return [
                '-quality', 'balanced',
                '-rc', 'vbr_peak',
                '-qp_i', '23',
                '-qp_p', '23',
            ]
        # Intel QuickSync options
        elif 'qsv' in self.gpu_encoder:
            return [
                '-preset', 'medium',
                '-global_quality', '23',
            ]

        return []

    def process_single_episode(
        self,
        demo_dir: Path,
        episode_index: int,
        task_index: int,
        demo_number: int,
        total_demos: int,
    ) -> Optional[Tuple[int, Dict]]:
        """Process a single episode (thread-safe)."""
        with self.print_lock:
            print(f"\nProcessing {demo_dir.name} ({demo_number}/{total_demos})...")

        episode = self.read_rosbag(demo_dir)
        if episode is None:
            return None

        with self.print_lock:
            print(f"  Extracted {len(episode.states)} frames")

        # Save parquet
        parquet_path, num_frames = self.save_episode_parquet(
            episode, episode_index, task_index
        )

        with self.print_lock:
            print(f"  Saved parquet: {parquet_path}")

        # Save videos
        video_paths = self.save_episode_videos(episode, episode_index)

        with self.print_lock:
            print(f"  Saved {len(video_paths)} videos")

        # Thread-safe statistics accumulation
        with self.stats_lock:
            self.all_states.extend(episode.states)
            self.all_actions.extend(episode.actions)

        # Return metadata for later collection
        metadata = {
            'episode_index': episode_index,
            'task_index': task_index,
            'num_frames': num_frames,
            'duration': episode.timestamps[-1] if episode.timestamps else 0,
            'source': demo_dir.name,
        }

        return num_frames, metadata

    def convert_all(self, task_index: int = 0, start_episode: int = 0) -> int:
        """Convert all demonstrations to GR00T format."""
        demos = self.find_demonstrations()

        if not demos:
            print("No demonstrations found!")
            return 0

        total_frames = 0
        episode_index = start_episode

        if self.num_threads <= 1:
            # Single-threaded processing (original behavior)
            for i, demo_dir in enumerate(demos):
                result = self.process_single_episode(
                    demo_dir, episode_index, task_index, i + 1, len(demos)
                )
                if result is not None:
                    num_frames, metadata = result
                    self.episode_metadata.append(metadata)
                    total_frames += num_frames
                    episode_index += 1
        else:
            # Multi-threaded processing
            with ThreadPoolExecutor(max_workers=self.num_threads) as executor:
                # Submit all jobs
                futures = {}
                for i, demo_dir in enumerate(demos):
                    future = executor.submit(
                        self.process_single_episode,
                        demo_dir,
                        episode_index + i,
                        task_index,
                        i + 1,
                        len(demos),
                    )
                    futures[future] = i

                # Collect results as they complete
                completed_metadata = {}
                for future in as_completed(futures):
                    result = future.result()
                    if result is not None:
                        num_frames, metadata = result
                        # Store with original index to maintain order
                        completed_metadata[futures[future]] = (num_frames, metadata)

                # Add metadata in order
                for i in range(len(demos)):
                    if i in completed_metadata:
                        num_frames, metadata = completed_metadata[i]
                        self.episode_metadata.append(metadata)
                        total_frames += num_frames
                        episode_index += 1

        print(f"\n{'='*60}")
        print(f"Converted {episode_index - start_episode} episodes ({total_frames} frames)")

        return episode_index - start_episode

    def compute_and_save_stats(self):
        """Compute and save normalization statistics."""
        if not self.all_states or not self.all_actions:
            print("No data to compute statistics")
            return

        states = np.array(self.all_states)
        actions = np.array(self.all_actions)

        stats = {
            'state': {
                'mean': states.mean(axis=0).tolist(),
                'std': states.std(axis=0).tolist(),
                'min': states.min(axis=0).tolist(),
                'max': states.max(axis=0).tolist(),
            },
            'action': {
                'mean': actions.mean(axis=0).tolist(),
                'std': actions.std(axis=0).tolist(),
                'min': actions.min(axis=0).tolist(),
                'max': actions.max(axis=0).tolist(),
            }
        }

        stats_path = self.meta_dir / 'stats.jsonl'
        with open(stats_path, 'w') as f:
            json.dump(stats, f, indent=2)
        print(f"Saved statistics to {stats_path}")

        # Also save relative stats (normalized)
        relative_stats = {
            'state': {
                'mean': [0.0] * 22,
                'std': [1.0] * 22,
            },
            'action': {
                'mean': [0.0] * 22,
                'std': [1.0] * 22,
            }
        }

        relative_stats_path = self.meta_dir / 'relative_stats.jsonl'
        with open(relative_stats_path, 'w') as f:
            json.dump(relative_stats, f, indent=2)
        print(f"Saved relative statistics to {relative_stats_path}")

    def save_episodes_metadata(self):
        """Save episodes.jsonl with per-episode metadata."""
        episodes_path = self.meta_dir / 'episodes.jsonl'
        with open(episodes_path, 'w') as f:
            for ep in self.episode_metadata:
                f.write(json.dumps(ep) + '\n')
        print(f"Saved episode metadata to {episodes_path}")

    def update_info_json(self, num_episodes: int, total_frames: int):
        """Update info.json with correct counts."""
        info_path = self.meta_dir / 'info.json'

        # Read existing info
        with open(info_path, 'r') as f:
            info = json.load(f)

        # Update counts
        info['total_episodes'] = num_episodes
        info['total_frames'] = total_frames
        info['total_chunks'] = (num_episodes + self.chunk_size - 1) // self.chunk_size
        info['total_videos'] = num_episodes * 4  # 4 cameras per episode

        # Update splits
        info['splits'] = {
            'train': f'0:{num_episodes}'
        }

        # Write back
        with open(info_path, 'w') as f:
            json.dump(info, f, indent=4)
        print(f"Updated {info_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Convert ROS2 bag demonstrations to GR00T N1.6 training format'
    )
    parser.add_argument(
        '--demos-dir',
        type=str,
        default='/home/alfie/alfiebot_ws/data/demonstrations',
        help='Directory containing demonstration folders'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default='/home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge',
        help='Output directory for GR00T format data'
    )
    parser.add_argument(
        '--task-index',
        type=int,
        default=0,
        help='Task index (0=pick up can, 1=put can down)'
    )
    parser.add_argument(
        '--fps',
        type=int,
        default=15,
        help='Target frames per second'
    )
    parser.add_argument(
        '--start-episode',
        type=int,
        default=0,
        help='Starting episode index (for appending to existing dataset)'
    )
    parser.add_argument(
        '--num-threads',
        type=int,
        default=1,
        help='Number of parallel threads for conversion (default: 1)'
    )
    parser.add_argument(
        '--use-gpu',
        action='store_true',
        help='Use GPU acceleration for video encoding (requires NVENC/AMF/QSV)'
    )

    args = parser.parse_args()

    print("="*60)
    print("ROS2 Bag to GR00T Converter")
    print("="*60)
    print(f"Demos directory: {args.demos_dir}")
    print(f"Output directory: {args.output_dir}")
    print(f"Task index: {args.task_index}")
    print(f"Target FPS: {args.fps}")
    print(f"Starting episode: {args.start_episode}")
    print(f"Number of threads: {args.num_threads}")
    print(f"GPU acceleration: {'Enabled' if args.use_gpu else 'Disabled'}")
    print("="*60)

    converter = RosbagToGrootConverter(
        demos_dir=args.demos_dir,
        output_dir=args.output_dir,
        fps=args.fps,
        num_threads=args.num_threads,
        use_gpu=args.use_gpu,
    )

    num_episodes = converter.convert_all(
        task_index=args.task_index,
        start_episode=args.start_episode
    )

    if num_episodes > 0:
        total_frames = sum(ep['num_frames'] for ep in converter.episode_metadata)

        # Save all metadata
        converter.compute_and_save_stats()
        converter.save_episodes_metadata()
        converter.update_info_json(
            num_episodes=args.start_episode + num_episodes,
            total_frames=total_frames
        )

        print("\n" + "="*60)
        print("Conversion complete!")
        print(f"  Episodes: {num_episodes}")
        print(f"  Total frames: {total_frames}")
        print("="*60)
    else:
        print("\nNo episodes converted.")


if __name__ == '__main__':
    main()
