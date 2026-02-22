#!/usr/bin/env python3
"""Ensemble sampling characterization for GR00T N1.6 action chunking.

Simulates realistic inference cadence (~5 Hz) over a configurable duration,
records all raw 16-step action horizons, and outputs per-joint CSV files:
  - The overlap matrix (each row = one inference, offset to correct frame)
  - Ground truth actions
  - Ensemble-averaged trajectories for each weighting strategy

Each inference fires only after the previous completes, advancing by
the number of action frames (at 15 FPS) that elapsed during the RTT.
This shows the actual overlap depth you'd get in production (~5-6 deep).

No ROS2 dependency — runs standalone against the ZMQ inference server.

Usage:
    python ensemble_characterization.py \
        --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
        --episode-index 3 \
        --host 192.168.50.201 --port 5555 \
        --duration-sec 4
"""

import argparse
import logging
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd

# Import reusable components
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from alfie_gr00t.core.chunk_buffer import ChunkBuffer, TimestampedChunk
from alfie_gr00t.core.zmq_client import ZMQClient, build_server_address
from alfie_gr00t.scripts.groot_open_loop_eval import (
    CAMERA_NAMES,
    JOINT_NAMES,
    JPEG_QUALITY,
    compress_jpeg,
    load_episode,
    load_video_frames,
)

logger = logging.getLogger(__name__)

CHUNK_SIZE = 16
TRAINING_FPS = 15
ENSEMBLE_STRATEGIES = ['uniform', 'recency', 'exp_decay', 'triangle']


# ── Horizon collection ────────────────────────────────────────────────

# Return type: list of (obs_frame, horizon_array, latency_ms)
HorizonEntry = tuple[int, np.ndarray, float]

ACTION_PERIOD_MS = 1000.0 / TRAINING_FPS  # 66.7ms per action step


def collect_horizons(
    client: ZMQClient,
    compressed_frames: list[dict],
    states: np.ndarray,
    task: str,
    num_frames: int,
    rtc: bool = False,
    rtc_freeze_steps: int = 4,
) -> list[HorizonEntry]:
    """Collect horizons at realistic cadence.

    Fires inference at frame 0, then advances by the number of action
    frames that elapsed during the inference RTT before firing the next.
    This simulates what the robot would actually experience.

    When rtc=True, sends prev_actions (tail of previous chunk) with each
    inference to enable server-side RTC freeze+inpaint.

    Returns list of (obs_frame, (16,22) array, latency_ms).
    """
    entries: list[HorizonEntry] = []
    t = 0
    prev_horizon: np.ndarray | None = None

    while t < num_frames:
        images = compressed_frames[t] if t < len(compressed_frames) else {}

        # RTC: send tail of previous chunk as freeze context
        prev_actions = None
        if rtc and prev_horizon is not None:
            # Send last rtc_freeze_steps actions from previous chunk
            prev_actions = prev_horizon[-rtc_freeze_steps:]

        t0 = time.monotonic()
        response = client.send_observation(
            images=images, state=states[t], language=task,
            prev_actions=prev_actions,
        )
        latency_ms = (time.monotonic() - t0) * 1000

        if response is not None and response.get('status') == 'ok':
            actions_list = response.get('actions', [])
            if actions_list:
                horizon = np.array(actions_list, dtype=np.float32)
                entries.append((t, horizon, latency_ms))
                prev_horizon = horizon
                rtc_tag = ' [RTC]' if rtc and prev_actions is not None else ''
                logger.info(
                    f'  inference {len(entries):3d}  obs_frame={t:3d}  '
                    f'latency={latency_ms:.0f}ms{rtc_tag}'
                )
                # Advance by however many action frames elapsed during inference
                frames_elapsed = max(1, round(latency_ms / ACTION_PERIOD_MS))
                t += frames_elapsed
        else:
            # Timeout or error — retry same frame (don't skip ahead)
            logger.warning(f'  inference failed at obs_frame={t}, retrying...')
            # Small advance to avoid infinite loop on persistent failures
            if latency_ms > 3000:
                t += 1

    logger.info(f'Collected {len(entries)} horizons over {num_frames} frames '
                f'(~{len(entries) / (num_frames / TRAINING_FPS):.1f} Hz)')
    return entries


def save_horizons(entries: list[HorizonEntry], path: str):
    """Save horizon entries to .npz for caching."""
    obs_frames = np.array([e[0] for e in entries], dtype=np.int32)
    horizons = np.array([e[1] for e in entries], dtype=np.float32)
    latencies = np.array([e[2] for e in entries], dtype=np.float32)
    np.savez_compressed(
        path,
        obs_frames=obs_frames,
        horizons=horizons,
        latencies=latencies,
    )
    logger.info(f'Saved {len(entries)} horizons to {path}')


def load_cached_horizons(path: str) -> list[HorizonEntry]:
    """Load cached horizons from .npz."""
    data = np.load(path)
    obs_frames = data['obs_frames']
    horizons = data['horizons']
    latencies = data['latencies']
    entries = [
        (int(obs_frames[i]), horizons[i], float(latencies[i]))
        for i in range(len(obs_frames))
    ]
    logger.info(f'Loaded {len(entries)} horizons from {path}')
    return entries


# ── Overlap matrix ────────────────────────────────────────────────────

def build_overlap_tensor(entries: list[HorizonEntry], num_frames: int):
    """Build overlap tensor: (num_inferences, max_col, 22).

    Each row corresponds to one inference call. The 16 action values
    are placed at columns [obs_frame, obs_frame+15].

    Returns (tensor, max_col, obs_frames_list).
    """
    n_inferences = len(entries)
    last_obs = entries[-1][0] if entries else 0
    max_col = max(num_frames, last_obs + CHUNK_SIZE)
    tensor = np.full((n_inferences, max_col, 22), np.nan, dtype=np.float32)
    obs_frames = []

    for i, (obs_frame, horizon, _) in enumerate(entries):
        obs_frames.append(obs_frame)
        n_steps = min(CHUNK_SIZE, horizon.shape[0])
        for j in range(n_steps):
            abs_frame = obs_frame + j
            if abs_frame < max_col:
                tensor[i, abs_frame, :] = horizon[j]

    return tensor, max_col, obs_frames


# ── Ensemble computation ──────────────────────────────────────────────

def compute_ensemble_rows(entries: list[HorizonEntry], num_frames: int, max_col: int):
    """Compute ensemble-averaged actions per strategy.

    Feeds chunks at their actual observation frames and queries every
    frame in [0, max_col).

    Returns dict mapping strategy name -> (max_col, 22) array.
    """
    results = {}

    for strategy in ENSEMBLE_STRATEGIES:
        buffer = ChunkBuffer(
            max_chunks=CHUNK_SIZE,
            strategy=strategy,
            decay_m=0.01,
            latency_skip=0,
            chunk_size=CHUNK_SIZE,
        )

        ensembled = np.full((max_col, 22), np.nan, dtype=np.float32)

        # Build lookup: frame -> list of entries arriving at that frame
        entry_by_frame: dict[int, list[tuple[int, np.ndarray]]] = {}
        for chunk_id, (obs_frame, horizon, _) in enumerate(entries):
            entry_by_frame.setdefault(obs_frame, []).append((chunk_id, horizon))

        for t in range(max_col):
            # Add any chunks that arrived at this frame
            if t in entry_by_frame:
                for chunk_id, horizon in entry_by_frame[t]:
                    chunk = TimestampedChunk(
                        actions=horizon,
                        obs_frame=t,
                        arrival_frame=t,
                        chunk_id=chunk_id,
                    )
                    buffer.add_chunk(chunk)

            action = buffer.get_action_raw(t)
            if action is not None:
                ensembled[t] = action

        results[f'ensemble_{strategy}'] = ensembled

    return results


# ── CSV output ────────────────────────────────────────────────────────

def write_joint_csvs(
    overlap_tensor, obs_frames, gt_actions, ensemble_results,
    max_col, output_dir,
):
    """Write one CSV per joint with the overlap matrix."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    n_inferences = overlap_tensor.shape[0]
    col_headers = [f'frame_{f}' for f in range(max_col)]

    for joint_idx, joint_name in enumerate(JOINT_NAMES):
        # Build all rows as a list of dicts, then create DataFrame once
        all_rows = []

        # Inference rows
        for i in range(n_inferences):
            row = {'': f'inf_{i}_f{obs_frames[i]}'}
            for f in range(max_col):
                val = overlap_tensor[i, f, joint_idx]
                row[f'frame_{f}'] = val if not np.isnan(val) else None
            all_rows.append(row)

        # Ground truth row
        gt_row = {'': 'gt'}
        for f in range(max_col):
            gt_row[f'frame_{f}'] = gt_actions[f, joint_idx] if f < len(gt_actions) else None
        all_rows.append(gt_row)

        # Ensemble rows
        for strategy_name, ensembled in ensemble_results.items():
            ens_row = {'': strategy_name}
            for f in range(max_col):
                val = ensembled[f, joint_idx]
                ens_row[f'frame_{f}'] = val if not np.isnan(val) else None
            all_rows.append(ens_row)

        df = pd.DataFrame(all_rows, columns=[''] + col_headers)
        csv_path = output_dir / f'{joint_name}.csv'
        df.to_csv(csv_path, index=False, float_format='%.6f')

    logger.info(f'Wrote {len(JOINT_NAMES)} CSV files to {output_dir}/')


# ── Summary ───────────────────────────────────────────────────────────

def print_summary(entries: list[HorizonEntry], overlap_tensor, max_col, num_frames):
    """Print overlap and prediction spread statistics."""
    n_inferences = overlap_tensor.shape[0]
    latencies = [e[2] for e in entries]

    # Overlap depth per frame (use joint 0 as proxy — all joints have same coverage)
    non_nan_count = np.sum(~np.isnan(overlap_tensor[:, :, 0]), axis=0)

    print()
    print('=' * 70)
    print('  ENSEMBLE CHARACTERIZATION SUMMARY')
    print('=' * 70)
    print()
    print(f'  Frames analyzed:       {num_frames}')
    print(f'  Inferences completed:  {n_inferences}')
    print(f'  Inference rate:        {n_inferences / (num_frames / TRAINING_FPS):.1f} Hz')
    print(f'  Total frame columns:   {max_col}')
    print(f'  Max overlap depth:     {non_nan_count.max()}')
    print(f'  Frames at max depth:   {np.sum(non_nan_count == non_nan_count.max())}')
    print()

    # Overlap depth histogram
    depth_counts = np.bincount(non_nan_count.astype(int))
    print(f'  OVERLAP DEPTH DISTRIBUTION')
    for depth in range(len(depth_counts)):
        if depth_counts[depth] > 0:
            bar = '#' * min(40, depth_counts[depth])
            print(f'    depth {depth:2d}: {depth_counts[depth]:3d} frames  {bar}')
    print()

    if latencies:
        lat = np.array(latencies)
        print(f'  INFERENCE LATENCY')
        print(f'    Mean:    {np.mean(lat):.0f} ms')
        print(f'    Median:  {np.median(lat):.0f} ms')
        print(f'    P95:     {np.percentile(lat, 95):.0f} ms')
        print(f'    Min:     {np.min(lat):.0f} ms')
        print(f'    Max:     {np.max(lat):.0f} ms')
        print(f'    Frames/inference: {np.mean(lat) / ACTION_PERIOD_MS:.1f}')
        print()

    # Prediction spread at high-overlap frames
    min_depth = max(2, non_nan_count.max() - 1)
    high_overlap_frames = np.where(non_nan_count >= min_depth)[0]

    if len(high_overlap_frames) > 0:
        print(f'  PREDICTION SPREAD (at {len(high_overlap_frames)} frames '
              f'with depth >= {min_depth})')
        print(f'  {"joint":25s}  {"mean_std":>10s}  {"max_std":>10s}')
        print(f'  {"-"*25}  {"-"*10}  {"-"*10}')

        for joint_idx, joint_name in enumerate(JOINT_NAMES):
            spreads = []
            for f in high_overlap_frames:
                preds = overlap_tensor[:, f, joint_idx]
                valid = preds[~np.isnan(preds)]
                if len(valid) > 1:
                    spreads.append(np.std(valid))
            if spreads:
                print(f'  {joint_name:25s}  {np.mean(spreads):10.6f}  {np.max(spreads):10.6f}')

    print()
    print('=' * 70)
    print()


# ── Main ──────────────────────────────────────────────────────────────

def run(args):
    """Run ensemble characterization."""
    dataset_path = args.dataset_path
    episode_index = args.episode_index
    num_requested = int(args.duration_sec * TRAINING_FPS)
    output_dir = Path(args.output_dir)
    cache_path = output_dir / f'horizons_ep{episode_index}.npz'

    print()
    print('=' * 60)
    print('  GR00T Ensemble Characterization')
    print('=' * 60)
    print(f'  Dataset:       {dataset_path}')
    print(f'  Episode:       {episode_index}')
    print(f'  Duration:      {args.duration_sec}s ({num_requested} frames)')
    print(f'  Task:          "{args.task}"')
    print(f'  RTC:           {"ON (freeze={})".format(args.rtc_freeze_steps) if args.rtc else "OFF"}')
    print(f'  Output:        {output_dir}/')
    print('=' * 60)
    print()

    # Load episode
    states, gt_actions = load_episode(dataset_path, episode_index)
    num_frames = min(len(states), num_requested)
    logger.info(f'Using {num_frames} of {len(states)} frames')

    # Collect or load horizons
    if args.load_horizons and cache_path.exists():
        entries = load_cached_horizons(str(cache_path))
        # Trim entries beyond requested duration
        entries = [e for e in entries if e[0] < num_frames]
    else:
        # Load video frames and pre-compress
        logger.info('Loading video frames...')
        video_frames = load_video_frames(dataset_path, episode_index, num_frames)
        logger.info(f'Pre-compressing {num_frames} frames to JPEG...')
        t0 = time.time()
        compressed_frames = [
            {cam: compress_jpeg(img) for cam, img in frame_dict.items()}
            for frame_dict in video_frames
        ]
        logger.info(f'Pre-compression done in {time.time() - t0:.1f}s')

        # Connect to server
        server_address = build_server_address(
            transport=args.transport, host=args.host, port=args.port,
        )
        logger.info(f'Connecting to {server_address}...')
        client = ZMQClient(
            server_address=server_address,
            timeout_ms=args.timeout_ms,
            logger=lambda msg: logger.info(msg),
        )
        if not client.connect():
            logger.error('Failed to connect')
            sys.exit(1)
        if not client.ping(timeout_ms=5000):
            logger.error('Server not responding')
            client.close()
            sys.exit(1)

        rtc_msg = ' with RTC freeze+inpaint' if args.rtc else ''
        logger.info(f'Running inference at realistic cadence over {num_frames} frames{rtc_msg}...')
        entries = collect_horizons(
            client, compressed_frames, states[:num_frames], args.task, num_frames,
            rtc=args.rtc, rtc_freeze_steps=args.rtc_freeze_steps,
        )
        client.close()

        if not entries:
            logger.error('No successful inferences. Exiting.')
            sys.exit(1)

        # Cache for fast re-runs
        output_dir.mkdir(parents=True, exist_ok=True)
        save_horizons(entries, str(cache_path))

    # Build overlap tensor
    logger.info('Building overlap matrices...')
    overlap_tensor, max_col, obs_frames = build_overlap_tensor(entries, num_frames)

    # Compute ensemble rows
    logger.info('Computing ensemble averages...')
    ensemble_results = compute_ensemble_rows(entries, num_frames, max_col)

    # Write CSVs
    logger.info('Writing per-joint CSVs...')
    write_joint_csvs(
        overlap_tensor, obs_frames, gt_actions, ensemble_results,
        max_col, output_dir,
    )

    # Summary
    print_summary(entries, overlap_tensor, max_col, num_frames)

    print(f'  CSV files: {output_dir}/')
    print(f'  Cached horizons: {cache_path}')
    print()


def parse_args():
    parser = argparse.ArgumentParser(
        description='Characterize ensemble sampling from GR00T action chunking',
    )
    parser.add_argument('--dataset-path', required=True,
                        help='Path to LeRobot-format dataset')
    parser.add_argument('--episode-index', type=int, default=0)
    parser.add_argument('--transport', choices=['tcp', 'ipc'], default='tcp')
    parser.add_argument('--host', default='192.168.50.201')
    parser.add_argument('--port', '-p', type=int, default=5555)
    parser.add_argument('--timeout-ms', type=int, default=5000)
    parser.add_argument('--duration-sec', type=float, default=4.0,
                        help='Duration in seconds (default: 4.0)')
    parser.add_argument('--task', default='find the can and pick it up')
    parser.add_argument('--output-dir', default='/tmp/ensemble_characterization/')
    parser.add_argument('--load-horizons', action='store_true',
                        help='Reuse cached .npz if available')
    parser.add_argument('--rtc', action='store_true',
                        help='Send prev_actions for RTC freeze+inpaint (server must have --rtc)')
    parser.add_argument('--rtc-freeze-steps', type=int, default=4,
                        help='Number of freeze steps for RTC (default: %(default)s)')
    parser.add_argument('--verbose', '-v', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
    )
    run(args)


if __name__ == '__main__':
    main()
