#!/usr/bin/env python3
"""Replay evaluation through the full GR00T client pipeline.

Replays recorded episodes from a LeRobot dataset through the ZMQ inference
server AND the full client-side processing pipeline (latency skip, chunk
promotion, blending, rate-limited interpolation, base velocity limiting).

This differs from groot_open_loop_eval.py which only evaluates the server's
raw predictions. This script shows what the robot would actually execute by
simulating the entire groot_client.py command callback in deterministic time,
including chunk stepping, promotion, hold-and-wait, and the 100Hz rate-limited
interpolation sub-loop.

Accepts the same groot_client.yaml config file used by the live client node.
Any YAML parameter can be overridden via CLI flags for A/B comparison without
editing the config file.

No ROS2 dependency — runs standalone.

Usage:

# Continuous mode with exp_decay ensembling + EMA
python src/alfie_gr00t/alfie_gr00t/scripts/groot_replay_eval.py \
    --config src/alfie_gr00t/config/groot_client.yaml \
    --dataset-path data/alfiebot.CanDoChallenge \
    --episode-index 0 \
    --continuous-inference true \
    --smoothing-strategy exp_decay \
    --joint-smoothing-alpha 0.9 \
    --host 192.168.50.201


    # Basic evaluation
    python groot_replay_eval.py \\
        --config config/groot_client.yaml \\
        --dataset-path data/alfiebot.CanDoChallenge \\
        --episode-index 0 \\
        --host 192.168.50.201

    # Compare rate limiting on vs off
    python groot_replay_eval.py --config config/groot_client.yaml \\
        --dataset-path data/alfiebot.CanDoChallenge --episode-index 0 \\
        --output-dir /tmp/eval_rl_on/

    python groot_replay_eval.py --config config/groot_client.yaml \\
        --dataset-path data/alfiebot.CanDoChallenge --episode-index 0 \\
        --rate-limit-enabled false --output-dir /tmp/eval_rl_off/

    # Multiple episodes with CSV export
    python groot_replay_eval.py --config config/groot_client.yaml \\
        --dataset-path data/alfiebot.CanDoChallenge \\
        --episode-indices 0,1,2,3,4 --save-csv

Output Directory Structure:
    <output-dir>/                         (default: /tmp/groot_replay_eval/)
        config.json                       Effective config (YAML + CLI overrides)
        summary.json                      All metrics, machine-readable (see below)
        episode_000/
            raw_trajectory.png            Raw server predictions vs GT, per joint
            raw_trajectory_grouped.png    Raw predictions vs GT, by body part
            processed_trajectory.png      Post-pipeline predictions vs GT, per joint
            processed_trajectory_grouped.png  Post-pipeline vs GT, by body part
            pipeline_delta.png            |processed - raw| per body part over time
            comparison.png               Overlay of raw vs processed vs GT (key joints)
            comms.png                     Latency, message sizes, effective FPS
            actions.csv                   Per-frame CSV (with --save-csv flag)

Key Output Metrics (in summary.json):

    Accuracy — how well predictions match ground truth:
        MSE / MAE (overall, per-joint, per-group)
            Computed for both 'raw' (server output) and 'processed' (after the
            full client pipeline). Comparing the two shows whether client-side
            processing helps or hurts trajectory accuracy.

    Pipeline delta — effect of client-side processing:
        mean |processed - raw|  (overall, per-joint, per-group)
            How much the pipeline modified the server's raw predictions.
        rate_limit_clip_ticks
            Number of 100Hz ticks where a position joint was actively
            velocity-clamped by the rate limiter.
        base_vel_clip_count / base_accel_clip_count
            How often base velocity magnitude or acceleration was capped.
        hold_count
            Frames spent in hold-and-wait (chunk exhausted, no pending).
            High values mean inference is too slow or n_action_steps too small.

    Trajectory smoothness — noise characterization:
        jerk_rms
            RMS of the 2nd derivative (d^2x/dt^2). Lower = smoother.
            Computed for GT, raw, and processed trajectories.
        jerk_ratio
            pred_jerk / gt_jerk. A normalized noise score.
            1.0x = as smooth as the human demonstration.
            >1x = noisier. The key number for comparing configs: if config A
            gives 3.5x and config B gives 1.8x, B produces smoother output.
        hf_power_ratio
            Fraction of FFT power above 3 Hz (Nyquist at 15 FPS = 7.5 Hz).
            Captures high-frequency jitter. GT demonstrations are ~0%.
            Model noise shows up here even if overall MSE is low.
        delta_rms
            RMS of frame-to-frame differences. Simpler smoothness measure.
        All smoothness metrics are also broken down per body part (base, back,
        left_arm, left_hand, right_arm, right_hand, head).

    Communication:
        Latency stats (mean, median, P95, min, max), server timing breakdown,
        message sizes, and bandwidth estimates.
"""

import argparse
import csv
import json
import logging
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from matplotlib import pyplot as plt

# Import ZMQ client and pipeline components (no ROS2 dependency)
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from alfie_gr00t.core.action_interpolator import ActionInterpolator
from alfie_gr00t.core.action_smoother import ActionSmoother
from alfie_gr00t.core.chunk_buffer import ChunkBuffer, TimestampedChunk
from alfie_gr00t.core.rate_limited_interpolator import RateLimitedInterpolator
from alfie_gr00t.core.zmq_client import ZMQClient, build_server_address
from alfie_gr00t.scripts.groot_open_loop_eval import (
    BODY_PART_GROUPS,
    CAMERA_NAMES,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    JOINT_NAMES,
    compress_jpeg,
    load_episode,
    load_stats,
    load_video_frames,
    plot_comms_report,
    plot_grouped_results,
    plot_trajectory_results,
    print_comms_summary,
    print_timing_breakdown,
)

logger = logging.getLogger(__name__)

# Training data rate
ACTION_STEP_PERIOD = 1.0 / 15.0  # ~66.7ms per action step
# 100Hz control ticks per 15 FPS frame (100/15 ≈ 6.67, use 7)
TICKS_PER_FRAME = 7
TICK_DT = 0.01  # 10ms per 100Hz tick

# Subplot height multiplier for all output plots. Taller subplots make
# high-frequency noise and jitter visually obvious in the trajectories.
SUBPLOT_Y_SCALE = 3


# ---------------------------------------------------------------------------
# YAML config loading (no ROS2 parameter server)
# ---------------------------------------------------------------------------

# Default values matching groot_client.py _declare_parameters()
PARAM_DEFAULTS = {
    'transport': 'tcp',
    'server_host': '192.168.50.108',
    'server_port': 5555,
    'ipc_path': '/tmp/groot_inference.sock',
    'use_async_zmq': False,
    'async_push_port': 5556,
    'async_pull_port': 5557,
    'inference_timeout_ms': 5000,
    'task_description': 'find the can and pick it up',
    'enable_safety_limits': True,
    'base_smoothing_alpha': 1.0,
    'joint_smoothing_alpha': 0.95,
    'action_chunk_size': 16,
    'n_action_steps': 8,
    'csv_log_path': '',
    'latency_skip': 4,
    'inference_trigger_step': 4,
    'chunk_blend_steps': 2,
    'debug_save_images': False,
    'h264_conditioning': False,
    'interpolate_actions': True,
    'back_init_height': 0.1,
    'continuous_inference': False,
    'smoothing_strategy': 'latest',
    'smoothing_decay_m': 0.01,
    'max_buffer_chunks': 8,
    'smoothing_method': 'none',
    'savgol_window': 5,
    'savgol_polyorder': 2,
    'butterworth_order': 2,
    'butterworth_cutoff_hz': 5.0,
    'interpolation_method': 'linear',
    'spline_window': 6,
    'rate_limit_enabled': True,
    'rate_limit_back': 0.3,
    'rate_limit_left_arm': 2.0,
    'rate_limit_left_gripper': 3.0,
    'rate_limit_right_arm': 2.0,
    'rate_limit_right_gripper': 3.0,
    'rate_limit_head': 1.0,
    'max_base_linear_x': 0.15,
    'max_base_linear_y': 0.15,
    'max_base_angular_z': 0.8,
    'max_base_linear_accel': 0.3,
    'max_base_angular_accel': 1.5,
}


def load_config(yaml_path: str) -> dict:
    """Load groot_client.yaml parameters without ROS2.

    Parses the YAML file and extracts the ros__parameters dict.
    Missing keys get defaults from PARAM_DEFAULTS.
    """
    import yaml

    with open(yaml_path) as f:
        raw = yaml.safe_load(f)

    # Navigate /alfie/groot_client -> ros__parameters
    params = {}
    for key, value in raw.items():
        if 'groot_client' in key:
            params = value.get('ros__parameters', {})
            break

    # Fill missing with defaults
    config = dict(PARAM_DEFAULTS)
    config.update(params)
    return config


def apply_cli_overrides(config: dict, args: argparse.Namespace) -> dict:
    """Apply CLI argument overrides to config dict."""
    overrides = {
        'host': 'server_host',
        'port': 'server_port',
        'transport': 'transport',
        'timeout_ms': 'inference_timeout_ms',
        'task': 'task_description',
        'n_action_steps': 'n_action_steps',
        'latency_skip': 'latency_skip',
        'inference_trigger_step': 'inference_trigger_step',
        'chunk_blend_steps': 'chunk_blend_steps',
        'rate_limit_enabled': 'rate_limit_enabled',
        'rate_limit_back': 'rate_limit_back',
        'rate_limit_left_arm': 'rate_limit_left_arm',
        'rate_limit_left_gripper': 'rate_limit_left_gripper',
        'rate_limit_right_arm': 'rate_limit_right_arm',
        'rate_limit_right_gripper': 'rate_limit_right_gripper',
        'rate_limit_head': 'rate_limit_head',
        'max_base_linear_x': 'max_base_linear_x',
        'max_base_linear_y': 'max_base_linear_y',
        'max_base_angular_z': 'max_base_angular_z',
        'max_base_linear_accel': 'max_base_linear_accel',
        'max_base_angular_accel': 'max_base_angular_accel',
        'h264_conditioning': 'h264_conditioning',
        'continuous_inference': 'continuous_inference',
        'smoothing_strategy': 'smoothing_strategy',
        'smoothing_decay_m': 'smoothing_decay_m',
        'max_buffer_chunks': 'max_buffer_chunks',
        'base_smoothing_alpha': 'base_smoothing_alpha',
        'joint_smoothing_alpha': 'joint_smoothing_alpha',
        'smoothing_method': 'smoothing_method',
        'savgol_window': 'savgol_window',
        'savgol_polyorder': 'savgol_polyorder',
        'butterworth_order': 'butterworth_order',
        'butterworth_cutoff_hz': 'butterworth_cutoff_hz',
        'interpolate_actions': 'interpolate_actions',
        'interpolation_method': 'interpolation_method',
        'spline_window': 'spline_window',
    }
    for arg_name, config_key in overrides.items():
        val = getattr(args, arg_name, None)
        if val is not None:
            config[config_key] = val
    return config


# ---------------------------------------------------------------------------
# Base velocity limiter (extracted from ActionPublisher._limit_base_velocity)
# ---------------------------------------------------------------------------

class BaseVelocityLimiter:
    """Standalone base velocity magnitude + acceleration limiter.

    Extracted from action_publisher.py:167-194 to avoid ROS2 dependency.
    """

    def __init__(
        self,
        max_linear_x: float = 0.15,
        max_linear_y: float = 0.15,
        max_angular_z: float = 0.8,
        max_linear_accel: float = 0.3,
        max_angular_accel: float = 1.5,
        dt: float = TICK_DT,
    ):
        self._max_base_vel = np.array([
            max_linear_x, max_linear_y, 0.5, 1.5, 1.5, max_angular_z,
        ], dtype=np.float32)
        self._max_linear_accel = max_linear_accel
        self._max_angular_accel = max_angular_accel
        self._dt = dt
        self._last_base_vel: Optional[np.ndarray] = None
        self.vel_clips = 0
        self.accel_clips = 0

    def limit(self, base_vel: np.ndarray) -> np.ndarray:
        result = base_vel.copy()

        # 1. Clamp magnitude per-axis
        clipped = np.clip(result, -self._max_base_vel, self._max_base_vel)
        if not np.array_equal(result, clipped):
            self.vel_clips += 1
        result = clipped

        # 2. Clamp acceleration (dv/dt) to prevent jerk
        if self._last_base_vel is not None:
            dv = result - self._last_base_vel
            max_dv_linear = self._max_linear_accel * self._dt
            max_dv_angular = self._max_angular_accel * self._dt
            dv[0:3] = np.clip(dv[0:3], -max_dv_linear, max_dv_linear)
            dv[3:6] = np.clip(dv[3:6], -max_dv_angular, max_dv_angular)
            new_result = self._last_base_vel + dv
            if not np.allclose(result, new_result, atol=1e-6):
                self.accel_clips += 1
            result = new_result

        self._last_base_vel = result.copy()
        return result

    def reset(self):
        self._last_base_vel = None
        self.vel_clips = 0
        self.accel_clips = 0


# ---------------------------------------------------------------------------
# Client pipeline simulator
# ---------------------------------------------------------------------------

class ClientPipelineSimulator:
    """Simulates the groot_client's full action processing pipeline.

    Supports two modes matching groot_client.py:

    CLASSIC MODE (continuous_inference=false):
        _classic_command_callback() logic from groot_client.py:1187-1330.
        One chunk at a time with promotion, blending, hold-and-wait.
        Pipeline: chunk[abs_idx] → blend → rate_limiter OR interpolator
                  → base_vel_limiter → output.

    CONTINUOUS MODE (continuous_inference=true):
        _continuous_command_callback() logic from groot_client.py:1077-1129.
        Every frame fires inference. Overlapping chunks combined via
        temporal ensembling in ChunkBuffer.
        Pipeline: ChunkBuffer.get_action(frame) → ActionSmoother
                  → ActionInterpolator → base_vel_limiter → output.

    Both modes run the 100Hz sub-loop (TICKS_PER_FRAME ticks per GT frame)
    for rate limiting and base velocity limiting.
    """

    def __init__(self, config: dict):
        self.continuous = bool(config.get('continuous_inference', False))
        self.action_chunk_size = int(config['action_chunk_size'])
        self.n_action_steps = int(config['n_action_steps'])
        self.latency_skip = int(config['latency_skip'])
        self.inference_trigger_step = int(config['inference_trigger_step'])
        self.chunk_blend_steps = int(config['chunk_blend_steps'])
        self.interpolate_actions = bool(config.get('interpolate_actions', True))

        # Rate limiter (used in classic mode; continuous mode skips it)
        self._rate_limiter: Optional[RateLimitedInterpolator] = None
        if config.get('rate_limit_enabled', True) and not self.continuous:
            self._rate_limiter = RateLimitedInterpolator(
                max_speeds={
                    'back': config['rate_limit_back'],
                    'left_arm': config['rate_limit_left_arm'],
                    'left_gripper': config['rate_limit_left_gripper'],
                    'right_arm': config['rate_limit_right_arm'],
                    'right_gripper': config['rate_limit_right_gripper'],
                    'head': config['rate_limit_head'],
                },
                dt=TICK_DT,
            )

        # Legacy ActionInterpolator (classic mode, rate_limit_enabled=false)
        self._action_interpolator: Optional[ActionInterpolator] = None
        if not self.continuous and self._rate_limiter is None and self.interpolate_actions:
            self._action_interpolator = ActionInterpolator(
                method=config.get('interpolation_method', 'linear'),
                spline_window=int(config.get('spline_window', 6)),
            )

        # Base velocity limiter (both modes)
        self._base_limiter = BaseVelocityLimiter(
            max_linear_x=config['max_base_linear_x'],
            max_linear_y=config['max_base_linear_y'],
            max_angular_z=config['max_base_angular_z'],
            max_linear_accel=config['max_base_linear_accel'],
            max_angular_accel=config['max_base_angular_accel'],
        )

        # --- Continuous mode components ---
        self._chunk_buffer: Optional[ChunkBuffer] = None
        self._action_smoother: Optional[ActionSmoother] = None
        self._cont_interpolator: Optional[ActionInterpolator] = None

        if self.continuous:
            self._chunk_buffer = ChunkBuffer(
                max_chunks=int(config.get('max_buffer_chunks', 8)),
                strategy=config.get('smoothing_strategy', 'latest'),
                decay_m=float(config.get('smoothing_decay_m', 0.01)),
                latency_skip=self.latency_skip,
                chunk_size=self.action_chunk_size,
                ema_alpha_base=float(config.get('base_smoothing_alpha', 1.0)),
                ema_alpha_joints=float(config.get('joint_smoothing_alpha', 1.0)),
            )

            sm = config.get('smoothing_method', 'none')
            self._action_smoother = ActionSmoother(
                method=sm,
                savgol_window=int(config.get('savgol_window', 5)),
                savgol_polyorder=int(config.get('savgol_polyorder', 2)),
                butter_order=int(config.get('butterworth_order', 2)),
                butter_cutoff_hz=float(config.get('butterworth_cutoff_hz', 5.0)),
            )

            if self.interpolate_actions:
                self._cont_interpolator = ActionInterpolator(
                    method=config.get('interpolation_method', 'linear'),
                    spline_window=int(config.get('spline_window', 6)),
                )

        # --- Classic mode chunk state ---
        self._action_chunk: Optional[np.ndarray] = None
        self._pending_chunk: Optional[np.ndarray] = None
        self._effective_skip = 0
        self._blend_from: Optional[np.ndarray] = None
        self._exec_idx = 0
        self._chunk_id = 0
        self._is_first_chunk = True

        # --- Tracking (both modes) ---
        self._hold_count = 0
        self._rate_limit_clip_ticks = 0
        self._current_frame = 0  # continuous mode frame counter

    def reset(self):
        """Reset all state for a new episode."""
        self._action_chunk = None
        self._pending_chunk = None
        self._effective_skip = 0
        self._blend_from = None
        self._exec_idx = 0
        self._chunk_id = 0
        self._is_first_chunk = True
        self._hold_count = 0
        self._rate_limit_clip_ticks = 0
        self._current_frame = 0
        if self._rate_limiter is not None:
            self._rate_limiter.reset()
        if self._action_interpolator is not None:
            self._action_interpolator.reset()
        self._base_limiter.reset()
        if self._chunk_buffer is not None:
            self._chunk_buffer.reset()
        if self._action_smoother is not None:
            self._action_smoother.reset()
        if self._cont_interpolator is not None:
            self._cont_interpolator.reset()

    # ------------------------------------------------------------------
    # Inference scheduling
    # ------------------------------------------------------------------

    def needs_inference(self) -> bool:
        """Check if we need to fire inference at the current step.

        Classic: trigger at inference_trigger_step (or first chunk).
        Continuous: always (every frame fires inference).
        """
        if self.continuous:
            return True
        if self._is_first_chunk:
            return True
        if self._pending_chunk is not None:
            return False
        return self._exec_idx == self.inference_trigger_step

    def install_chunk(self, chunk: np.ndarray):
        """Install an action chunk from the server.

        Classic: first chunk installs immediately (skip=0), subsequent as pending.
        Continuous: adds to ChunkBuffer with current frame as arrival_frame.
        """
        if self.continuous:
            self._chunk_buffer.add_chunk(TimestampedChunk(
                actions=chunk,
                obs_frame=self._current_frame,
                arrival_frame=self._current_frame,
                chunk_id=self._chunk_id,
            ))
            self._chunk_id += 1
        else:
            if self._is_first_chunk:
                self._action_chunk = chunk
                self._effective_skip = 0
                self._is_first_chunk = False
                self._chunk_id = 1
            else:
                self._pending_chunk = chunk

    # ------------------------------------------------------------------
    # Frame stepping
    # ------------------------------------------------------------------

    def step_frame(self) -> tuple[np.ndarray, np.ndarray, dict]:
        """Advance one GT frame (15 FPS). Run 100Hz sub-loop internally.

        Returns:
            (raw_action, processed_action, info_dict)
        """
        if self.continuous:
            return self._step_continuous()
        return self._step_classic()

    def _step_continuous(self) -> tuple[np.ndarray, np.ndarray, dict]:
        """Continuous mode: ChunkBuffer → Smoother → Interpolator → base limiter.

        Replicates _continuous_command_callback() from groot_client.py:1077-1129.
        """
        frame = self._current_frame
        zeros = np.zeros(22, dtype=np.float32)

        # Get raw ensembled action from ChunkBuffer (before EMA)
        raw_action_pre = self._chunk_buffer.get_action_raw(frame)
        if raw_action_pre is None:
            self._current_frame += 1
            self._hold_count += 1
            return zeros, zeros, {
                'hold': True, 'chunk_id': self._chunk_id,
                'abs_idx': 0, 'promoted': False,
            }

        raw_action = raw_action_pre.copy()

        # Get ensembled action (with EMA)
        action = self._chunk_buffer.get_action(frame)
        if action is None:
            action = raw_action.copy()

        # Post-ensembling filter (savgol / butterworth)
        action = self._action_smoother.smooth(action)

        # ActionInterpolator (15 FPS → sub-frame)
        if self._cont_interpolator is not None:
            self._cont_interpolator.update_waypoint(frame, action)
            # Also feed next frame for interpolation lookahead
            next_raw = self._chunk_buffer.get_action(frame + 1)
            if next_raw is not None:
                next_smoothed = self._action_smoother.smooth(next_raw)
                self._cont_interpolator.update_waypoint(frame + 1, next_smoothed)

        # 100Hz sub-loop: interpolation + base velocity limiting
        processed = action.copy()
        for tick in range(TICKS_PER_FRAME):
            if self._cont_interpolator is not None:
                frac_time = frame + tick / TICKS_PER_FRAME
                interp = self._cont_interpolator.evaluate(frac_time)
                if interp is not None:
                    tick_action = np.asarray(interp, dtype=np.float64)
                else:
                    tick_action = action.copy()
            else:
                tick_action = action.copy()

            tick_action[0:6] = self._base_limiter.limit(
                np.asarray(tick_action[0:6], dtype=np.float32)
            )
            processed = tick_action

        self._current_frame += 1

        return raw_action.astype(np.float32), processed.astype(np.float32), {
            'hold': False, 'chunk_id': self._chunk_id,
            'abs_idx': frame, 'promoted': False,
        }

    def _step_classic(self) -> tuple[np.ndarray, np.ndarray, dict]:
        """Classic mode: chunk stepping with promotion, blending, rate limiting.

        Replicates _classic_command_callback() from groot_client.py:1187-1330.
        """
        chunk = self._action_chunk
        if chunk is None:
            zeros = np.zeros(22, dtype=np.float32)
            self._current_frame += 1
            return zeros, zeros, {'hold': True, 'chunk_id': 0, 'abs_idx': 0}

        # Check promotion: if exec window exhausted and pending available
        promoted = False
        if self._pending_chunk is not None and self._exec_idx >= self.n_action_steps:
            # Save blend source
            old_abs_idx = min(self._effective_skip + self._exec_idx, len(chunk) - 1)
            if self.chunk_blend_steps > 0:
                self._blend_from = chunk[old_abs_idx].copy()

            # Promote
            self._effective_skip = self.latency_skip + (1 if self.chunk_blend_steps > 0 else 0)
            self._action_chunk = self._pending_chunk
            self._pending_chunk = None
            self._chunk_id += 1
            self._exec_idx = 0
            chunk = self._action_chunk
            promoted = True

        abs_idx = self._effective_skip + self._exec_idx

        # Hold-and-wait if all 16 actions exhausted with no pending
        is_hold = False
        if abs_idx >= len(chunk):
            is_hold = True
            self._hold_count += 1
            raw_action = chunk[-1].copy()
            raw_action[0:6] = 0.0  # zero base velocity

            # Still run through rate limiter and base limiter for hold
            processed = raw_action.copy()
            for _ in range(TICKS_PER_FRAME):
                tick_action = raw_action.copy()
                if self._rate_limiter is not None:
                    self._rate_limiter.set_target(tick_action)
                    stepped = self._rate_limiter.step()
                    if stepped is not None:
                        tick_action = stepped
                tick_action[0:6] = self._base_limiter.limit(tick_action[0:6])
                processed = tick_action

            self._exec_idx += 1
            self._current_frame += 1
            return raw_action, processed, {
                'hold': True, 'chunk_id': self._chunk_id,
                'abs_idx': len(chunk) - 1, 'promoted': promoted,
            }

        abs_idx = min(abs_idx, len(chunk) - 1)
        raw_action = chunk[abs_idx].copy()
        target = chunk[abs_idx].copy()

        # Chunk transition blend
        if self._blend_from is not None:
            if self._exec_idx == 0:
                # Blend over first frame (alpha 0→1)
                # In real client, alpha=t_frac which goes 0→1 within the step
                # For simulation at frame boundaries, use midpoint alpha=0.5
                alpha = 0.5
                target = (1.0 - alpha) * self._blend_from + alpha * target
            else:
                self._blend_from = None

        # 100Hz sub-loop: rate limiter OR legacy interpolator + base vel limiter
        processed = target.copy()
        for tick in range(TICKS_PER_FRAME):
            tick_action = target.copy()

            if self._rate_limiter is not None:
                # G1-style: rate-limited interpolation at 100Hz
                pre_limit = tick_action[6:22].copy()
                self._rate_limiter.set_target(tick_action)
                stepped = self._rate_limiter.step()
                if stepped is not None:
                    tick_action = stepped
                # Track if any joint was actively rate-limited
                if not np.allclose(pre_limit, tick_action[6:22], atol=1e-6):
                    self._rate_limit_clip_ticks += 1
            elif self._action_interpolator is not None and abs_idx < len(chunk) - 1:
                # Legacy: ActionInterpolator lerps between chunk actions
                for wi in range(max(0, abs_idx - 2), min(len(chunk), abs_idx + 4)):
                    self._action_interpolator.update_waypoint(wi, chunk[wi])
                frac_t = abs_idx + tick / TICKS_PER_FRAME
                interp = self._action_interpolator.evaluate(frac_t)
                if interp is not None:
                    tick_action = np.asarray(interp, dtype=np.float64)

            tick_action[0:6] = self._base_limiter.limit(
                np.asarray(tick_action[0:6], dtype=np.float32)
            )
            processed = tick_action

        self._exec_idx += 1
        self._current_frame += 1

        return raw_action, processed.astype(np.float32), {
            'hold': is_hold, 'chunk_id': self._chunk_id,
            'abs_idx': abs_idx, 'promoted': promoted,
        }

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    @property
    def hold_count(self) -> int:
        return self._hold_count

    @property
    def rate_limit_clip_ticks(self) -> int:
        return self._rate_limit_clip_ticks

    @property
    def base_vel_clips(self) -> int:
        return self._base_limiter.vel_clips

    @property
    def base_accel_clips(self) -> int:
        return self._base_limiter.accel_clips

    @property
    def mode_name(self) -> str:
        return 'continuous' if self.continuous else 'classic'


# ---------------------------------------------------------------------------
# Metrics computation
# ---------------------------------------------------------------------------

def compute_metrics(
    gt_actions: np.ndarray,
    pred_actions: np.ndarray,
) -> dict:
    """Compute MSE/MAE metrics overall, per-joint, and per-group."""
    mse_overall = float(np.mean((gt_actions - pred_actions) ** 2))
    mae_overall = float(np.mean(np.abs(gt_actions - pred_actions)))

    per_joint_mse = np.mean((gt_actions - pred_actions) ** 2, axis=0)
    per_joint_mae = np.mean(np.abs(gt_actions - pred_actions), axis=0)

    per_joint_mse_dict = {JOINT_NAMES[i]: float(per_joint_mse[i]) for i in range(22)}
    per_joint_mae_dict = {JOINT_NAMES[i]: float(per_joint_mae[i]) for i in range(22)}

    grouped_mse = {}
    grouped_mae = {}
    for name, start, end in BODY_PART_GROUPS:
        grouped_mse[name] = float(np.mean((gt_actions[:, start:end] - pred_actions[:, start:end]) ** 2))
        grouped_mae[name] = float(np.mean(np.abs(gt_actions[:, start:end] - pred_actions[:, start:end])))

    return {
        'mse_overall': mse_overall,
        'mae_overall': mae_overall,
        'per_joint_mse': per_joint_mse_dict,
        'per_joint_mae': per_joint_mae_dict,
        'grouped_mse': grouped_mse,
        'grouped_mae': grouped_mae,
    }


def compute_pipeline_delta(
    raw_actions: np.ndarray,
    processed_actions: np.ndarray,
    pipeline: ClientPipelineSimulator,
) -> dict:
    """Compute metrics showing the effect of client-side processing."""
    delta = np.abs(processed_actions - raw_actions)

    per_joint_delta = {
        JOINT_NAMES[i]: float(np.mean(delta[:, i])) for i in range(22)
    }
    per_group_delta = {}
    for name, start, end in BODY_PART_GROUPS:
        per_group_delta[name] = float(np.mean(delta[:, start:end]))

    return {
        'mean_delta_overall': float(np.mean(delta)),
        'mean_delta_per_joint': per_joint_delta,
        'mean_delta_per_group': per_group_delta,
        'rate_limit_clip_ticks': pipeline.rate_limit_clip_ticks,
        'base_vel_clip_count': pipeline.base_vel_clips,
        'base_accel_clip_count': pipeline.base_accel_clips,
        'hold_count': pipeline.hold_count,
    }


def compute_smoothness(
    gt_actions: np.ndarray,
    raw_actions: np.ndarray,
    processed_actions: np.ndarray,
    fps: float = 15.0,
) -> dict:
    """Compute trajectory smoothness/noise metrics.

    Compares the noise characteristics of raw and processed predictions
    against the smooth ground truth demonstrations.

    Metrics:
      - jerk_rms: RMS of 2nd derivative (acceleration change). Lower = smoother.
      - delta_rms: RMS of 1st derivative (frame-to-frame change).
      - jerk_ratio: pred_jerk / gt_jerk. 1.0 = same smoothness as GT. >1 = noisier.
      - hf_power_ratio: fraction of FFT power above cutoff (3 Hz). Captures
        high-frequency noise that humans perceive as jitter.

    All computed per body part and overall.
    """
    dt = 1.0 / fps
    hf_cutoff_hz = 3.0  # above this = "high frequency noise"

    def _jerk_rms(traj: np.ndarray) -> float:
        """RMS of 2nd derivative (jerk ≈ d²x/dt²)."""
        if len(traj) < 3:
            return 0.0
        vel = np.diff(traj, axis=0) / dt
        accel = np.diff(vel, axis=0) / dt
        return float(np.sqrt(np.mean(accel ** 2)))

    def _delta_rms(traj: np.ndarray) -> float:
        """RMS of frame-to-frame differences."""
        if len(traj) < 2:
            return 0.0
        diffs = np.diff(traj, axis=0)
        return float(np.sqrt(np.mean(diffs ** 2)))

    def _hf_power_ratio(traj: np.ndarray) -> float:
        """Fraction of FFT power above cutoff frequency."""
        if len(traj) < 4:
            return 0.0
        N = len(traj)
        # Work on each dimension, average ratios
        ratios = []
        ndim = traj.shape[1] if traj.ndim > 1 else 1
        if traj.ndim == 1:
            traj = traj.reshape(-1, 1)
        for d in range(ndim):
            sig = traj[:, d] - np.mean(traj[:, d])  # remove DC
            fft_mag = np.abs(np.fft.rfft(sig))
            freqs = np.fft.rfftfreq(N, d=dt)
            total_power = np.sum(fft_mag ** 2)
            if total_power < 1e-12:
                ratios.append(0.0)
                continue
            hf_mask = freqs >= hf_cutoff_hz
            hf_power = np.sum(fft_mag[hf_mask] ** 2)
            ratios.append(hf_power / total_power)
        return float(np.mean(ratios))

    result = {}

    # Overall metrics
    gt_jerk = _jerk_rms(gt_actions)
    raw_jerk = _jerk_rms(raw_actions)
    proc_jerk = _jerk_rms(processed_actions)

    result['gt_jerk_rms'] = gt_jerk
    result['raw_jerk_rms'] = raw_jerk
    result['processed_jerk_rms'] = proc_jerk
    result['raw_jerk_ratio'] = raw_jerk / gt_jerk if gt_jerk > 1e-9 else 0.0
    result['processed_jerk_ratio'] = proc_jerk / gt_jerk if gt_jerk > 1e-9 else 0.0

    result['gt_delta_rms'] = _delta_rms(gt_actions)
    result['raw_delta_rms'] = _delta_rms(raw_actions)
    result['processed_delta_rms'] = _delta_rms(processed_actions)

    result['raw_hf_power_ratio'] = _hf_power_ratio(raw_actions)
    result['processed_hf_power_ratio'] = _hf_power_ratio(processed_actions)
    result['gt_hf_power_ratio'] = _hf_power_ratio(gt_actions)

    # Per body part
    per_group = {}
    for name, start, end in BODY_PART_GROUPS:
        gt_j = _jerk_rms(gt_actions[:, start:end])
        raw_j = _jerk_rms(raw_actions[:, start:end])
        proc_j = _jerk_rms(processed_actions[:, start:end])
        per_group[name] = {
            'gt_jerk_rms': gt_j,
            'raw_jerk_rms': raw_j,
            'processed_jerk_rms': proc_j,
            'raw_jerk_ratio': raw_j / gt_j if gt_j > 1e-9 else 0.0,
            'processed_jerk_ratio': proc_j / gt_j if gt_j > 1e-9 else 0.0,
            'raw_hf_power_ratio': _hf_power_ratio(raw_actions[:, start:end]),
            'processed_hf_power_ratio': _hf_power_ratio(processed_actions[:, start:end]),
            'gt_hf_power_ratio': _hf_power_ratio(gt_actions[:, start:end]),
        }
    result['per_group'] = per_group

    return result


# ---------------------------------------------------------------------------
# New plots
# ---------------------------------------------------------------------------

def plot_pipeline_delta(
    raw_actions: np.ndarray,
    processed_actions: np.ndarray,
    episode_index: int,
    save_path: str,
):
    """Plot |processed - raw| per body part over time."""
    delta = np.abs(processed_actions - raw_actions)
    N = len(delta)
    frames = np.arange(N)

    fig, axes = plt.subplots(nrows=len(BODY_PART_GROUPS), ncols=1,
                             figsize=(14, 2.5 * SUBPLOT_Y_SCALE * len(BODY_PART_GROUPS)),
                             sharex=True)
    fig.suptitle(f'Episode {episode_index} — Pipeline Delta |processed - raw|',
                 fontsize=14, color='blue')

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']

    for i, (name, start, end) in enumerate(BODY_PART_GROUPS):
        ax = axes[i]
        group_delta = delta[:, start:end]
        for d in range(end - start):
            joint_name = JOINT_NAMES[start + d]
            ax.plot(frames, group_delta[:, d], linewidth=0.8, alpha=0.8,
                    label=joint_name)
        ax.set_ylabel('|delta|')
        ax.set_title(f'{name} (mean={np.mean(group_delta):.6f})', fontsize=10,
                     color=colors[i % len(colors)])
        ax.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=0.2)

    axes[-1].set_xlabel('Frame')
    plt.tight_layout()
    Path(save_path).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(save_path, dpi=100, bbox_inches='tight')
    plt.close()
    logger.info(f'Pipeline delta plot saved: {save_path}')


def plot_comparison(
    gt_actions: np.ndarray,
    raw_actions: np.ndarray,
    processed_actions: np.ndarray,
    states: np.ndarray,
    episode_index: int,
    action_horizon: int,
    save_path: str,
):
    """Overlay raw vs processed vs GT for key joints."""
    # Key joints for quick visual comparison
    key_joints = [
        (0, 'cmd_vel_lx'),
        (5, 'cmd_vel_az'),
        (6, 'back_joint'),
        (14, 'right_shoulder_pitch'),
        (15, 'right_elbow_pitch'),
        (18, 'right_gripper'),
        (19, 'head_yaw'),
    ]

    N = len(gt_actions)
    frames = np.arange(N)

    fig, axes = plt.subplots(nrows=len(key_joints), ncols=1,
                             figsize=(14, 2.5 * SUBPLOT_Y_SCALE * len(key_joints)),
                             sharex=True)
    fig.suptitle(f'Episode {episode_index} — Raw vs Processed vs GT (key joints)',
                 fontsize=14, color='blue')

    for i, (idx, name) in enumerate(key_joints):
        ax = axes[i]
        ax.plot(frames, states[:, idx], color='gray', alpha=0.4, linewidth=1, label='state')
        ax.plot(frames, gt_actions[:, idx], color='black', linewidth=1.2, label='GT')
        ax.plot(frames, raw_actions[:, idx], color='#1f77b4', linewidth=1, linestyle='--',
                alpha=0.8, label='raw pred')
        ax.plot(frames, processed_actions[:, idx], color='#d62728', linewidth=1,
                linestyle=':', alpha=0.9, label='processed')

        # Inference points
        for j in range(0, N, action_horizon):
            kwargs = {'label': 'inference'} if j == 0 else {}
            ax.plot(j, gt_actions[j, idx], 'go', markersize=3, **kwargs)

        ax.set_title(name, fontsize=10)
        if i == 0:
            ax.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=0.2)

    axes[-1].set_xlabel('Frame')
    plt.tight_layout()
    Path(save_path).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(save_path, dpi=100, bbox_inches='tight')
    plt.close()
    logger.info(f'Comparison plot saved: {save_path}')


# ---------------------------------------------------------------------------
# Scaled wrappers for imported plot functions
# ---------------------------------------------------------------------------
# The imported plot_trajectory_results and plot_grouped_results hardcode their
# figsize. These wrappers temporarily monkey-patch matplotlib to apply the
# SUBPLOT_Y_SCALE factor, then restore the original.

def _scaled_plot_trajectory_results(**kwargs):
    """plot_trajectory_results with SUBPLOT_Y_SCALE applied to subplot height."""
    _orig_subplots = plt.subplots

    def _patched_subplots(*args, **kw):
        if 'figsize' in kw:
            w, h = kw['figsize']
            kw['figsize'] = (w, h * SUBPLOT_Y_SCALE)
        return _orig_subplots(*args, **kw)

    plt.subplots = _patched_subplots
    try:
        plot_trajectory_results(**kwargs)
    finally:
        plt.subplots = _orig_subplots


def _scaled_plot_grouped_results(**kwargs):
    """plot_grouped_results with SUBPLOT_Y_SCALE applied to subplot height."""
    _orig_subplots = plt.subplots

    def _patched_subplots(*args, **kw):
        if 'figsize' in kw:
            w, h = kw['figsize']
            kw['figsize'] = (w, h * SUBPLOT_Y_SCALE)
        return _orig_subplots(*args, **kw)

    plt.subplots = _patched_subplots
    try:
        plot_grouped_results(**kwargs)
    finally:
        plt.subplots = _orig_subplots


# ---------------------------------------------------------------------------
# CSV output
# ---------------------------------------------------------------------------

def save_actions_csv(
    save_path: str,
    gt_states: np.ndarray,
    gt_actions: np.ndarray,
    raw_actions: np.ndarray,
    processed_actions: np.ndarray,
    frame_info: list[dict],
):
    """Save per-frame CSV with raw, processed, GT, and state."""
    Path(save_path).parent.mkdir(parents=True, exist_ok=True)

    header = ['frame', 'chunk_id', 'abs_idx', 'hold']
    for prefix in ['raw', 'processed', 'gt_action', 'gt_state']:
        for jn in JOINT_NAMES:
            header.append(f'{prefix}_{jn}')

    with open(save_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        N = len(gt_actions)
        for i in range(N):
            info = frame_info[i] if i < len(frame_info) else {}
            row = [
                i,
                info.get('chunk_id', 0),
                info.get('abs_idx', 0),
                int(info.get('hold', False)),
            ]
            row.extend(raw_actions[i].tolist())
            row.extend(processed_actions[i].tolist())
            row.extend(gt_actions[i].tolist())
            row.extend(gt_states[i].tolist())
            writer.writerow(row)

    logger.info(f'Actions CSV saved: {save_path}')


# ---------------------------------------------------------------------------
# Main evaluation loop
# ---------------------------------------------------------------------------

def run_episode(
    client: ZMQClient,
    pipeline: ClientPipelineSimulator,
    config: dict,
    gt_states: np.ndarray,
    gt_actions: np.ndarray,
    compressed_frames: list[dict],
    task: str,
    closed_loop: bool = False,
) -> dict:
    """Replay one episode through the full pipeline.

    Returns dict with raw_actions, processed_actions, frame_info, timings.
    """
    N = len(gt_states)
    raw_actions = np.full((N, 22), np.nan, dtype=np.float32)
    processed_actions = np.full((N, 22), np.nan, dtype=np.float32)
    frame_info = []

    all_server_timings = []
    all_client_timings = []

    # Closed-loop state tracking
    cl_state = gt_states[0].copy() if closed_loop else None

    pipeline.reset()
    num_inference = 0

    # Prefetch: in continuous open-loop mode, we know the next frame's observation
    # and can ask the server to VLA-prep it while running GPU inference on the current.
    can_prefetch = pipeline.continuous and not closed_loop
    last_prefetch_id = None  # tracks the prefetch ID we submitted last request

    for gt_frame in range(N):
        # Determine observation state
        if closed_loop and cl_state is not None:
            state_raw = cl_state.copy()
        else:
            state_raw = gt_states[gt_frame]

        # Check if we need inference
        if pipeline.needs_inference():
            images = compressed_frames[gt_frame] if gt_frame < len(compressed_frames) else {}

            # Build prefetch args for next frame
            pf_images = None
            pf_state = None
            pf_id = None
            if can_prefetch and gt_frame + 1 < N:
                next_frame = gt_frame + 1
                pf_images = compressed_frames[next_frame] if next_frame < len(compressed_frames) else None
                pf_state = gt_states[next_frame]
                pf_id = next_frame

            response = client.send_observation(
                images=images,
                state=state_raw,
                language=task,
                prefetch_images=pf_images,
                prefetch_state=pf_state,
                prefetch_id=pf_id,
                use_prefetch_id=gt_frame if last_prefetch_id == gt_frame else None,
            )
            last_prefetch_id = pf_id

            if response is None:
                logger.error(f'Inference failed at frame {gt_frame}')
                # Step with whatever we have
                raw, proc, info = pipeline.step_frame()
                raw_actions[gt_frame] = raw
                processed_actions[gt_frame] = proc
                frame_info.append(info)
                continue

            if response.get('status') != 'ok':
                logger.error(f"Server error at frame {gt_frame}: {response.get('error_message')}")
                raw, proc, info = pipeline.step_frame()
                raw_actions[gt_frame] = raw
                processed_actions[gt_frame] = proc
                frame_info.append(info)
                continue

            # Collect timing
            if response.get('server_timing'):
                all_server_timings.append(response['server_timing'])
            if response.get('client_timing'):
                all_client_timings.append(response['client_timing'])

            # Extract and install chunk
            actions_list = response.get('actions', [])
            if actions_list:
                chunk = np.array(actions_list, dtype=np.float32)
                pipeline.install_chunk(chunk)
                num_inference += 1

        # Step the pipeline for this frame
        raw, proc, info = pipeline.step_frame()
        raw_actions[gt_frame] = raw
        processed_actions[gt_frame] = proc
        frame_info.append(info)

        # Closed-loop: propagate processed action as next state
        if closed_loop and cl_state is not None:
            cl_state[0:6] = proc[0:6]    # base velocity
            cl_state[6:22] = proc[6:22]  # joint positions

    return {
        'raw_actions': raw_actions,
        'processed_actions': processed_actions,
        'frame_info': frame_info,
        'server_timings': all_server_timings,
        'client_timings': all_client_timings,
        'num_inference': num_inference,
    }


def run_eval(args):
    """Run replay evaluation for all requested episodes."""
    # Load config from YAML
    config = load_config(args.config)
    config = apply_cli_overrides(config, args)

    # Build server address
    server_address = build_server_address(
        transport=config['transport'],
        host=config['server_host'],
        port=config['server_port'],
    )

    action_chunk_size = int(config['action_chunk_size'])
    task = config['task_description']
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Resolve episode list
    if args.episode_indices:
        episode_indices = [int(x.strip()) for x in args.episode_indices.split(',')]
    else:
        episode_indices = [args.episode_index]

    # Print config banner
    is_continuous = bool(config.get('continuous_inference', False))
    mode_label = 'CONTINUOUS (temporal ensembling)' if is_continuous else 'CLASSIC (chunk promotion)'

    print()
    print('=' * 70)
    print('  GR00T Replay Evaluation (Full Client Pipeline)')
    print('=' * 70)
    print(f'  Config:               {args.config}')
    print(f'  Dataset:              {args.dataset_path}')
    print(f'  Episodes:             {episode_indices}')
    print(f'  Server:               {server_address}')
    print(f'  Task:                 "{task}"')
    print(f'  Closed-Loop:          {args.closed_loop}')
    print(f'  Output:               {output_dir}')
    print(f'  Mode:                 {mode_label}')
    print()
    print('  Pipeline Config:')
    print(f'    action_chunk_size:      {config["action_chunk_size"]}')
    print(f'    latency_skip:           {config["latency_skip"]}')
    if is_continuous:
        print(f'    smoothing_strategy:     {config["smoothing_strategy"]}')
        print(f'    smoothing_decay_m:      {config["smoothing_decay_m"]}')
        print(f'    max_buffer_chunks:      {config["max_buffer_chunks"]}')
        print(f'    base_smoothing_alpha:   {config["base_smoothing_alpha"]}')
        print(f'    joint_smoothing_alpha:  {config["joint_smoothing_alpha"]}')
        print(f'    smoothing_method:       {config["smoothing_method"]}')
        if config["smoothing_method"] == 'savgol':
            print(f'    savgol_window:          {config["savgol_window"]}')
            print(f'    savgol_polyorder:       {config["savgol_polyorder"]}')
        elif config["smoothing_method"] == 'butterworth':
            print(f'    butterworth_order:      {config["butterworth_order"]}')
            print(f'    butterworth_cutoff_hz:  {config["butterworth_cutoff_hz"]}')
        print(f'    interpolate_actions:    {config["interpolate_actions"]}')
        if config.get('interpolate_actions'):
            print(f'    interpolation_method:   {config["interpolation_method"]}')
    else:
        print(f'    n_action_steps:         {config["n_action_steps"]}')
        print(f'    inference_trigger_step: {config["inference_trigger_step"]}')
        print(f'    chunk_blend_steps:      {config["chunk_blend_steps"]}')
        print(f'    rate_limit_enabled:     {config["rate_limit_enabled"]}')
        if config.get('rate_limit_enabled'):
            print(f'    rate_limit_back:        {config["rate_limit_back"]}')
            print(f'    rate_limit_left_arm:    {config["rate_limit_left_arm"]}')
            print(f'    rate_limit_right_arm:   {config["rate_limit_right_arm"]}')
            print(f'    rate_limit_head:        {config["rate_limit_head"]}')
        elif config.get('interpolate_actions'):
            print(f'    interpolate_actions:    {config["interpolate_actions"]}')
            print(f'    interpolation_method:   {config["interpolation_method"]}')
    print(f'    max_base_linear_x:      {config["max_base_linear_x"]}')
    print(f'    max_base_linear_y:      {config["max_base_linear_y"]}')
    print(f'    max_base_angular_z:     {config["max_base_angular_z"]}')
    print(f'    max_base_linear_accel:  {config["max_base_linear_accel"]}')
    print(f'    max_base_angular_accel: {config["max_base_angular_accel"]}')
    print(f'    h264_conditioning:      {config["h264_conditioning"]}')
    print('=' * 70)
    print()

    # Save effective config (JSON-serializable copy)
    json_config = {k: (v if not isinstance(v, np.ndarray) else v.tolist())
                   for k, v in config.items()}
    config_path = output_dir / 'config.json'
    with open(config_path, 'w') as f:
        json.dump(json_config, f, indent=2, default=str)

    # Connect to server
    logger.info(f'Connecting to server at {server_address}...')
    client = ZMQClient(
        server_address=server_address,
        timeout_ms=int(config['inference_timeout_ms']),
        logger=lambda msg: logger.info(msg),
    )

    if not client.connect():
        logger.error(f'Failed to connect to server at {server_address}')
        sys.exit(1)

    if not client.ping(timeout_ms=5000):
        logger.error(f'Server not responding at {server_address}')
        client.close()
        sys.exit(1)

    logger.info(f'Server verified at {server_address}')

    # JPEG quality — use 95 to match live client pipeline (not 80 from open_loop_eval)
    jpeg_quality = 95

    # Build pipeline
    pipeline = ClientPipelineSimulator(config)

    all_episode_metrics = {}
    eval_mode = 'CLOSED-LOOP' if args.closed_loop else 'OPEN-LOOP'

    for episode_index in episode_indices:
        print(f'\n--- Episode {episode_index} ---')
        ep_dir = output_dir / f'episode_{episode_index:03d}'
        ep_dir.mkdir(parents=True, exist_ok=True)

        # Load data
        logger.info(f'Loading episode {episode_index}...')
        gt_states, gt_actions = load_episode(args.dataset_path, episode_index)
        N = len(gt_states)
        logger.info(f'Episode has {N} frames')

        logger.info('Loading video frames...')
        video_frames = load_video_frames(args.dataset_path, episode_index, N)

        # Pre-compress to JPEG
        logger.info(f'Pre-compressing {N} frames to JPEG (quality={jpeg_quality})...')
        t0 = time.time()
        compressed_frames = []
        for frame_dict in video_frames:
            compressed_frames.append(
                {cam: compress_jpeg(img, quality=jpeg_quality) for cam, img in frame_dict.items()}
            )
        logger.info(f'Pre-compression done in {time.time() - t0:.1f}s')

        # Run replay
        logger.info('Running replay evaluation...')
        t0 = time.time()
        result = run_episode(
            client=client,
            pipeline=pipeline,
            config=config,
            gt_states=gt_states,
            gt_actions=gt_actions,
            compressed_frames=compressed_frames,
            task=task,
            closed_loop=args.closed_loop,
        )
        elapsed = time.time() - t0
        logger.info(f'Replay done in {elapsed:.1f}s ({result["num_inference"]} inference steps)')

        raw = result['raw_actions']
        proc = result['processed_actions']

        # Trim NaN rows (if any inference failed at the start)
        valid_mask = ~np.isnan(raw[:, 0])
        if not np.all(valid_mask):
            first_valid = np.argmax(valid_mask)
            raw = raw[first_valid:]
            proc = proc[first_valid:]
            gt_actions_trimmed = gt_actions[first_valid:len(raw) + first_valid]
            gt_states_trimmed = gt_states[first_valid:len(raw) + first_valid]
            frame_info = result['frame_info'][first_valid:]
        else:
            gt_actions_trimmed = gt_actions[:len(raw)]
            gt_states_trimmed = gt_states[:len(raw)]
            frame_info = result['frame_info']

        actual_steps = len(raw)
        logger.info(f'{actual_steps} valid frames')

        if actual_steps == 0:
            logger.error(f'No valid frames for episode {episode_index}, skipping.')
            continue

        # Compute metrics
        raw_metrics = compute_metrics(gt_actions_trimmed, raw)
        proc_metrics = compute_metrics(gt_actions_trimmed, proc)
        delta_metrics = compute_pipeline_delta(raw, proc, pipeline)
        smoothness = compute_smoothness(gt_actions_trimmed, raw, proc)

        # Print summary
        print()
        print(f'  [{eval_mode}] RAW       MSE: {raw_metrics["mse_overall"]:.6f}  '
              f'MAE: {raw_metrics["mae_overall"]:.6f}')
        print(f'  [{eval_mode}] PROCESSED MSE: {proc_metrics["mse_overall"]:.6f}  '
              f'MAE: {proc_metrics["mae_overall"]:.6f}')
        print(f'  Pipeline delta (mean |proc-raw|): {delta_metrics["mean_delta_overall"]:.6f}')
        print(f'  Hold frames: {delta_metrics["hold_count"]}/{actual_steps} '
              f'({100*delta_metrics["hold_count"]/actual_steps:.1f}%)')
        print(f'  Rate-limit clips: {delta_metrics["rate_limit_clip_ticks"]} ticks')
        print(f'  Base vel clips: {delta_metrics["base_vel_clip_count"]}, '
              f'accel clips: {delta_metrics["base_accel_clip_count"]}')
        print()

        # Per-group comparison
        print(f'  {"Group":<12}  {"RAW MSE":>10}  {"PROC MSE":>10}  {"RAW MAE":>10}  {"PROC MAE":>10}  {"Delta":>10}')
        print(f'  {"-"*12}  {"-"*10}  {"-"*10}  {"-"*10}  {"-"*10}  {"-"*10}')
        for name, _, _ in BODY_PART_GROUPS:
            print(f'  {name:<12}  '
                  f'{raw_metrics["grouped_mse"][name]:10.6f}  '
                  f'{proc_metrics["grouped_mse"][name]:10.6f}  '
                  f'{raw_metrics["grouped_mae"][name]:10.6f}  '
                  f'{proc_metrics["grouped_mae"][name]:10.6f}  '
                  f'{delta_metrics["mean_delta_per_group"][name]:10.6f}')
        print()

        # Trajectory smoothness / noise
        print('  TRAJECTORY SMOOTHNESS')
        print(f'  {"":12}  {"Jerk RMS":>10}  {"Jerk Ratio":>10}  {"Delta RMS":>10}  {"HF Power%":>10}')
        print(f'  {"":12}  {"-"*10}  {"-"*10}  {"-"*10}  {"-"*10}')
        print(f'  {"GT":<12}  {smoothness["gt_jerk_rms"]:10.4f}  {"1.00x":>10}  '
              f'{smoothness["gt_delta_rms"]:10.6f}  {100*smoothness["gt_hf_power_ratio"]:9.1f}%')
        print(f'  {"Raw pred":<12}  {smoothness["raw_jerk_rms"]:10.4f}  '
              f'{smoothness["raw_jerk_ratio"]:9.2f}x  '
              f'{smoothness["raw_delta_rms"]:10.6f}  {100*smoothness["raw_hf_power_ratio"]:9.1f}%')
        print(f'  {"Processed":<12}  {smoothness["processed_jerk_rms"]:10.4f}  '
              f'{smoothness["processed_jerk_ratio"]:9.2f}x  '
              f'{smoothness["processed_delta_rms"]:10.6f}  {100*smoothness["processed_hf_power_ratio"]:9.1f}%')
        print()

        # Per-group smoothness
        print(f'  {"Group":<12}  {"GT Jerk":>10}  {"Raw Jerk":>10}  {"Proc Jerk":>10}  '
              f'{"Raw Ratio":>10}  {"Proc Ratio":>10}  {"Raw HF%":>8}  {"Proc HF%":>8}')
        print(f'  {"-"*12}  {"-"*10}  {"-"*10}  {"-"*10}  {"-"*10}  {"-"*10}  {"-"*8}  {"-"*8}')
        for name, _, _ in BODY_PART_GROUPS:
            sg = smoothness['per_group'][name]
            print(f'  {name:<12}  {sg["gt_jerk_rms"]:10.4f}  {sg["raw_jerk_rms"]:10.4f}  '
                  f'{sg["processed_jerk_rms"]:10.4f}  {sg["raw_jerk_ratio"]:9.2f}x  '
                  f'{sg["processed_jerk_ratio"]:9.2f}x  '
                  f'{100*sg["raw_hf_power_ratio"]:7.1f}%  '
                  f'{100*sg["processed_hf_power_ratio"]:7.1f}%')
        print()

        # Comms diagnostics
        latencies = client.get_latency_history()
        message_sizes = client.get_message_size_history()
        if latencies:
            print_comms_summary(latencies, message_sizes)
        if result['server_timings'] or result['client_timings']:
            print_timing_breakdown(result['server_timings'], result['client_timings'])

        # Determine inference step size for plot markers
        # The effective horizon is n_action_steps (how many GT frames per chunk)
        n_steps = int(config['n_action_steps'])

        # Generate plots
        if not args.no_plots:
            # Raw trajectory
            _scaled_plot_trajectory_results(
                state_across_time=gt_states_trimmed,
                gt_action_across_time=gt_actions_trimmed,
                pred_action_across_time=raw,
                episode_index=episode_index,
                action_horizon=n_steps,
                save_plot_path=str(ep_dir / 'raw_trajectory.png'),
                eval_mode=f'{eval_mode} RAW',
            )

            # Raw grouped
            _scaled_plot_grouped_results(
                state_across_time=gt_states_trimmed,
                gt_action_across_time=gt_actions_trimmed,
                pred_action_across_time=raw,
                episode_index=episode_index,
                action_horizon=n_steps,
                save_plot_path=str(ep_dir / 'raw_trajectory_grouped.png'),
                eval_mode=f'{eval_mode} RAW',
            )

            # Processed trajectory
            _scaled_plot_trajectory_results(
                state_across_time=gt_states_trimmed,
                gt_action_across_time=gt_actions_trimmed,
                pred_action_across_time=proc,
                episode_index=episode_index,
                action_horizon=n_steps,
                save_plot_path=str(ep_dir / 'processed_trajectory.png'),
                eval_mode=f'{eval_mode} PROCESSED',
            )

            # Processed grouped
            _scaled_plot_grouped_results(
                state_across_time=gt_states_trimmed,
                gt_action_across_time=gt_actions_trimmed,
                pred_action_across_time=proc,
                episode_index=episode_index,
                action_horizon=n_steps,
                save_plot_path=str(ep_dir / 'processed_trajectory_grouped.png'),
                eval_mode=f'{eval_mode} PROCESSED',
            )

            # Pipeline delta
            plot_pipeline_delta(
                raw_actions=raw,
                processed_actions=proc,
                episode_index=episode_index,
                save_path=str(ep_dir / 'pipeline_delta.png'),
            )

            # Raw vs processed comparison overlay
            plot_comparison(
                gt_actions=gt_actions_trimmed,
                raw_actions=raw,
                processed_actions=proc,
                states=gt_states_trimmed,
                episode_index=episode_index,
                action_horizon=n_steps,
                save_path=str(ep_dir / 'comparison.png'),
            )

            # Comms report
            if latencies:
                plot_comms_report(latencies, message_sizes,
                                 str(ep_dir / 'comms.png'))

        # Save CSV
        if args.save_csv:
            save_actions_csv(
                save_path=str(ep_dir / 'actions.csv'),
                gt_states=gt_states_trimmed,
                gt_actions=gt_actions_trimmed,
                raw_actions=raw,
                processed_actions=proc,
                frame_info=frame_info,
            )

        # Collect comms metrics
        comms_metrics = {}
        if latencies:
            lat_arr = np.array(latencies)
            comms_metrics = {
                'mean_latency_ms': float(np.mean(lat_arr)),
                'median_latency_ms': float(np.median(lat_arr)),
                'p95_latency_ms': float(np.percentile(lat_arr, 95)),
                'min_latency_ms': float(np.min(lat_arr)),
                'max_latency_ms': float(np.max(lat_arr)),
            }
            if message_sizes:
                comms_metrics['mean_send_kb'] = float(np.mean([s for s, _ in message_sizes]) / 1024)
                comms_metrics['mean_recv_kb'] = float(np.mean([r for _, r in message_sizes]) / 1024)

        all_episode_metrics[str(episode_index)] = {
            'num_frames': actual_steps,
            'num_inference_steps': result['num_inference'],
            'raw_metrics': raw_metrics,
            'processed_metrics': proc_metrics,
            'pipeline_delta': delta_metrics,
            'smoothness': smoothness,
            'comms': comms_metrics,
        }

    client.close()

    # Aggregate metrics across episodes
    aggregate = {}
    if all_episode_metrics:
        all_raw_mse = [m['raw_metrics']['mse_overall'] for m in all_episode_metrics.values()]
        all_raw_mae = [m['raw_metrics']['mae_overall'] for m in all_episode_metrics.values()]
        all_proc_mse = [m['processed_metrics']['mse_overall'] for m in all_episode_metrics.values()]
        all_proc_mae = [m['processed_metrics']['mae_overall'] for m in all_episode_metrics.values()]
        all_delta = [m['pipeline_delta']['mean_delta_overall'] for m in all_episode_metrics.values()]
        all_raw_jerk = [m['smoothness']['raw_jerk_rms'] for m in all_episode_metrics.values()]
        all_proc_jerk = [m['smoothness']['processed_jerk_rms'] for m in all_episode_metrics.values()]
        all_gt_jerk = [m['smoothness']['gt_jerk_rms'] for m in all_episode_metrics.values()]
        all_raw_hf = [m['smoothness']['raw_hf_power_ratio'] for m in all_episode_metrics.values()]
        all_proc_hf = [m['smoothness']['processed_hf_power_ratio'] for m in all_episode_metrics.values()]

        mean_gt_jerk = float(np.mean(all_gt_jerk))
        mean_raw_jerk = float(np.mean(all_raw_jerk))
        mean_proc_jerk = float(np.mean(all_proc_jerk))

        aggregate = {
            'num_episodes': len(all_episode_metrics),
            'raw_mse_mean': float(np.mean(all_raw_mse)),
            'raw_mae_mean': float(np.mean(all_raw_mae)),
            'processed_mse_mean': float(np.mean(all_proc_mse)),
            'processed_mae_mean': float(np.mean(all_proc_mae)),
            'pipeline_delta_mean': float(np.mean(all_delta)),
            'gt_jerk_rms_mean': mean_gt_jerk,
            'raw_jerk_rms_mean': mean_raw_jerk,
            'processed_jerk_rms_mean': mean_proc_jerk,
            'raw_jerk_ratio_mean': mean_raw_jerk / mean_gt_jerk if mean_gt_jerk > 1e-9 else 0.0,
            'processed_jerk_ratio_mean': mean_proc_jerk / mean_gt_jerk if mean_gt_jerk > 1e-9 else 0.0,
            'raw_hf_power_ratio_mean': float(np.mean(all_raw_hf)),
            'processed_hf_power_ratio_mean': float(np.mean(all_proc_hf)),
        }

        print()
        print('=' * 70)
        print('  AGGREGATE RESULTS')
        print('=' * 70)
        print(f'  Episodes evaluated: {aggregate["num_episodes"]}')
        print(f'  RAW       MSE: {aggregate["raw_mse_mean"]:.6f}  MAE: {aggregate["raw_mae_mean"]:.6f}')
        print(f'  PROCESSED MSE: {aggregate["processed_mse_mean"]:.6f}  MAE: {aggregate["processed_mae_mean"]:.6f}')
        print(f'  Pipeline delta: {aggregate["pipeline_delta_mean"]:.6f}')
        print()
        print(f'  Smoothness (jerk RMS):')
        print(f'    GT:        {aggregate["gt_jerk_rms_mean"]:.4f}')
        print(f'    Raw:       {aggregate["raw_jerk_rms_mean"]:.4f}  ({aggregate["raw_jerk_ratio_mean"]:.2f}x GT)')
        print(f'    Processed: {aggregate["processed_jerk_rms_mean"]:.4f}  ({aggregate["processed_jerk_ratio_mean"]:.2f}x GT)')
        print(f'  High-freq noise (power >3Hz):')
        print(f'    Raw:       {100*aggregate["raw_hf_power_ratio_mean"]:.1f}%')
        print(f'    Processed: {100*aggregate["processed_hf_power_ratio_mean"]:.1f}%')
        print('=' * 70)
        print()

    # Save summary JSON
    summary = {
        'config': json_config,
        'eval_mode': eval_mode,
        'episodes': all_episode_metrics,
        'aggregate': aggregate,
    }
    summary_path = output_dir / 'summary.json'
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2, default=str)
    logger.info(f'Summary saved: {summary_path}')

    print(f'Results saved to: {output_dir}')


def parse_args():
    parser = argparse.ArgumentParser(
        description='Replay evaluation through the full GR00T client pipeline',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic evaluation with default config
  python groot_replay_eval.py \\
      --config config/groot_client.yaml \\
      --dataset-path data/alfiebot.CanDoChallenge \\
      --episode-index 0 --host 192.168.50.201

  # Compare rate limiting on vs off
  python groot_replay_eval.py ... --output-dir /tmp/eval_rl_on/
  python groot_replay_eval.py ... --rate-limit-enabled false --output-dir /tmp/eval_rl_off/

  # Multiple episodes
  python groot_replay_eval.py ... --episode-indices 0,1,2,3,4
""",
    )

    # Required
    parser.add_argument('--config', required=True,
                        help='Path to groot_client.yaml')
    parser.add_argument('--dataset-path', required=True,
                        help='Path to LeRobot dataset')

    # Episode selection
    parser.add_argument('--episode-index', type=int, default=0,
                        help='Single episode index (default: 0)')
    parser.add_argument('--episode-indices', type=str, default=None,
                        help='Comma-separated episode indices (overrides --episode-index)')

    # Output
    parser.add_argument('--output-dir', default='/tmp/groot_replay_eval',
                        help='Output directory (default: /tmp/groot_replay_eval)')
    parser.add_argument('--save-csv', action='store_true',
                        help='Save per-frame action CSV')
    parser.add_argument('--no-plots', action='store_true',
                        help='Disable plot generation')
    parser.add_argument('--closed-loop', action='store_true',
                        help='Propagate processed actions as next state')

    # Server connection overrides
    parser.add_argument('--host', type=str, default=None,
                        help='Server host (overrides YAML)')
    parser.add_argument('--port', type=int, default=None,
                        help='Server port (overrides YAML)')
    parser.add_argument('--transport', type=str, default=None,
                        choices=['tcp', 'ipc'],
                        help='Transport (overrides YAML)')
    parser.add_argument('--timeout-ms', type=int, default=None,
                        help='ZMQ inference timeout in ms (overrides YAML)')

    # Pipeline parameter overrides
    parser.add_argument('--task', type=str, default=None,
                        help='Task description (overrides YAML)')
    parser.add_argument('--n-action-steps', type=int, default=None,
                        help='Actions per chunk to execute')
    parser.add_argument('--latency-skip', type=int, default=None,
                        help='Skip first N actions on chunk promotion')
    parser.add_argument('--inference-trigger-step', type=int, default=None,
                        help='Fire inference at this step within execution window')
    parser.add_argument('--chunk-blend-steps', type=int, default=None,
                        help='Blend steps at chunk transitions')

    # Rate limiting overrides
    parser.add_argument('--rate-limit-enabled', type=_str_to_bool, default=None,
                        help='Enable rate-limited interpolation')
    parser.add_argument('--rate-limit-back', type=float, default=None)
    parser.add_argument('--rate-limit-left-arm', type=float, default=None)
    parser.add_argument('--rate-limit-left-gripper', type=float, default=None)
    parser.add_argument('--rate-limit-right-arm', type=float, default=None)
    parser.add_argument('--rate-limit-right-gripper', type=float, default=None)
    parser.add_argument('--rate-limit-head', type=float, default=None)

    # Base velocity overrides
    parser.add_argument('--max-base-linear-x', type=float, default=None)
    parser.add_argument('--max-base-linear-y', type=float, default=None)
    parser.add_argument('--max-base-angular-z', type=float, default=None)
    parser.add_argument('--max-base-linear-accel', type=float, default=None)
    parser.add_argument('--max-base-angular-accel', type=float, default=None)

    # Other overrides
    parser.add_argument('--h264-conditioning', type=_str_to_bool, default=None,
                        help='Apply H.264 conditioning to images')
    parser.add_argument('--continuous-inference', type=_str_to_bool, default=None,
                        help='Enable continuous inference with temporal ensembling')
    parser.add_argument('--smoothing-strategy', type=str, default=None,
                        choices=['latest', 'uniform', 'recency', 'exp_decay', 'triangle'],
                        help='Temporal ensembling strategy (continuous mode)')
    parser.add_argument('--smoothing-decay-m', type=float, default=None)
    parser.add_argument('--max-buffer-chunks', type=int, default=None)
    parser.add_argument('--base-smoothing-alpha', type=float, default=None,
                        help='EMA alpha for base velocity (1.0=off)')
    parser.add_argument('--joint-smoothing-alpha', type=float, default=None,
                        help='EMA alpha for joints (1.0=off)')
    parser.add_argument('--smoothing-method', type=str, default=None,
                        choices=['none', 'savgol', 'butterworth'],
                        help='Post-ensembling filter')
    parser.add_argument('--savgol-window', type=int, default=None)
    parser.add_argument('--savgol-polyorder', type=int, default=None)
    parser.add_argument('--butterworth-order', type=int, default=None)
    parser.add_argument('--butterworth-cutoff-hz', type=float, default=None)
    parser.add_argument('--interpolate-actions', type=_str_to_bool, default=None,
                        help='Enable inter-action interpolation')
    parser.add_argument('--interpolation-method', type=str, default=None,
                        choices=['linear', 'cubic_spline'])
    parser.add_argument('--spline-window', type=int, default=None)

    # Logging
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Verbose logging')

    return parser.parse_args()


def _str_to_bool(v: str) -> bool:
    """Parse boolean CLI arguments."""
    if isinstance(v, bool):
        return v
    if v.lower() in ('true', '1', 'yes'):
        return True
    if v.lower() in ('false', '0', 'no'):
        return False
    raise argparse.ArgumentTypeError(f'Boolean value expected, got {v!r}')


def main():
    args = parse_args()

    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S',
    )

    run_eval(args)


if __name__ == '__main__':
    main()
