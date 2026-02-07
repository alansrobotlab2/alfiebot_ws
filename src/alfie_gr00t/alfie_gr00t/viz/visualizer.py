"""Gradio-based visualizer for GR00T inference inputs and outputs.

Displays camera feeds, state vectors, action horizons, and inference
statistics in a browser-accessible UI. Designed as an external importable
class with no ROS2 dependency.

Usage:
    from alfie_gr00t.viz import GrootVisualizer

    viz = GrootVisualizer(enable=True, port=7860)

    # In your inference loop:
    viz.update(observation=obs_dict, response=resp_dict)

    # On shutdown:
    viz.close()
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np

try:
    import gradio as gr
    import plotly.graph_objects as go
    GRADIO_AVAILABLE = True
except ImportError:
    gr = None
    go = None
    GRADIO_AVAILABLE = False


# Joint names matching the 22D state/action vector layout
JOINT_NAMES = [
    'cmd_vel_lx', 'cmd_vel_ly', 'cmd_vel_lz',
    'cmd_vel_ax', 'cmd_vel_ay', 'cmd_vel_az',
    'back_joint',
    'left_shoulder_yaw', 'left_shoulder_pitch', 'left_elbow_pitch',
    'left_wrist_pitch', 'left_wrist_roll', 'left_gripper',
    'right_shoulder_yaw', 'right_shoulder_pitch', 'right_elbow_pitch',
    'right_wrist_pitch', 'right_wrist_roll', 'right_gripper',
    'head_yaw', 'head_pitch', 'head_roll',
]

# Joint groups: name -> (start_idx, end_idx, plotly_color)
JOINT_GROUPS = {
    'Base velocity': (0, 6, '#6495ED'),
    'Back':          (6, 7, '#66CC66'),
    'Left arm':      (7, 12, '#FF6666'),
    'Left gripper':  (12, 13, '#FF99CC'),
    'Right arm':     (13, 18, '#FFCC44'),
    'Right gripper': (18, 19, '#FFaa44'),
    'Head':          (19, 22, '#CCCCCC'),
}

CAMERA_NAMES = ['left_wide', 'right_wide', 'left_center', 'right_center']


def _decode_image(img) -> Optional[np.ndarray]:
    """Decode an image from JPEG bytes or pass through numpy array."""
    if img is None:
        return None
    if isinstance(img, np.ndarray):
        if img.ndim == 3 and img.shape[2] == 3:
            return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img
    if isinstance(img, (bytes, bytearray)):
        arr = np.frombuffer(img, np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is not None:
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return None


def _build_state_plot(state: list) -> go.Figure:
    """Build a bar chart of the 22D state vector, color-coded by joint group."""
    colors = []
    for i in range(len(JOINT_NAMES)):
        color = '#888888'
        for _, (start, end, c) in JOINT_GROUPS.items():
            if start <= i < end:
                color = c
                break
        colors.append(color)

    fig = go.Figure(data=[
        go.Bar(
            x=JOINT_NAMES,
            y=state,
            marker_color=colors,
        )
    ])
    fig.update_layout(
        title='State Vector (22D)',
        xaxis_title='Joint',
        yaxis_title='Normalized Value',
        yaxis_range=[-3, 3],
        height=300,
        margin=dict(l=40, r=20, t=40, b=80),
        xaxis_tickangle=-45,
        template='plotly_dark',
    )
    fig.add_hline(y=0, line_dash='dash', line_color='white', opacity=0.3)
    return fig


def _build_action_plot(actions: list) -> go.Figure:
    """Build line plots of the action horizon, grouped by body part."""
    actions_arr = np.array(actions)  # (16, 22)
    num_steps = actions_arr.shape[0]
    x = list(range(num_steps))

    fig = go.Figure()

    for group_name, (start, end, color) in JOINT_GROUPS.items():
        # Average across joints in the group for readability
        group_mean = actions_arr[:, start:end].mean(axis=1)
        fig.add_trace(go.Scatter(
            x=x,
            y=group_mean.tolist(),
            mode='lines+markers',
            name=group_name,
            line=dict(color=color, width=2),
            marker=dict(size=4),
        ))

    fig.update_layout(
        title='Action Horizon (16 steps)',
        xaxis_title='Timestep',
        yaxis_title='Normalized Value',
        yaxis_range=[-3, 3],
        height=300,
        margin=dict(l=40, r=20, t=40, b=40),
        template='plotly_dark',
        legend=dict(
            orientation='h',
            yanchor='bottom',
            y=1.02,
            xanchor='right',
            x=1,
        ),
    )
    fig.add_hline(y=0, line_dash='dash', line_color='white', opacity=0.3)
    return fig


def _empty_state_plot() -> go.Figure:
    return _build_state_plot([0.0] * 22)


def _empty_action_plot() -> go.Figure:
    return _build_action_plot([[0.0] * 22] * 16)


def _placeholder_image() -> np.ndarray:
    """Return a dark placeholder image."""
    img = np.zeros((280, 320, 3), dtype=np.uint8)
    return img


class GrootVisualizer:
    """Gradio-based visualizer for GR00T inference I/O.

    When `enable=False`, all methods are no-ops with zero overhead.
    When enabled, launches a Gradio web server in a background thread
    that can be accessed at http://<host>:<port>.

    Args:
        enable: Whether to activate the visualizer.
        port: Port for the Gradio web server.
        share: Whether to create a public Gradio share link.
    """

    def __init__(
        self,
        enable: bool = True,
        port: int = 7860,
        share: bool = False,
    ):
        self._enabled = enable and GRADIO_AVAILABLE

        if not enable:
            return

        if not GRADIO_AVAILABLE:
            print(
                'Warning: gradio not installed. '
                'Install with: pip install gradio. '
                'Visualizer disabled.'
            )
            return

        self._lock = threading.Lock()
        self._port = port
        self._share = share

        # Latest data
        self._images: dict[str, Optional[np.ndarray]] = {
            name: None for name in CAMERA_NAMES
        }
        self._state: list = [0.0] * 22
        self._actions: list = [[0.0] * 22] * 16
        self._task: str = ''
        self._inference_time_ms: float = 0.0
        self._status: str = 'waiting'
        self._frame_count: int = 0
        self._update_count: int = 0

        # Build and launch UI
        self._demo = self._build_ui()
        self._demo.launch(
            server_name='0.0.0.0',
            server_port=self._port,
            share=self._share,
            prevent_thread_lock=True,
            quiet=True,
            theme=gr.themes.Monochrome(),
        )

    def _build_ui(self) -> 'gr.Blocks':
        """Build the Gradio Blocks layout."""
        with gr.Blocks(
            title='GR00T Inference Visualizer',
        ) as demo:
            gr.Markdown('# GR00T Inference Visualizer')

            # Camera feeds
            with gr.Row():
                cam_components = {}
                for name in CAMERA_NAMES:
                    cam_components[name] = gr.Image(
                        label=name,
                        height=280,
                    )

            # Task description
            task_box = gr.Textbox(
                label='Task Description',
                interactive=False,
            )

            # State and action plots
            with gr.Row():
                state_plot = gr.Plot(label='State Vector')
                action_plot = gr.Plot(label='Action Horizon')

            # Stats
            stats_box = gr.Textbox(
                label='Inference Stats',
                interactive=False,
            )

            # Timer-driven refresh
            timer = gr.Timer(value=0.25)

            # All outputs for the refresh function
            outputs = (
                [cam_components[n] for n in CAMERA_NAMES]
                + [task_box, state_plot, action_plot, stats_box]
            )

            timer.tick(fn=self._refresh, outputs=outputs)

        return demo

    def _refresh(self):
        """Called by Gradio Timer to update all components."""
        t0 = time.monotonic()

        with self._lock:
            images = []
            for name in CAMERA_NAMES:
                img = self._images.get(name)
                if img is not None:
                    images.append(img)
                else:
                    images.append(_placeholder_image())

            state = list(self._state)
            actions = [list(a) for a in self._actions]
            task = self._task
            inference_ms = self._inference_time_ms
            status = self._status
            frame = self._frame_count
            update_count = self._update_count

        t_lock = time.monotonic()

        state_fig = _build_state_plot(state)
        t_state = time.monotonic()

        action_fig = _build_action_plot(actions)
        t_action = time.monotonic()

        self._refresh_count = getattr(self, '_refresh_count', 0) + 1
        total_ms = (t_action - t0) * 1000
        lock_ms = (t_lock - t0) * 1000
        state_ms = (t_state - t_lock) * 1000
        action_ms = (t_action - t_state) * 1000

        # Log every 10th refresh to avoid spam
        if self._refresh_count % 10 == 1:
            import logging
            logger = logging.getLogger('groot_viz')
            logger.info(
                f'[viz refresh #{self._refresh_count}] '
                f'total={total_ms:.1f}ms '
                f'(lock={lock_ms:.1f}ms, state_plot={state_ms:.1f}ms, '
                f'action_plot={action_ms:.1f}ms) | '
                f'data_updates={update_count}, frame={frame}'
            )

        stats_text = (
            f'Inference: {inference_ms:.1f}ms | '
            f'Frame: {frame} | '
            f'Status: {status} | '
            f'Viz refresh: {total_ms:.0f}ms'
        )

        return images + [task, state_fig, action_fig, stats_text]

    def update(
        self,
        observation: Optional[dict] = None,
        response: Optional[dict] = None,
    ) -> None:
        """Update the visualizer with new observation and/or response data.

        Args:
            observation: Dict with keys:
                - 'images': dict mapping camera name to JPEG bytes or numpy array
                - 'state': list of 22 floats (normalized state vector)
                - 'language': str (task description)
            response: Dict with keys:
                - 'actions': list of 16 lists of 22 floats
                - 'inference_time_ms': float
                - 'status': str ('ok' or 'error')
        """
        if not self._enabled:
            return

        with self._lock:
            if observation is not None:
                # Update images
                images = observation.get('images', {})
                for name in CAMERA_NAMES:
                    if name in images:
                        decoded = _decode_image(images[name])
                        if decoded is not None:
                            self._images[name] = decoded

                # Update state
                state = observation.get('state')
                if state is not None:
                    if isinstance(state, np.ndarray):
                        self._state = state.tolist()
                    else:
                        self._state = list(state)

                # Update task
                language = observation.get('language')
                if language is not None:
                    self._task = str(language)

            if response is not None:
                # Update actions
                actions = response.get('actions')
                if actions is not None:
                    if isinstance(actions, np.ndarray):
                        self._actions = actions.tolist()
                    else:
                        self._actions = actions

                # Update stats
                inference_ms = response.get('inference_time_ms')
                if inference_ms is not None:
                    self._inference_time_ms = float(inference_ms)

                status = response.get('status')
                if status is not None:
                    self._status = str(status)

            self._frame_count += 1
            self._update_count += 1

            # Log every 50th update to track data inflow rate
            if self._update_count % 50 == 1:
                import logging
                logger = logging.getLogger('groot_viz')
                logger.info(
                    f'[viz update #{self._update_count}] '
                    f'images={list(observation.get("images", {}).keys()) if observation else "none"}, '
                    f'frame={self._frame_count}'
                )

    def close(self) -> None:
        """Shut down the Gradio server."""
        if not self._enabled:
            return
        try:
            self._demo.close()
        except Exception:
            pass

    @property
    def enabled(self) -> bool:
        """Whether the visualizer is active."""
        return self._enabled
