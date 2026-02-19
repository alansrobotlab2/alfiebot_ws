"""Async ZeroMQ client using PUSH/PULL for decoupled observation/action flow.

Observations are pushed non-blocking to the server. Action results arrive
asynchronously on a background receiver thread and are stored in a
single-slot buffer (latest overwrites previous).

The existing REQ/REP ZMQClient is still used for open-loop eval and ping.
This async client is used for live robot inference where decoupled
observation delivery enables server-side preprocessing overlap.
"""

import threading
import time
from collections import deque
from typing import Any, Callable, Optional

import numpy as np
import zmq

from .zmq_client import (
    DEFAULT_IPC_PATH,
    build_server_address,
    msg_from_bytes,
    msg_to_bytes,
)


class ZMQAsyncClient:
    """Async ZMQ client for GR00T inference server.

    Uses PUSH socket to send observations and PULL socket to receive
    action results. Communication is fully decoupled: the client never
    blocks waiting for a response.

    A background receiver thread polls the PULL socket and stores the
    latest action result in a single-slot buffer (same pattern as
    ObservationBridge).

    Each observation is tagged with a monotonic obs_id so the client
    can track which observation produced which action result.
    """

    def __init__(
        self,
        push_address: str,
        pull_address: str,
        req_address: str = '',
        timeout_ms: int = 100,
        logger: Optional[Callable[[str], None]] = None,
    ):
        """Initialize async ZMQ client.

        Args:
            push_address: ZMQ address for PUSH socket (client → server observations).
            pull_address: ZMQ address for PULL socket (server → client actions).
            req_address: ZMQ address for REQ socket (ping/health checks only).
            timeout_ms: Receive timeout for PULL socket polling.
            logger: Optional logging function.
        """
        self._push_address = push_address
        self._pull_address = pull_address
        self._req_address = req_address
        self._timeout_ms = timeout_ms
        self._log = logger or (lambda msg: None)

        # ZMQ context and sockets
        self._context: Optional[zmq.Context] = None
        self._push_socket: Optional[zmq.Socket] = None
        self._pull_socket: Optional[zmq.Socket] = None

        # Connection state
        self._connected = False

        # Monotonic observation counter
        self._obs_id = 0

        # Single-slot result buffer (latest overwrites)
        self._latest_result: Optional[dict] = None
        self._result_lock = threading.Lock()
        self._result_event = threading.Event()

        # Receiver thread
        self._running = False
        self._receiver_thread: Optional[threading.Thread] = None

        # Statistics
        self._push_count = 0
        self._recv_count = 0
        self._latency_history: deque = deque(maxlen=100)
        self._obs_timestamps: dict[int, float] = {}  # obs_id → push time

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def obs_id(self) -> int:
        return self._obs_id

    @property
    def average_latency_ms(self) -> float:
        if not self._latency_history:
            return 0.0
        return sum(self._latency_history) / len(self._latency_history)

    def connect(self) -> bool:
        """Connect PUSH and PULL sockets and start receiver thread."""
        try:
            if self._context is None:
                self._context = zmq.Context()

            # PUSH socket: send observations (non-blocking)
            self._push_socket = self._context.socket(zmq.PUSH)
            self._push_socket.setsockopt(zmq.SNDTIMEO, self._timeout_ms)
            self._push_socket.setsockopt(zmq.LINGER, 0)
            self._push_socket.setsockopt(zmq.SNDHWM, 1)  # drop old if server is slow
            self._push_socket.connect(self._push_address)

            # PULL socket: receive actions (polled by receiver thread)
            self._pull_socket = self._context.socket(zmq.PULL)
            self._pull_socket.setsockopt(zmq.RCVTIMEO, self._timeout_ms)
            self._pull_socket.setsockopt(zmq.LINGER, 0)
            self._pull_socket.setsockopt(zmq.RCVHWM, 1)
            self._pull_socket.connect(self._pull_address)

            # Start receiver thread
            self._running = True
            self._receiver_thread = threading.Thread(
                target=self._receiver_loop,
                name='zmq_async_receiver',
                daemon=True,
            )
            self._receiver_thread.start()

            self._connected = True
            self._log(
                f'Async ZMQ connected: PUSH→{self._push_address}, '
                f'PULL←{self._pull_address}'
            )
            return True

        except zmq.ZMQError as e:
            self._log(f'Failed to connect async ZMQ: {e}')
            self._connected = False
            return False

    def close(self):
        """Stop receiver thread and close all sockets."""
        self._running = False
        self._connected = False

        if self._receiver_thread is not None and self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=2.0)
            self._receiver_thread = None

        if self._push_socket is not None:
            self._push_socket.close(linger=0)
            self._push_socket = None

        if self._pull_socket is not None:
            self._pull_socket.close(linger=0)
            self._pull_socket = None

        if self._context is not None:
            self._context.term()
            self._context = None

    def push_observation(
        self,
        images: dict[str, bytes],
        state: np.ndarray,
        language: str,
    ) -> int:
        """Push observation to server (non-blocking).

        Args:
            images: Dictionary mapping camera names to JPEG bytes.
            state: State vector (22D).
            language: Task description string.

        Returns:
            obs_id assigned to this observation, or -1 on failure.
        """
        if not self._connected or self._push_socket is None:
            return -1

        obs_id = self._obs_id
        self._obs_id += 1

        observation = {
            'obs_id': obs_id,
            'timestamp': time.monotonic(),
            **images,
            'state': state.tolist() if isinstance(state, np.ndarray) else state,
            'language': language,
        }

        try:
            packed = msg_to_bytes({
                'endpoint': 'get_action',
                'data': {'observation': observation},
            })
            self._push_socket.send(packed, zmq.NOBLOCK)
            self._obs_timestamps[obs_id] = time.monotonic()
            self._push_count += 1

            # Prune old timestamps (keep last 100)
            if len(self._obs_timestamps) > 100:
                oldest = min(self._obs_timestamps.keys())
                del self._obs_timestamps[oldest]

            return obs_id

        except zmq.Again:
            self._log('PUSH socket full — observation dropped')
            return -1
        except zmq.ZMQError as e:
            self._log(f'PUSH error: {e}')
            return -1

    def pop_result(self) -> Optional[dict]:
        """Pop latest action result (non-blocking).

        Returns:
            Action result dict with 'obs_id', 'actions', 'server_timing',
            or None if no new result available.
        """
        with self._result_lock:
            result = self._latest_result
            self._latest_result = None
        return result

    def wait_for_result(self, timeout_s: float = 1.0) -> Optional[dict]:
        """Block until a result arrives or timeout.

        Useful for first-chunk acquisition where the client has no
        actions to execute while waiting.

        Args:
            timeout_s: Maximum time to wait in seconds.

        Returns:
            Action result dict, or None on timeout.
        """
        self._result_event.clear()
        self._result_event.wait(timeout=timeout_s)
        return self.pop_result()

    def _receiver_loop(self):
        """Background thread: poll PULL socket, store latest result."""
        while self._running:
            try:
                packed = self._pull_socket.recv()
                response = msg_from_bytes(packed)
                now = time.monotonic()

                # Parse response — server sends the same format as REQ/REP
                # but with obs_id added
                result = self._parse_response(response, now)
                if result is not None:
                    with self._result_lock:
                        self._latest_result = result
                    self._result_event.set()
                    self._recv_count += 1

            except zmq.Again:
                continue  # timeout — poll again
            except zmq.ZMQError:
                if not self._running:
                    break
                continue

    def _parse_response(self, response: Any, recv_time: float) -> Optional[dict]:
        """Parse server response into standardized result dict."""
        # Server wraps in the PolicyServer format: (action_dict, info)
        if isinstance(response, (list, tuple)):
            action_dict = response[0] if len(response) > 0 else {}
            info = response[1] if len(response) > 1 else {}

            obs_id = info.get('obs_id', -1)

            # Compute end-to-end latency
            push_time = self._obs_timestamps.get(obs_id)
            latency_ms = (recv_time - push_time) * 1000 if push_time else 0.0
            if latency_ms > 0:
                self._latency_history.append(latency_ms)

            return {
                'obs_id': obs_id,
                'actions': action_dict.get('actions', []),
                'server_timing': info.get('server_timing', {}),
                'latency_ms': latency_ms,
                'status': 'ok',
            }
        elif isinstance(response, dict):
            obs_id = response.get('obs_id', -1)
            push_time = self._obs_timestamps.get(obs_id)
            latency_ms = (recv_time - push_time) * 1000 if push_time else 0.0
            if latency_ms > 0:
                self._latency_history.append(latency_ms)

            return {
                'obs_id': obs_id,
                'actions': response.get('actions', []),
                'server_timing': response.get('server_timing', {}),
                'latency_ms': latency_ms,
                'status': 'ok',
            }

        return None

    def ping(self, timeout_ms: int = 3000) -> bool:
        """Ping server via a temporary REQ socket.

        Uses the REQ/REP port (not PUSH/PULL) for health checking.
        """
        if not self._req_address:
            return self._connected

        try:
            ctx = zmq.Context()
            sock = ctx.socket(zmq.REQ)
            sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
            sock.setsockopt(zmq.SNDTIMEO, timeout_ms)
            sock.setsockopt(zmq.LINGER, 0)
            sock.connect(self._req_address)

            sock.send(msg_to_bytes({'endpoint': 'ping'}))
            response = msg_from_bytes(sock.recv())

            sock.close(linger=0)
            ctx.term()
            return response is not None

        except (zmq.ZMQError, Exception):
            return False

    def get_stats(self) -> dict:
        """Get client statistics."""
        return {
            'connected': self._connected,
            'push_address': self._push_address,
            'pull_address': self._pull_address,
            'obs_id': self._obs_id,
            'push_count': self._push_count,
            'recv_count': self._recv_count,
            'average_latency_ms': self.average_latency_ms,
            'latency_samples': len(self._latency_history),
        }

    def get_latency_history(self) -> list[float]:
        """Get full latency history for analysis."""
        return list(self._latency_history)
