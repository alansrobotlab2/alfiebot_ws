"""Stdlib HTTP server for servotool3: static React app + JSON API + SSE stream.

Deliberately dependency-free. The robot's Jetson has no node/npm and no web
framework installed, so the frontend is plain (build-free) React served from
disk and the backend is http.server. That keeps `ros2 run alfie_tools servotool`
working on a fresh image with nothing but a ROS install.

Live state goes out over Server-Sent Events rather than websockets: one-way,
trivially proxied, and reconnects on its own in every browser.

There is no authentication. Bound to 0.0.0.0 by default (that is the point - it
is used from a laptop next to the robot), so anyone who can reach the port can
move the arms. Keep it on a trusted network.
"""

import json
import mimetypes
import os
import posixpath
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, Optional
from urllib.parse import urlparse

WEB_ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')

# Cap concurrent event streams so a pile of stale browser tabs cannot pin the
# ROS callback thread with snapshot serialization.
MAX_STREAMS = 8


class _Handler(BaseHTTPRequestHandler):
    """Request handler. One instance per request, many threads at once."""

    protocol_version = 'HTTP/1.1'
    server_version = 'servotool3'

    # -- plumbing ----------------------------------------------------------

    @property
    def bridge(self):
        return self.server.bridge

    @property
    def logger(self):
        return self.server.logger

    def log_message(self, fmt: str, *args) -> None:
        """Route access logging to ROS debug instead of stderr."""
        if self.logger is not None:
            self.logger.debug('http %s' % (fmt % args))

    def _send(self, code: int, body: bytes, content_type: str,
              extra: Optional[Dict[str, str]] = None) -> None:
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(body)

    def _json(self, payload: Any, code: int = 200) -> None:
        self._send(code, json.dumps(payload).encode('utf-8'), 'application/json')

    def _error(self, code: int, message: str) -> None:
        self._json({'ok': False, 'error': message}, code)

    def _read_json(self) -> Optional[Dict[str, Any]]:
        try:
            length = int(self.headers.get('Content-Length') or 0)
        except ValueError:
            return None
        if length <= 0 or length > 1 << 20:
            return {} if length == 0 else None
        try:
            return json.loads(self.rfile.read(length).decode('utf-8'))
        except (ValueError, UnicodeDecodeError):
            return None

    # -- routing -----------------------------------------------------------

    def do_GET(self) -> None:          # noqa: N802 - http.server API
        path = urlparse(self.path).path
        try:
            if path == '/api/config':
                self._json(self.bridge.describe())
            elif path == '/api/state':
                self._json(self.bridge.snapshot())
            elif path == '/api/stream':
                self._stream()
            else:
                self._static(path)
        except (BrokenPipeError, ConnectionResetError):
            pass                        # browser navigated away mid-response

    def do_HEAD(self) -> None:         # noqa: N802 - http.server API
        self.do_GET()

    def do_POST(self) -> None:         # noqa: N802 - http.server API
        path = urlparse(self.path).path
        handler: Optional[Callable[[Dict[str, Any]], Any]] = {
            '/api/heartbeat': self._post_heartbeat,
            '/api/select': self._post_select,
            '/api/register': self._post_register,
            '/api/lock': self._post_lock,
            '/api/control': self._post_control,
            '/api/command': self._post_command,
            '/api/torque_off': self._post_torque_off,
            '/api/estop': self._post_estop,
            '/api/calibrate_back': self._post_calibrate,
        }.get(path)

        if handler is None:
            self._error(404, f'no such endpoint: {path}')
            return

        payload = self._read_json()
        if payload is None:
            self._error(400, 'body is not valid JSON')
            return

        try:
            self._json(handler(payload))
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:        # a tool bug must not kill the server
            if self.logger is not None:
                self.logger.error(f'{path} failed: {exc!r}')
            self._error(500, f'{type(exc).__name__}: {exc}')

    # -- endpoints ---------------------------------------------------------

    def _post_heartbeat(self, _payload: Dict[str, Any]) -> Dict[str, Any]:
        self.bridge.touch()
        return {'ok': True}

    def _post_select(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Point the register view at one physical bus servo."""
        try:
            bus_id = int(payload.get('bus_id'))
        except (TypeError, ValueError):
            return {'ok': False, 'error': 'bus_id must be an integer'}
        return self.bridge.select(payload.get('subsystem'), bus_id)

    def _post_register(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Write one servo register."""
        try:
            bus_id = int(payload['bus_id'])
            address = int(payload['address'])
            value = int(payload['value'])
        except (KeyError, TypeError, ValueError):
            return {'ok': False,
                    'error': 'bus_id, address and value must all be integers'}
        return self.bridge.write_register(
            payload.get('subsystem'), bus_id, address, value)

    def _post_lock(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Set or clear the EEPROM write lock."""
        try:
            bus_id = int(payload['bus_id'])
        except (KeyError, TypeError, ValueError):
            return {'ok': False, 'error': 'bus_id must be an integer'}
        return self.bridge.set_lock(
            payload.get('subsystem'), bus_id, bool(payload.get('locked')))

    def _post_control(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        subsystem = payload.get('subsystem')
        action = payload.get('action')
        if action == 'take':
            return self.bridge.take(subsystem)
        if action == 'release':
            return self.bridge.release(subsystem)
        if action == 'release_all':
            self.bridge.release_all('released from the web UI')
            return {'ok': True}
        return {'ok': False, 'error': f'unknown control action "{action}"'}

    def _post_command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.bridge.apply_command(payload)

    def _post_torque_off(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.bridge.torque_off(payload.get('subsystem'))

    def _post_estop(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        action = payload.get('action')
        if action == 'engage':
            return self.bridge.engage_estop()
        if action == 'reset':
            return self.bridge.reset_estop()
        return {'ok': False, 'error': f'unknown e-stop action "{action}"'}

    def _post_calibrate(self, _payload: Dict[str, Any]) -> Dict[str, Any]:
        return self.bridge.calibrate_back()

    # -- SSE ---------------------------------------------------------------

    def _stream(self) -> None:
        """Push a state snapshot at the configured rate until the client goes."""
        if not self.server.claim_stream():
            self._error(503, 'too many open event streams')
            return

        self.close_connection = True    # no content length: stream until closed
        try:
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.send_header('X-Accel-Buffering', 'no')
            self.end_headers()

            period = 1.0 / self.server.stream_rate
            next_tick = time.monotonic()
            while not self.server.shutting_down:
                payload = json.dumps(self.bridge.snapshot(), separators=(',', ':'))
                self.wfile.write(f'data: {payload}\n\n'.encode('utf-8'))
                self.wfile.flush()
                next_tick += period
                time.sleep(max(0.0, next_tick - time.monotonic()))
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass                        # tab closed / network went away
        finally:
            self.server.release_stream()

    # -- static files ------------------------------------------------------

    def _static(self, path: str) -> None:
        rel = posixpath.normpath(path).lstrip('/')
        if rel in ('', '.'):
            rel = 'index.html'

        full = os.path.normpath(os.path.join(WEB_ROOT, rel))
        # normpath collapses "..", so a prefix check is enough to keep requests
        # inside the web root.
        if not full.startswith(WEB_ROOT + os.sep) or not os.path.isfile(full):
            self._error(404, f'not found: {path}')
            return

        content_type, _ = mimetypes.guess_type(full)
        with open(full, 'rb') as handle:
            body = handle.read()
        self._send(200, body, content_type or 'application/octet-stream')


class ServoToolServer(ThreadingHTTPServer):
    """Threaded HTTP server holding a reference to the ROS bridge."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, host: str, port: int, bridge, logger=None,
                 stream_rate: float = 10.0) -> None:
        super().__init__((host, port), _Handler)
        self.bridge = bridge
        self.logger = logger
        self.stream_rate = max(1.0, float(stream_rate))
        self.shutting_down = False
        self._streams = 0
        self._stream_lock = threading.Lock()

    def claim_stream(self) -> bool:
        with self._stream_lock:
            if self._streams >= MAX_STREAMS:
                return False
            self._streams += 1
            return True

    def release_stream(self) -> None:
        with self._stream_lock:
            self._streams = max(0, self._streams - 1)

    def stop(self) -> None:
        self.shutting_down = True
        self.shutdown()
        self.server_close()
