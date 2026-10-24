"""Stdlib HTTP server for the episode manager: static React app, JSON API, MP4s.

Same shape as alfie_tools servotool: no web framework, because the Jetson has
none installed and `ros2 run` should work on an image with only ROS on it.

Videos are served with byte-range support. Browsers will play an MP4 without
it, but cannot seek one, and scrubbing four synced cameras is the whole point.

There is no authentication and the default bind is 0.0.0.0, so anyone who can
reach the port can relabel or delete episodes. Keep it on a trusted network.
"""

import json
import mimetypes
import os
import posixpath
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, Optional
from urllib.parse import parse_qs, urlparse

WEB_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')

CHUNK = 1 << 16
RANGE_RE = re.compile(r'bytes=(\d*)-(\d*)$')


class _Handler(BaseHTTPRequestHandler):

    protocol_version = 'HTTP/1.1'
    server_version = 'episode_manager'

    # -- plumbing ----------------------------------------------------------

    @property
    def manager(self):
        return self.server.manager

    def log_message(self, fmt: str, *args) -> None:
        """Route access logging to ROS debug instead of stderr."""
        if self.server.logger is not None:
            self.server.logger.debug('http %s' % (fmt % args))

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
        url = urlparse(self.path)
        try:
            if url.path == '/api/dataset':
                self._json(self.manager.describe())
            elif url.path == '/api/status':
                self._json(self.manager.status())
            elif url.path == '/api/job':
                self._json(self.manager.job_status())
            elif url.path == '/api/video':
                self._video(parse_qs(url.query))
            else:
                self._static(url.path)
        except (BrokenPipeError, ConnectionResetError):
            pass                        # browser navigated away mid-response

    def do_HEAD(self) -> None:         # noqa: N802 - http.server API
        self.do_GET()

    def do_POST(self) -> None:         # noqa: N802 - http.server API
        path = urlparse(self.path).path
        handler: Optional[Callable[[Dict[str, Any]], Any]] = {
            '/api/select_dataset': lambda p: self.manager.select_dataset(str(p.get('name'))),
            '/api/set_task': lambda p: self.manager.set_task(int(p['episode']), int(p['task'])),
            '/api/delete': lambda p: self.manager.delete(int(p['episode'])),
            '/api/convert': lambda p: self.manager.convert(
                int(p.get('task', 0)), bool(p.get('force')), str(p.get('task_name') or '')),
            '/api/cancel': lambda p: self.manager.cancel(),
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
        except (KeyError, TypeError, ValueError) as exc:
            self._error(400, f'bad request: {exc!r}')
        except Exception as exc:        # a tool bug must not kill the server
            if self.server.logger is not None:
                self.server.logger.error(f'{path} failed: {exc!r}')
            self._error(500, f'{type(exc).__name__}: {exc}')

    # -- video -------------------------------------------------------------

    def _video(self, query: Dict[str, list]) -> None:
        try:
            index = int(query['episode'][0])
            camera = query['camera'][0]
        except (KeyError, IndexError, ValueError):
            self._error(400, 'need ?episode=<int>&camera=<name>')
            return
        path = self.manager.dataset.video_path(index, camera)
        if path is None:
            self._error(404, f'no {camera} video for episode {index}')
            return

        size = path.stat().st_size
        start, end = 0, size - 1
        code = 200
        match = RANGE_RE.match(self.headers.get('Range', '').strip())
        if match and size:
            first, last = match.groups()
            if first:
                start = int(first)
                end = min(int(last), size - 1) if last else size - 1
            elif last:                  # suffix range: the final N bytes
                start = max(0, size - int(last))
            if start > end or start >= size:
                self.send_response(416)
                self.send_header('Content-Range', f'bytes */{size}')
                self.send_header('Content-Length', '0')
                self.end_headers()
                return
            code = 206

        self.send_response(code)
        self.send_header('Content-Type', 'video/mp4')
        self.send_header('Accept-Ranges', 'bytes')
        self.send_header('Content-Length', str(end - start + 1))
        self.send_header('Cache-Control', 'no-cache')
        if code == 206:
            self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
        self.end_headers()
        if self.command == 'HEAD':
            return
        with open(path, 'rb') as handle:
            handle.seek(start)
            remaining = end - start + 1
            while remaining > 0:
                data = handle.read(min(CHUNK, remaining))
                if not data:
                    break
                self.wfile.write(data)
                remaining -= len(data)

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


class EpisodeManagerServer(ThreadingHTTPServer):
    """Threaded HTTP server holding a reference to the EpisodeManager."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, host: str, port: int, manager, logger=None) -> None:
        super().__init__((host, port), _Handler)
        self.manager = manager
        self.logger = logger
