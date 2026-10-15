#!/usr/bin/env python3
"""
VR Monitor - runs the HTTPS web UI server and the VR WebSocket server, and keeps
the latest goal per device (left/right controller, headset) for the teleop node.
"""

import asyncio
import http.server
import mimetypes
import os
import posixpath
import socket
import ssl
import threading
import time
from typing import Optional
from urllib.parse import unquote, urlsplit

from alfie_vr.alfievr.config import AlfieVRConfig
from alfie_vr.alfievr.inputs.vr_ws_server import VRWebSocketServer

DEVICES = ('left', 'right', 'headset')

CONTENT_TYPES = {
    '.html': 'text/html',
    '.js': 'application/javascript',
    '.css': 'text/css',
    '.json': 'application/json',
    '.jpg': 'image/jpeg',
    '.jpeg': 'image/jpeg',
    '.png': 'image/png',
    '.gif': 'image/gif',
    '.ico': 'image/x-icon',
    '.obj': 'model/obj',
    '.mtl': 'text/plain',
}


def _meshes_dir() -> Optional[str]:
    """URDF meshes from the alfie_urdf share directory (None if not installed)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('alfie_urdf'), 'meshes')
    except Exception:
        return None


def get_local_ip():
    """Get the local IP address of this machine."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "localhost"


def _safe_join(root: str, rel: str) -> Optional[str]:
    """Join rel under root, refusing anything that escapes root.

    Lexical check (abspath, not realpath) so a --symlink-install tree, whose
    files link back into src/, still serves.
    """
    root_abs = os.path.abspath(root)
    path = os.path.abspath(os.path.join(root_abs, rel))
    if os.path.commonpath([root_abs, path]) != root_abs:
        return None
    return path


class SimpleAPIHandler(http.server.BaseHTTPRequestHandler):
    """Static file handler for the web UI (web-ui/) and URDF meshes (/meshes/)."""

    def end_headers(self):
        """Add CORS headers to all responses."""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        try:
            super().end_headers()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, ssl.SSLError):
            pass

    def do_OPTIONS(self):
        """Handle preflight CORS requests."""
        self.send_response(200)
        self.end_headers()

    def log_message(self, format, *args):
        """Disable default per-request logging."""
        pass

    def do_GET(self):
        """Serve files; query strings are ignored, paths may not leave their root."""
        path = posixpath.normpath(unquote(urlsplit(self.path).path))
        if path in ('/', '/index.html', '.'):
            path = '/index.html'

        if path.startswith('/meshes/'):
            root = self.server.meshes_path
            rel = path[len('/meshes/'):]
            allowed = ('.obj', '.mtl')
        else:
            root = self.server.web_root_path
            rel = path.lstrip('/')
            allowed = tuple(CONTENT_TYPES)

        ext = os.path.splitext(rel)[1].lower()
        file_path = _safe_join(root, rel) if root and ext in allowed else None
        if file_path is None or not os.path.isfile(file_path):
            self.send_error(404, "Not found")
            return
        self.serve_file(file_path, CONTENT_TYPES.get(ext) or mimetypes.guess_type(file_path)[0])

    def serve_file(self, file_path, content_type):
        """Serve a file with the given content type."""
        try:
            with open(file_path, 'rb') as f:
                content = f.read()
            self.send_response(200)
            self.send_header('Content-Type', content_type or 'application/octet-stream')
            self.send_header('Content-Length', str(len(content)))
            # Disable caching for development - always fetch fresh files
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.end_headers()
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError, ssl.SSLError):
            pass
        except Exception as e:
            print(f"Error serving file {file_path}: {e}")
            self.send_error(500, "Internal server error")


class SimpleHTTPSServer:
    """Threaded HTTPS server for the web interface."""

    def __init__(self, config: AlfieVRConfig):
        self.config = config
        self.httpd = None
        self.server_thread = None

    async def start(self):
        """Start the HTTPS server."""
        self.httpd = http.server.ThreadingHTTPServer(
            (self.config.host_ip, self.config.https_port), SimpleAPIHandler)
        self.httpd.daemon_threads = True
        self.httpd.web_root_path = self.config.web_root
        self.httpd.meshes_path = _meshes_dir()

        # Setup SSL with mobile browser compatibility
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.maximum_version = ssl.TLSVersion.TLSv1_3
        # Broad cipher support for Android Chrome and Meta Quest browser
        context.set_ciphers(
            "ECDHE+AESGCM:ECDHE+CHACHA20:DHE+AESGCM:DHE+CHACHA20:"
            "ECDH+AESGCM:DH+AESGCM:ECDH+AES:DH+AES:RSA+AESGCM:RSA+AES:!aNULL:!eNULL:!MD5"
        )
        context.load_cert_chain(self.config.certfile, self.config.keyfile)
        self.httpd.socket = context.wrap_socket(self.httpd.socket, server_side=True)

        self.server_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.server_thread.start()
        print(f"🌐 HTTPS server started on {self.config.host_ip}:{self.config.https_port}")

    async def stop(self):
        """Stop the HTTPS server."""
        if self.httpd:
            self.httpd.shutdown()
            if self.server_thread:
                self.server_thread.join(timeout=5)
            print("🌐 HTTPS server stopped")


class VRMonitor:
    """Runs the VR servers and holds the latest goal per device."""

    def __init__(self, debug_logs: bool = False):
        self.debug_logs = debug_logs
        self.config = AlfieVRConfig()
        self.vr_server = None
        self.https_server = None
        self.command_queue = None
        self.is_running = False
        self._goals = {d: None for d in DEVICES}
        self._release_count = 0  # bumped on every client release request
        self._release_reason = None
        self._goal_lock = threading.Lock()

    async def start_monitoring(self):
        """Start the servers and consume goals until stopped (run in its own loop)."""
        self.command_queue = asyncio.Queue()
        self.vr_server = VRWebSocketServer(
            command_queue=self.command_queue,
            config=self.config,
            debug_logs=self.debug_logs,
        )
        self.https_server = SimpleHTTPSServer(self.config)

        try:
            await self.https_server.start()
            await self.vr_server.start()
            self.is_running = True

            host_display = get_local_ip() if self.config.host_ip == "0.0.0.0" else self.config.host_ip
            print("📱 Open your VR headset browser and navigate to:")
            print(f"   https://{host_display}:{self.config.https_port}")

            await self.monitor_commands()
        finally:
            await self.stop_monitoring()

    async def monitor_commands(self):
        """Store the latest goal per device; apply release requests."""
        while self.is_running:
            try:
                goal = await asyncio.wait_for(self.command_queue.get(), timeout=1.0)
            except asyncio.TimeoutError:
                continue

            with self._goal_lock:
                if goal.arm == 'release':
                    for hand in goal.metadata.get('hands', ()):
                        if hand in self._goals:
                            self._goals[hand] = None
                    self._release_count += 1
                    self._release_reason = goal.metadata.get('reason')
                elif goal.arm in self._goals:
                    self._goals[goal.arm] = goal

    def get_latest_goals(self, max_age: Optional[float] = None) -> dict:
        """Latest goal per device ('left', 'right', 'headset').

        A device whose latest goal is older than max_age seconds is returned as
        None, so a dropped link or lost controller never replays stale input.
        """
        now = time.monotonic()
        with self._goal_lock:
            out = {}
            for device, goal in self._goals.items():
                if goal is not None and max_age is not None and now - goal.stamp > max_age:
                    goal = None
                out[device] = goal
            return out

    def get_release_state(self):
        """(count, reason) of client release requests; count increases on each."""
        with self._goal_lock:
            return self._release_count, self._release_reason

    def set_passthrough_mode(self, active: bool):
        """Tell connected VR clients to enter/leave passthrough mode."""
        if self.vr_server:
            self.vr_server.broadcast_message_sync({
                'command': 'passthrough_mode',
                'active': active
            })

    async def stop_monitoring(self):
        """Stop the servers."""
        self.is_running = False
        if self.vr_server:
            await self.vr_server.stop()
        if self.https_server:
            await self.https_server.stop()
        print("✅ VR Monitor stopped")
