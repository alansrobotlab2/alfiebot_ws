"""
room_node — on-demand visual room recognition service.

Alfie has no map or metric localization, so "which room am I in?" is answered by
*appearance*: does the current camera view look like a room he's been taught? This
node owns that capability and exposes it to the conversation agent over a tiny
local HTTP endpoint (mirroring how the agent already reaches the QMD search
daemon), keeping the vision model out of the LLM process.

What it does:
  * Subscribes to the left wide eye
    (``stereo_camera/left_wide/image_raw/compressed``) and keeps only the
    latest JPEG (BEST_EFFORT depth-1, matching the camera publisher).
  * Lazily loads a DINOv2 image encoder (``encoder.Encoder``) on the first
    request — no VRAM is held while idle.
  * Embeds the current view and matches it against a persistent
    ``RoomStore`` of taught rooms.
  * Serves three routes on ``127.0.0.1:<http_port>``:
      - ``POST /classify``           -> best-matching room + confidence
      - ``POST /teach {"name": ...}`` -> remember the current view as that room
      - ``GET  /rooms``              -> the known-room list
  * Publishes a latched ``room`` String after each classify (telemetry / other
    consumers), and best-effort mirrors the known-room names into the Obsidian
    vault (``<vault_root>/alfie/rooms.md``) so the user can see what Alfie knows.

Everything vision runs on the HTTP worker thread; ROS callbacks only stash the
latest frame under a lock, so the node stays responsive and inference never
blocks the ROS executor.
"""
import json
import os
import threading
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from alfie_room.room_store import RoomStore


class RoomNode(Node):
    def __init__(self):
        super().__init__('room_node')

        # --- parameters -------------------------------------------------------
        self.left_topic = self.declare_parameter(
            'left_topic', 'stereo_camera/left_wide/image_raw/compressed').value
        self.http_port = int(self.declare_parameter('http_port', 8182).value)
        self.model_path = os.path.expanduser(self.declare_parameter(
            'model_path', '~/alfiebot_ws/models/dinov2_vits14.onnx').value)
        self.input_size = int(self.declare_parameter('input_size', 224).value)
        store_path = self.declare_parameter(
            'store_path', '~/alfiebot_ws/data/rooms/rooms.json').value
        sim_threshold = float(self.declare_parameter('sim_threshold', 0.55).value)
        margin = float(self.declare_parameter('margin', 0.05).value)
        max_exemplars = int(self.declare_parameter('max_exemplars_per_room', 12).value)
        self.vault_root = self.declare_parameter('vault_root', '~/obsidian').value

        self.store = RoomStore(store_path, sim_threshold=sim_threshold,
                               margin=margin,
                               max_exemplars_per_room=max_exemplars)

        # --- frame cache (written by ROS callbacks, read by HTTP workers) -----
        self._frame_lock = threading.Lock()
        self._left_jpeg = None

        # --- encoder (lazy) ---------------------------------------------------
        self._encoder = None
        self._encoder_lock = threading.Lock()
        self._encoder_error = None

        # --- ROS I/O ----------------------------------------------------------
        # Match the camera publisher: BEST_EFFORT, keep-last depth 1 (latest frame).
        cam_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, self.left_topic,
                                 self._on_left, cam_qos)

        # Latched current-room telemetry so late subscribers still get the value.
        room_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.room_pub = self.create_publisher(String, 'room', room_qos)

        # --- HTTP server thread ----------------------------------------------
        self._httpd = self._make_server()
        self._http_thread = threading.Thread(
            target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()

        self.get_logger().info(
            f'room_node up: HTTP on 127.0.0.1:{self.http_port}, '
            f'model={self.model_path}, {len(self.store.rooms())} room(s) known.')
        self._write_rooms_md()  # reflect current known rooms at startup

    # --- ROS callbacks (keep tiny) -------------------------------------------

    def _on_left(self, msg):
        with self._frame_lock:
            self._left_jpeg = bytes(msg.data)

    def _snapshot_frame(self):
        """Return the latest left-eye JPEG bytes under the lock.

        May be None if the eye hasn't published a frame yet.
        """
        with self._frame_lock:
            return self._left_jpeg

    # --- encoder (lazy load) --------------------------------------------------

    def _get_encoder(self):
        """Build the encoder on first use; returns (encoder, error_str)."""
        if self._encoder is not None:
            return self._encoder, None
        with self._encoder_lock:
            if self._encoder is not None:
                return self._encoder, None
            if self._encoder_error is not None:
                return None, self._encoder_error
            if not os.path.exists(self.model_path):
                self._encoder_error = (
                    f'model not found at {self.model_path} — run '
                    'scripts/export_dinov2_onnx.py')
                self.get_logger().error(self._encoder_error)
                return None, self._encoder_error
            try:
                from alfie_room.encoder import Encoder
                self.get_logger().info(f'Loading encoder {self.model_path} ...')
                self._encoder = Encoder(self.model_path, input_size=self.input_size)
                self.get_logger().info(
                    f'Encoder ready (providers={self._encoder.providers}).')
            except Exception as e:
                self._encoder_error = f'failed to load encoder: {e}'
                self.get_logger().error(self._encoder_error)
                return None, self._encoder_error
        return self._encoder, None

    def _current_embedding(self):
        """Embed the current left-eye view; returns (vec, error_str)."""
        enc, err = self._get_encoder()
        if enc is None:
            return None, err
        jpeg = self._snapshot_frame()
        if jpeg is None:
            return None, 'no camera frame received yet'
        try:
            vec = enc.embed_jpeg(jpeg)
        except Exception as e:
            return None, f'embedding failed: {e}'
        if vec is None:
            return None, 'could not decode camera frame'
        return vec, None

    # --- request handlers (called from HTTP worker threads) -------------------

    def handle_classify(self):
        vec, err = self._current_embedding()
        if err:
            return {"error": err}
        result = self.store.classify(vec)
        # Publish latched telemetry: the room, or "unknown".
        msg = String()
        msg.data = result.get("room") or "unknown"
        self.room_pub.publish(msg)
        return result

    def handle_teach(self, name):
        name = (name or "").strip()
        if not name:
            return {"error": "a room name is required"}
        vec, err = self._current_embedding()
        if err:
            return {"error": err}
        count = self.store.teach(name, vec)
        self._write_rooms_md()
        self.get_logger().info(f"Taught room '{name}' (now {count} exemplar(s)).")
        return {"taught": name, "exemplars": count,
                "known_rooms": sorted(self.store.rooms().keys())}

    def handle_rooms(self):
        return {"rooms": self.store.rooms(),
                "known_rooms": sorted(self.store.rooms().keys())}

    # --- vault mirror (best-effort) ------------------------------------------

    def _write_rooms_md(self):
        """Mirror the known-room names into the Obsidian vault.

        Lets the user (and the agent via vault_search) see what Alfie knows;
        a no-op if the vault directory is absent.
        """
        root = os.path.expanduser(self.vault_root or "")
        if not root or not os.path.isdir(root):
            return
        try:
            rooms = self.store.rooms()
            lines = ["---", "kind: rooms", "---", "# Rooms Alfie knows", ""]
            if rooms:
                for name in sorted(rooms):
                    lines.append(f"- {name} ({rooms[name]} view(s) learned)")
            else:
                lines.append("_(none taught yet)_")
            lines.append("")
            lines.append(f"_updated {datetime.now(timezone.utc).isoformat(timespec='seconds')}_")
            out_dir = os.path.join(root, "alfie")
            os.makedirs(out_dir, exist_ok=True)
            with open(os.path.join(out_dir, "rooms.md"), "w") as f:
                f.write("\n".join(lines) + "\n")
        except OSError as e:
            self.get_logger().warn(f'could not write rooms.md: {e}')

    # --- HTTP plumbing --------------------------------------------------------

    def _make_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):  # silence default stderr logging
                pass

            def _send(self, code, obj):
                body = json.dumps(obj).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self):
                length = int(self.headers.get("Content-Length") or 0)
                if not length:
                    return {}
                try:
                    return json.loads(self.rfile.read(length) or b"{}")
                except (ValueError, TypeError):
                    return {}

            def do_GET(self):
                if self.path.rstrip("/") == "/rooms":
                    self._send(200, node.handle_rooms())
                else:
                    self._send(404, {"error": "not found"})

            def do_POST(self):
                path = self.path.rstrip("/")
                if path == "/classify":
                    self._send(200, node.handle_classify())
                elif path == "/teach":
                    body = self._read_json()
                    self._send(200, node.handle_teach(body.get("name")))
                else:
                    self._send(404, {"error": "not found"})

        return ThreadingHTTPServer(("127.0.0.1", self.http_port), Handler)

    def destroy_node(self):
        try:
            self._httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
