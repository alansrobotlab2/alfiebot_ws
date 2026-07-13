#!/usr/bin/env python3
"""
nanoowl_node — open-vocabulary object detection with NanoOWL (OWL-ViT).

Subscribes to a compressed camera stream, runs NanoOWL on the latest frame at a
fixed rate, and publishes the detections as an ``alfie_msgs/Detections`` list on
``nanoowl/detections``. What to look for is set by a free-text *prompt* (a
comma-separated list of queries, e.g. ``"a person, a face, a cup"``) that can be
changed live by publishing a ``std_msgs/String`` on ``nanoowl/prompt`` — so the
agent can point the detector at whatever it currently cares about.

Inference is throttled by a timer rather than run per-frame: the subscription
keeps only the newest frame (depth-1) and the timer processes it at
``rate_hz``, so a slow model never backs up the camera pipeline.

Speed: with ``image_encoder_engine`` pointing at a pre-built TensorRT engine
the OWL-ViT image encoder runs on TensorRT (big speedup on Jetson). With it
unset the node falls back to a pure-PyTorch encoder — slower, but it runs with
no engine build. See the package README for how to build the engine.
"""
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from alfie_msgs.msg import Detection, Detections

from cv_bridge import CvBridge
import cv2

# NanoOWL is a heavy optional dependency (torch + transformers + optional
# TensorRT). Import lazily so the module can at least be imported for linting;
# the node constructor turns a missing dep into a clear, actionable error.
try:
    from nanoowl.owl_predictor import OwlPredictor
    _IMPORT_ERR = None
except Exception as e:  # pragma: no cover - depends on host install
    OwlPredictor = None
    _IMPORT_ERR = e


def _parse_prompt(text):
    """'[a person, a face]' or 'a person, a face' -> ['a person', 'a face']."""
    text = text.strip()
    if text.startswith('[') and text.endswith(']'):
        text = text[1:-1]
    return [q.strip() for q in text.split(',') if q.strip()]


class NanoOwlNode(Node):
    def __init__(self):
        super().__init__('nanoowl_node')

        # --- Parameters ---------------------------------------------------
        self.image_topic = self.declare_parameter(
            'image_topic',
            'stereo_camera/left_center/image_raw/compressed').value
        self.detections_topic = self.declare_parameter(
            'detections_topic', 'nanoowl/detections').value
        self.prompt_topic = self.declare_parameter(
            'prompt_topic', 'nanoowl/prompt').value
        self.annotated_topic = self.declare_parameter(
            'annotated_topic', 'nanoowl/annotated/compressed').value

        self.model_name = self.declare_parameter(
            'model_name', 'google/owlvit-base-patch32').value
        # Path to a pre-built TensorRT image-encoder engine. Empty = pure PyTorch.
        self.image_encoder_engine = self.declare_parameter(
            'image_encoder_engine', '').value
        self.prompt = self.declare_parameter(
            'prompt', 'a person, a face, a hand').value
        self.threshold = float(self.declare_parameter('threshold', 0.1).value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 5.0).value)
        self.publish_annotated = bool(
            self.declare_parameter('publish_annotated', False).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 70).value)

        if OwlPredictor is None:
            self.get_logger().error(
                'nanoowl is not installed: %r\n'
                'Install it with:\n'
                '  pip install transformers Pillow\n'
                '  pip install git+https://github.com/NVIDIA-AI-IOT/nanoowl\n'
                'See the alfie_nanoowl README for TensorRT engine build steps.'
                % _IMPORT_ERR)
            raise SystemExit(1)

        # --- Model --------------------------------------------------------
        engine = self.image_encoder_engine or None
        self.get_logger().info(
            f'Loading NanoOWL model="{self.model_name}" '
            f'engine={engine or "<pytorch>"} ...')
        t0 = time.time()
        self.predictor = OwlPredictor(
            self.model_name,
            image_encoder_engine=engine,
        )
        self.get_logger().info(f'Model ready in {time.time() - t0:.1f}s')

        self.bridge = CvBridge()
        self._text = []
        self._text_encodings = None
        self._set_prompt(self.prompt)

        # --- Latest-frame slot (depth-1, overwrite not queue) -------------
        self._latest = None  # (np.ndarray BGR, header)

        # --- I/O ----------------------------------------------------------
        sensor_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.det_pub = self.create_publisher(
            Detections, self.detections_topic, 10)
        if self.publish_annotated:
            self.annot_pub = self.create_publisher(
                CompressedImage, self.annotated_topic, sensor_qos)
        self.create_subscription(
            CompressedImage, self.image_topic, self._on_image, sensor_qos)
        self.create_subscription(
            String, self.prompt_topic, self._on_prompt, 10)

        self.create_timer(1.0 / max(self.rate_hz, 0.1), self._on_timer)

        self.get_logger().info(
            f'nanoowl_node up: "{self.image_topic}" -> "{self.detections_topic}" '
            f'@ {self.rate_hz:.1f} Hz, prompt={self._text}')

    # ---- prompt handling -------------------------------------------------
    def _set_prompt(self, text):
        """(Re)encode the text queries; cached so predict() skips text encoding."""
        queries = _parse_prompt(text)
        if not queries:
            self.get_logger().warning(f'Empty prompt ignored: {text!r}')
            return
        self._text = queries
        self._text_encodings = self.predictor.encode_text(queries)
        self.get_logger().info(f'Prompt set: {queries}')

    def _on_prompt(self, msg):
        self._set_prompt(msg.data)

    # ---- image handling --------------------------------------------------
    def _on_image(self, msg):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:  # pragma: no cover
            self.get_logger().warning(f'decode failed: {e}',
                                      throttle_duration_sec=2.0)
            return
        self._latest = (img, msg.header)

    def _on_timer(self):
        if self._latest is None or self._text_encodings is None:
            return
        img, header = self._latest
        self._latest = None  # consume; wait for a fresh frame next tick

        h, w = img.shape[:2]
        # NanoOWL wants a PIL RGB image.
        from PIL import Image
        pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

        t0 = time.time()
        try:
            output = self.predictor.predict(
                image=pil,
                text=self._text,
                text_encodings=self._text_encodings,
                threshold=self.threshold,
                pad_square=False,
            )
        except Exception as e:  # pragma: no cover
            self.get_logger().error(f'inference failed: {e}',
                                    throttle_duration_sec=2.0)
            return
        dt = time.time() - t0

        det_msg = self._build_msg(output, header, w, h)
        self.det_pub.publish(det_msg)
        self.get_logger().info(
            f'{len(det_msg.detections)} det in {dt * 1e3:.0f}ms',
            throttle_duration_sec=1.0)

        if self.publish_annotated:
            self._publish_annotated(img, det_msg, header)

    # ---- message building ------------------------------------------------
    def _build_msg(self, output, header, width, height):
        msg = Detections()
        msg.header = header
        msg.source_topic = self.image_topic
        msg.prompt = ', '.join(self._text)
        msg.image_width = int(width)
        msg.image_height = int(height)

        boxes = _to_numpy(output.boxes)      # [N, 4] pixel corners x0,y0,x1,y1
        scores = _to_numpy(output.scores)    # [N]
        labels = _to_numpy(output.labels)    # [N] index into self._text
        for i in range(len(scores)):
            d = Detection()
            idx = int(labels[i])
            d.class_id = idx
            d.label = self._text[idx] if 0 <= idx < len(self._text) else str(idx)
            d.score = float(scores[i])
            x0, y0, x1, y1 = (float(v) for v in boxes[i])
            # OWL-ViT returns pixel-space corners; normalize to 0..1 and clamp
            # (boxes can extend a hair past the image edge).
            d.x0 = min(max(x0 / width, 0.0), 1.0)
            d.y0 = min(max(y0 / height, 0.0), 1.0)
            d.x1 = min(max(x1 / width, 0.0), 1.0)
            d.y1 = min(max(y1 / height, 0.0), 1.0)
            msg.detections.append(d)
        return msg

    def _publish_annotated(self, img, det_msg, header):
        vis = img.copy()
        h, w = vis.shape[:2]
        for d in det_msg.detections:
            p0 = (int(d.x0 * w), int(d.y0 * h))
            p1 = (int(d.x1 * w), int(d.y1 * h))
            cv2.rectangle(vis, p0, p1, (0, 255, 0), 2)
            cv2.putText(vis, f'{d.label} {d.score:.2f}',
                        (p0[0], max(p0[1] - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                        cv2.LINE_AA)
        ok, buf = cv2.imencode(
            '.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            return
        out = CompressedImage()
        out.header = header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self.annot_pub.publish(out)


def _to_numpy(t):
    """Torch tensor (any device) or array-like -> 1-2D numpy array."""
    if hasattr(t, 'detach'):
        t = t.detach().cpu().numpy()
    return np.asarray(t)


def main(args=None):
    rclpy.init(args=args)
    node = NanoOwlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
