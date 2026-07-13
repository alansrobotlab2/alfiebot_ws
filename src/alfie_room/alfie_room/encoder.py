"""
encoder — turn a camera frame into an L2-normalized image embedding.

Wraps a DINOv2 ViT-S/14 image encoder exported to ONNX (see
``scripts/export_dinov2_onnx.py``) and runs it through onnxruntime. DINOv2's
self-supervised features are strong for pure image-to-image place recognition
(no text head needed), and ViT-S is small (~21M params) so a single on-demand
inference is cheap — tens of ms on the Orin GPU, ~100-300 ms on CPU, which is
fine because we only run it when the user asks.

Preprocessing matches DINOv2's expected input: RGB, resized to 224x224 (a
multiple of the patch size 14), scaled to [0,1], then ImageNet mean/std
normalized, laid out as NCHW float32.

The session prefers CUDA (CUDAExecutionProvider) and silently falls back to CPU
if the GPU provider isn't available in this onnxruntime build — so the node runs
either way. The model is loaded lazily by the caller (the node builds the encoder
only on the first classify/teach request) to avoid holding VRAM while idle.
"""
import cv2
import numpy as np

# ImageNet normalization constants DINOv2 was trained with.
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(3, 1, 1)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(3, 1, 1)

_PREFERRED_PROVIDERS = ["CUDAExecutionProvider", "CPUExecutionProvider"]


class Encoder:
    """A lazily-usable ONNX image encoder producing unit-length embeddings."""

    def __init__(self, model_path, input_size=224, providers=None):
        import onnxruntime as ort  # imported here so the module loads without ORT

        self.input_size = int(input_size)
        available = set(ort.get_available_providers())
        wanted = providers or _PREFERRED_PROVIDERS
        use = [p for p in wanted if p in available] or ["CPUExecutionProvider"]

        so = ort.SessionOptions()
        so.log_severity_level = 3  # warnings+; keep startup quiet
        self.session = ort.InferenceSession(model_path, sess_options=so,
                                            providers=use)
        self.providers = self.session.get_providers()
        self._input_name = self.session.get_inputs()[0].name

    # --- preprocessing --------------------------------------------------------

    def preprocess_bgr(self, bgr):
        """BGR HxWx3 uint8 (OpenCV order) -> NCHW float32 batch of 1, normalized."""
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.input_size, self.input_size),
                         interpolation=cv2.INTER_AREA)
        chw = rgb.astype(np.float32).transpose(2, 0, 1) / 255.0
        chw = (chw - _MEAN) / _STD
        return chw[np.newaxis, ...]  # (1, 3, H, W)

    def decode_jpeg(self, jpeg_bytes):
        """Decode JPEG bytes to a BGR uint8 image, or None if undecodable."""
        arr = np.frombuffer(jpeg_bytes, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    # --- inference ------------------------------------------------------------

    def _pool(self, out):
        """Reduce a raw model output to a single (dim,) embedding vector.

        Handles the common DINOv2 export shapes: a pooled (1, dim) output is used
        as-is; a token sequence (1, tokens, dim) is reduced to its CLS token
        (index 0), which is the standard global descriptor.
        """
        arr = np.asarray(out, dtype=np.float32)
        if arr.ndim == 3:      # (batch, tokens, dim) -> CLS token
            return arr[0, 0, :]
        if arr.ndim == 2:      # (batch, dim)
            return arr[0]
        return arr.reshape(-1)

    def embed_bgr(self, bgr):
        """Return a unit-length embedding for a BGR image."""
        x = self.preprocess_bgr(bgr)
        outputs = self.session.run(None, {self._input_name: x})
        vec = self._pool(outputs[0])
        norm = float(np.linalg.norm(vec))
        return vec / norm if norm > 1e-12 else vec

    def embed_jpeg(self, jpeg_bytes):
        """Return a unit-length embedding for a JPEG frame, or None if undecodable."""
        bgr = self.decode_jpeg(jpeg_bytes)
        if bgr is None:
            return None
        return self.embed_bgr(bgr)

    def embed_jpegs_mean(self, jpeg_list):
        """Fuse several JPEG frames into one averaged unit embedding.

        Used to combine the left+right eyes into a single view descriptor.
        Frames that fail to decode are skipped; returns None if none decode.
        """
        vecs = [v for v in (self.embed_jpeg(j) for j in jpeg_list) if v is not None]
        if not vecs:
            return None
        mean = np.mean(np.vstack(vecs), axis=0)
        norm = float(np.linalg.norm(mean))
        return mean / norm if norm > 1e-12 else mean
