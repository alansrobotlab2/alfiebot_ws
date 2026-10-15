"""Shared image sizing for the GR00T pipeline.

Every image that reaches the model (live observations, training MP4s,
diagnostic replays) must go through the same geometry:
center-crop to the target aspect ratio, then resize to the target size.
Plain resizing between aspect ratios (e.g. 4:3 -> 352x256) would distort
the image, and the distortion would differ between old and new data.

rosbag_to_groot.py keeps a standalone copy of fit_image() because it runs
outside the installed package -- keep the two in sync.
"""

import cv2
import numpy as np

# Model input size (must match training data MP4s and the camera node output)
IMAGE_WIDTH = 352
IMAGE_HEIGHT = 256


def center_crop_to_aspect(img: np.ndarray, width: int, height: int) -> np.ndarray:
    """Center-crop img to the aspect ratio width:height (no resize)."""
    h, w = img.shape[:2]
    if w * height > h * width:
        # Too wide: trim left/right
        new_w = round(h * width / height)
        x0 = (w - new_w) // 2
        return img[:, x0:x0 + new_w]
    if w * height < h * width:
        # Too tall: trim top/bottom
        new_h = round(w * height / width)
        y0 = (h - new_h) // 2
        return img[y0:y0 + new_h, :]
    return img


def fit_image(
    img: np.ndarray,
    width: int = IMAGE_WIDTH,
    height: int = IMAGE_HEIGHT,
    interpolation: int = cv2.INTER_LINEAR,
) -> np.ndarray:
    """Center-crop to the target aspect ratio, then resize to width x height.

    No-op for images already at the target size.
    """
    if img.shape[1] == width and img.shape[0] == height:
        return img
    img = center_crop_to_aspect(img, width, height)
    return cv2.resize(img, (width, height), interpolation=interpolation)
