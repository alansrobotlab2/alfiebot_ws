#!/usr/bin/env python3
"""Compare raw rosbag image vs H.264-encoded training image.

Extracts the first left_wide image from:
  A) The rosbag (JPEG decompress -> resize -> RGB, NO H.264)
  B) The training MP4 video (first frame, already H.264 encoded)

Produces pixel-level statistics and visual comparison to characterize
what H.264 encoding (yuv420p, crf=23) does to the training images.

Output: /tmp/groot_image_comparison/

Usage:
    python3 compare_rosbag_vs_training.py
    python3 compare_rosbag_vs_training.py --rosbag /path/to/file.mcap --video /path/to/episode.mp4
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import av
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

try:
    import zstandard as zstd
    HAS_ZSTD = True
except ImportError:
    HAS_ZSTD = False

VIDEO_WIDTH = 320
VIDEO_HEIGHT = 240
LEFT_WIDE_TOPIC = '/alfie/stereo_camera/left_wide/image_raw/compressed'
OUTPUT_DIR = '/tmp/groot_image_comparison'


def extract_rosbag_image(mcap_path: str) -> np.ndarray:
    """Extract first left_wide image from rosbag, same processing as rosbag_to_groot.py.

    Pipeline: CompressedImage -> imdecode(BGR) -> resize(320,240) -> cvtColor(BGR2RGB)
    This is what the image looks like BEFORE H.264 encoding.

    Returns: numpy array shape (240, 320, 3) dtype uint8, RGB format
    """
    mcap_path = Path(mcap_path)

    if mcap_path.suffix == '.zstd':
        decompressed = mcap_path.with_suffix('')
        if not decompressed.exists():
            if not HAS_ZSTD:
                raise RuntimeError('zstandard package needed for .mcap.zstd files')
            print(f'Decompressing {mcap_path.name}...')
            dctx = zstd.ZstdDecompressor()
            with open(mcap_path, 'rb') as ifh:
                with open(decompressed, 'wb') as ofh:
                    dctx.copy_stream(ifh, ofh)
        mcap_path = decompressed

    storage_options = StorageOptions(uri=str(mcap_path), storage_id='mcap')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic == LEFT_WIDE_TOPIC:
            msg = deserialize_message(data, CompressedImage)
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is not None:
                img = cv2.resize(img, (VIDEO_WIDTH, VIDEO_HEIGHT))
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                print(f'Extracted rosbag image: {img.shape}, dtype={img.dtype}, ts={timestamp_ns}')
                return img

    raise RuntimeError(f'No left_wide image found in {mcap_path}')


def extract_training_frame(video_path: str) -> np.ndarray:
    """Extract first frame from training video.

    The video was encoded with: libx264, crf=23, yuv420p pixel format.
    PyAV decodes back to RGB24, which is what the GR00T data loader does.

    Returns: numpy array shape (240, 320, 3) dtype uint8, RGB format
    """
    container = av.open(str(video_path))
    stream = container.streams.video[0]

    for frame in container.decode(video=0):
        img = frame.to_ndarray(format='rgb24')
        print(f'Extracted training frame: {img.shape}, dtype={img.dtype}, '
              f'codec={stream.codec_context.name}, pix_fmt={stream.codec_context.pix_fmt}')
        container.close()
        return img

    container.close()
    raise RuntimeError(f'No frames found in {video_path}')


def compute_statistics(img_a: np.ndarray, img_b: np.ndarray) -> str:
    """Compute per-channel statistics and differences between two RGB images."""
    lines = []
    lines.append('=' * 70)
    lines.append('IMAGE COMPARISON: Raw JPEG pipeline vs H.264 training')
    lines.append('=' * 70)

    channel_names = ['Red', 'Green', 'Blue']

    for label, img in [('Source A (Rosbag raw)', img_a), ('Source B (Training H.264)', img_b)]:
        lines.append(f'\n{label}:')
        lines.append(f'  Shape: {img.shape}, dtype: {img.dtype}')
        for c, name in enumerate(channel_names):
            ch = img[:, :, c].astype(np.float64)
            lines.append(f'  {name:5s}: mean={ch.mean():.4f}, std={ch.std():.4f}, '
                         f'min={ch.min():.0f}, max={ch.max():.0f}')

    diff = np.abs(img_a.astype(np.float64) - img_b.astype(np.float64))

    lines.append(f'\nAbsolute Difference (|A - B|):')
    for c, name in enumerate(channel_names):
        d = diff[:, :, c]
        lines.append(f'  {name:5s}: mean={d.mean():.4f}, max={d.max():.0f}, '
                     f'p50={np.percentile(d, 50):.1f}, p90={np.percentile(d, 90):.1f}, '
                     f'p95={np.percentile(d, 95):.1f}, p99={np.percentile(d, 99):.1f}')

    overall_diff = diff.mean()
    lines.append(f'\n  Overall mean abs diff: {overall_diff:.4f}')
    lines.append(f'  Overall max abs diff: {diff.max():.0f}')

    mse = np.mean((img_a.astype(np.float64) - img_b.astype(np.float64)) ** 2)
    if mse == 0:
        psnr = float('inf')
    else:
        psnr = 10 * np.log10(255.0 ** 2 / mse)
    lines.append(f'\n  MSE: {mse:.4f}')
    lines.append(f'  PSNR: {psnr:.2f} dB')

    lines.append(f'\nDifference histogram (all channels combined):')
    flat_diff = diff.flatten()
    for threshold in [0, 1, 2, 3, 4, 5, 10, 15, 20, 30]:
        count = np.sum(flat_diff <= threshold)
        pct = 100.0 * count / flat_diff.size
        lines.append(f'  diff <= {threshold:2d}: {pct:6.2f}% ({count:,d} / {flat_diff.size:,d} pixels)')

    lines.append('=' * 70)
    return '\n'.join(lines)


def save_visualizations(img_a: np.ndarray, img_b: np.ndarray, output_dir: str):
    """Save individual images, side-by-side comparison, and difference heatmap."""
    os.makedirs(output_dir, exist_ok=True)

    cv2.imwrite(os.path.join(output_dir, 'source_a_rosbag_raw.png'),
                cv2.cvtColor(img_a, cv2.COLOR_RGB2BGR))
    cv2.imwrite(os.path.join(output_dir, 'source_b_training_h264.png'),
                cv2.cvtColor(img_b, cv2.COLOR_RGB2BGR))

    # Side-by-side with amplified difference
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    axes[0].imshow(img_a)
    axes[0].set_title('Source A: Rosbag Raw\n(JPEG decode -> resize -> RGB)', fontsize=10)
    axes[0].axis('off')

    axes[1].imshow(img_b)
    axes[1].set_title('Source B: Training H.264\n(H.264 yuv420p crf=23 decode -> RGB)', fontsize=10)
    axes[1].axis('off')

    diff = np.abs(img_a.astype(np.float64) - img_b.astype(np.float64))
    diff_amplified = np.clip(diff * 10, 0, 255).astype(np.uint8)
    axes[2].imshow(diff_amplified)
    axes[2].set_title(f'Abs Difference (x10)\nmean={diff.mean():.2f}, max={diff.max():.0f}', fontsize=10)
    axes[2].axis('off')

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'side_by_side.png'), dpi=150, bbox_inches='tight')
    plt.close()

    # Per-channel heatmaps and histograms
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))

    channel_names = ['Red', 'Green', 'Blue']
    colors = ['red', 'green', 'blue']
    for c, name in enumerate(channel_names):
        ch_diff = diff[:, :, c]

        im = axes[0, c].imshow(ch_diff, cmap='hot', vmin=0, vmax=max(diff.max(), 1))
        axes[0, c].set_title(f'{name} channel diff\nmean={ch_diff.mean():.2f}, max={ch_diff.max():.0f}')
        axes[0, c].axis('off')
        plt.colorbar(im, ax=axes[0, c], fraction=0.046, pad=0.04)

        axes[1, c].hist(ch_diff.flatten(), bins=50, color=colors[c], alpha=0.7)
        axes[1, c].set_title(f'{name} channel diff distribution')
        axes[1, c].set_xlabel('Absolute pixel difference')
        axes[1, c].set_ylabel('Count')
        axes[1, c].axvline(ch_diff.mean(), color='black', linestyle='--',
                           label=f'mean={ch_diff.mean():.2f}')
        axes[1, c].legend()

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'difference_heatmap.png'), dpi=150, bbox_inches='tight')
    plt.close()

    print(f'Saved visualizations to {output_dir}/')


def main():
    parser = argparse.ArgumentParser(
        description='Compare raw rosbag image vs H.264-encoded training image'
    )
    parser.add_argument(
        '--rosbag',
        default='/home/alfie/alfiebot_ws/data/demonstrations/demo_20260102_195919/demo_20260102_195919_0.mcap',
        help='Path to .mcap or .mcap.zstd rosbag file'
    )
    parser.add_argument(
        '--video',
        default='/home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge/videos/chunk-000/observation.images.left_wide/episode_000000.mp4',
        help='Path to training episode MP4 video'
    )
    parser.add_argument(
        '--output-dir',
        default=OUTPUT_DIR,
        help='Output directory for comparison results'
    )
    args = parser.parse_args()

    rosbag_path = Path(args.rosbag)
    video_path = Path(args.video)

    if not rosbag_path.exists():
        zstd_path = Path(str(rosbag_path) + '.zstd')
        if zstd_path.exists():
            rosbag_path = zstd_path
        else:
            print(f'ERROR: Rosbag not found: {rosbag_path}')
            sys.exit(1)

    if not video_path.exists():
        print(f'ERROR: Video not found: {video_path}')
        sys.exit(1)

    print('=' * 60)
    print('Raw JPEG vs H.264 Training Image Comparison')
    print('=' * 60)
    print(f'Rosbag: {rosbag_path}')
    print(f'Video:  {video_path}')
    print(f'Output: {args.output_dir}')
    print()

    print('Extracting first left_wide image from rosbag...')
    img_a = extract_rosbag_image(str(rosbag_path))
    print()

    print('Extracting first frame from training video...')
    img_b = extract_training_frame(str(video_path))
    print()

    assert img_a.shape == img_b.shape, f'Shape mismatch: {img_a.shape} vs {img_b.shape}'
    assert img_a.dtype == img_b.dtype == np.uint8, f'dtype mismatch: {img_a.dtype} vs {img_b.dtype}'

    stats = compute_statistics(img_a, img_b)
    print(stats)

    save_visualizations(img_a, img_b, args.output_dir)

    stats_path = os.path.join(args.output_dir, 'stats.txt')
    with open(stats_path, 'w') as f:
        f.write(stats)
    print(f'\nSaved stats to {stats_path}')
    print('\nDone.')


if __name__ == '__main__':
    main()
