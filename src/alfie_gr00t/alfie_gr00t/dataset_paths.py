"""Where episode data lives on disk.

Kept at the package root (not under core/) so ROS nodes running on plain
system python can import it without pulling in the ZMQ/msgpack client stack.

Every dataset (one challenge, one task family, one experiment) gets its own
folder under a common data root, named by a short ``dataset`` slug:

    <data_root>/<dataset>/
        demonstrations/   raw MCAP bags written by data_recorder (demo_YYYYMMDD_HHMMSS/)
        lerobot/          GR00T-flavoured LeRobot v2 output of rosbag_to_groot.py
                          (meta/, data/, videos/) -- pass this to --dataset-path

The recorder, the converter and the episode manager all derive their paths from here so a dataset is chosen in exactly one place.
``ALFIE_DATA_ROOT`` and ``ALFIE_DATASET`` override the defaults for scripts run
outside ROS launch.
"""

import os
from pathlib import Path

DEFAULT_DATA_ROOT = '~/alfiebot_ws/data'
DEFAULT_DATASET = 'default'

DEMOS_SUBDIR = 'demonstrations'
LEROBOT_SUBDIR = 'lerobot'


def data_root(root: str | None = None) -> Path:
    """Expanded data root: explicit arg, then $ALFIE_DATA_ROOT, then the default."""
    return Path(os.path.expanduser(root or os.environ.get('ALFIE_DATA_ROOT') or DEFAULT_DATA_ROOT))


def dataset_name(name: str | None = None) -> str:
    """Dataset slug: explicit arg, then $ALFIE_DATASET, then the default."""
    return name or os.environ.get('ALFIE_DATASET') or DEFAULT_DATASET


def dataset_dir(dataset: str | None = None, root: str | None = None) -> Path:
    return data_root(root) / dataset_name(dataset)


def demos_dir(dataset: str | None = None, root: str | None = None) -> Path:
    """Raw rosbag demonstrations for a dataset."""
    return dataset_dir(dataset, root) / DEMOS_SUBDIR


def lerobot_dir(dataset: str | None = None, root: str | None = None) -> Path:
    """Converted GR00T LeRobot v2 dataset (the ``--dataset-path`` for Isaac-GR00T)."""
    return dataset_dir(dataset, root) / LEROBOT_SUBDIR
