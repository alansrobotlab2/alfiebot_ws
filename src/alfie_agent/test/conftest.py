"""Make alfie_gr00t's task registry importable from the source tree.

The agent's skills read it; without a sourced (and rebuilt) workspace the
sibling package is used directly.
"""
import importlib.util
import sys
from pathlib import Path


def _has_registry():
    try:
        return importlib.util.find_spec("alfie_gr00t.task_registry") is not None
    except ModuleNotFoundError:
        return False


if not _has_registry():
    # A stale install can leave an empty alfie_gr00t namespace on the path;
    # drop it so the source package is the one imported.
    sys.modules.pop("alfie_gr00t", None)
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "alfie_gr00t"))
