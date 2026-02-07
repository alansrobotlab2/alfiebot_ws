# GR00T Client Core Modules
"""Core modules for GR00T N1.6 inference client."""

from .normalization import Normalizer
from .zmq_client import ZMQClient

# Lazy imports for ROS2-dependent modules to allow standalone (non-ROS) usage
def __getattr__(name):
    if name == "ObservationBridge":
        from .observation_bridge import ObservationBridge
        return ObservationBridge
    if name == "ActionPublisher":
        from .action_publisher import ActionPublisher
        return ActionPublisher
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = [
    'Normalizer',
    'ZMQClient',
    'ObservationBridge',
    'ActionPublisher',
]
