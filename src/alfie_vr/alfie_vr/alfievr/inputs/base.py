"""
Base classes and data structures for input providers.
"""

import asyncio
import time
import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Literal, Dict, Any


@dataclass
class ControlGoal:
    """Latest input for one tracked device, produced by an input provider.

    arm is "left"/"right" (controllers), "headset", or "release" (the client
    asked to drop control of the hands listed in metadata["hands"]).
    """
    arm: Literal["left", "right", "headset", "release"]
    target_position: Optional[np.ndarray] = None  # VR position (m), WebXR local space
    wrist_roll_deg: Optional[float] = None        # Roll relative to tracking origin
    wrist_flex_deg: Optional[float] = None        # Pitch relative to tracking origin

    # Raw per-device data: buttons, thumbstick, trigger, grip_active, quaternion...
    metadata: Optional[Dict[str, Any]] = None

    # time.monotonic() when the server received the message (staleness checks)
    stamp: float = field(default_factory=time.monotonic)


class BaseInputProvider(ABC):
    """Abstract base class for input providers."""

    def __init__(self, command_queue: asyncio.Queue):
        self.command_queue = command_queue
        self.is_running = False

    @abstractmethod
    async def start(self):
        """Start the input provider."""
        pass

    @abstractmethod
    async def stop(self):
        """Stop the input provider."""
        pass

    async def send_goal(self, goal: ControlGoal):
        """Send a control goal to the command queue."""
        await self.command_queue.put(goal)
