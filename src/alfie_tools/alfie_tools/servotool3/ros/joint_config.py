"""Static description of Alfie's gen2 servo layout, as servotool3 presents it.

Gen2 hardware splits actuation across independent Pico driver boards. There is
no GDBState / GDBServoService any more, and with them went register-level access
to the servos: the firmware owns the memory map and exposes only SI-unit command
and state messages. servotool3 is therefore built around what gen2 actually
publishes:

    left arm   alfie/low/left_arm/armstate    ArmState   (6 logical joints)
    right arm  alfie/low/right_arm/armstate   ArmState   (6 logical joints)
    head       alfie/low/headstate            HeadState  (3 servos + 2 eye LEDs)
    back       alfie/low/backstate            BackState  (linear actuator + IMU)

Angles are radians, speeds rad/s, accelerations rad/s^2, and per-servo polarity
is already applied by the module firmware, so nothing here converts units.

The arms expose 6 logical joints over 7 physical bus servos: the firmware
expands shoulder-pitch into a mirrored coupled pair (bus ids 2 and 3). The bus
ids below are informational only - they cannot be addressed individually from
ROS in gen2.
"""

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Subsystems (mirrors command_mux's subsystem names, which are also the
# cmd/<subsystem>/<source> topic segments)
# ---------------------------------------------------------------------------
LEFT_ARM = 'left_arm'
RIGHT_ARM = 'right_arm'
HEAD = 'head'
EYES = 'eyes'
BACK = 'back'

SERVO_SUBSYSTEMS = (LEFT_ARM, RIGHT_ARM, HEAD)
ALL_SUBSYSTEMS = (LEFT_ARM, RIGHT_ARM, HEAD, EYES, BACK)

SUBSYSTEM_LABELS = {
    LEFT_ARM: 'Left arm',
    RIGHT_ARM: 'Right arm',
    HEAD: 'Head',
    EYES: 'Eyes',
    BACK: 'Back / spine',
}

# ---------------------------------------------------------------------------
# Servo command scaling (see picoarmdriver/include/config.h)
# ---------------------------------------------------------------------------
# ServoCmd.target_torque is the raw STS torque-limit register: 0..1000, where
# 1000 is 100% of locked-rotor torque. 0 means *no* torque, not "default", so a
# command that leaves it at zero produces a limp servo.
TORQUE_MAX = 1000.0
DEFAULT_TORQUE = 1000.0

# target_speed = 0 means "unlimited" to the servo firmware, which is how a
# freshly commanded joint snaps. servotool3 always sends a real speed.
DEFAULT_SPEED = 1.0            # rad/s
MAX_SPEED = 6.0                # rad/s, matches command_mux's max_servo_speed
DEFAULT_ACCEL = 0.0            # rad/s^2, 0 = servo's own acceleration limit
MAX_ACCEL = 50.0               # rad/s^2

# Eye LEDs are raw PWM duty through a TB6612FNG at 100 Hz.
EYE_PWM_MAX = 4095

# Back actuator (picobackdriver/include/config.h). ACTUATOR_MAX_POSITION is the
# measured stroke and is shorter than the URDF's nominal 0.39 m; use the
# firmware number, it is the one that bounds the hardware.
BACK_MIN_POSITION = 0.0
BACK_MAX_POSITION = 0.35
BACK_MAX_VELOCITY = 0.046
BACK_MAX_ACCELERATION = 0.35
BACK_DEFAULT_VELOCITY = 0.03
BACK_DEFAULT_ACCELERATION = 0.10

# ---------------------------------------------------------------------------
# Servo status bits (STS/SMS status register), same decode as
# alfie_bringup/watchdog_checks.py
# ---------------------------------------------------------------------------
SERVO_STATUS_FLAGS: Tuple[Tuple[int, str], ...] = (
    (1, 'voltage'),
    (2, 'sensor'),
    (4, 'temperature'),
    (8, 'current'),
    (16, 'angle'),
    (32, 'overload'),
)

# Faults that master_low_status auto-estops on; worth calling out in the UI.
CRITICAL_STATUS_BITS = 4 | 32   # temperature | overload


def decode_status(status: int) -> List[str]:
    """Return the names of the set fault bits in an STS status byte."""
    return [name for bit, name in SERVO_STATUS_FLAGS if status & bit]


# ---------------------------------------------------------------------------
# Joint table
# ---------------------------------------------------------------------------

@dataclass
class JointInfo:
    """One logical joint as commanded over ROS."""

    subsystem: str
    index: int                  # index into ArmCmd.joint_cmd / HeadCmd.servos
    label: str                  # short human label
    urdf_joint: str             # joint name in alfie_urdf (also the JointState name)
    bus_ids: Tuple[int, ...]    # physical servo ids behind this joint
    lower: float = -math.pi     # rad, from the URDF when available
    upper: float = math.pi

    @property
    def key(self) -> str:
        """Stable id used by the web UI, e.g. 'left_arm.2'."""
        return f'{self.subsystem}.{self.index}'


# Arm joint order is fixed by ArmCmd; bus ids come from the firmware's
# SERVO_JOINT_MAP (shoulder pitch drives the coupled pair 2+3).
_ARM_JOINTS = (
    ('Shoulder yaw', 'shoulder_yaw_joint', (1,)),
    ('Shoulder pitch', 'shoulder_pitch_joint', (2, 3)),
    ('Elbow pitch', 'elbow_pitch_joint', (4,)),
    ('Wrist pitch', 'wrist_pitch_joint', (5,)),
    ('Wrist roll', 'wrist_roll_joint', (6,)),
    ('Gripper', 'gripper_active_joint', (7,)),
)

# Head array order is fixed by HeadCmd: [0] pan (bus 1), [1] tilt (bus 2),
# [2] roll (bus 3).
_HEAD_JOINTS = (
    ('Yaw / pan', 'head_yaw_joint', (1,)),
    ('Pitch / tilt', 'head_pitch_joint', (2,)),
    ('Roll', 'head_roll_joint', (3,)),
)


def _build_joints() -> List[JointInfo]:
    joints: List[JointInfo] = []
    for subsystem, prefix in ((LEFT_ARM, 'left_'), (RIGHT_ARM, 'right_')):
        for index, (label, suffix, bus_ids) in enumerate(_ARM_JOINTS):
            joints.append(JointInfo(subsystem, index, label, prefix + suffix, bus_ids))
    for index, (label, urdf_joint, bus_ids) in enumerate(_HEAD_JOINTS):
        joints.append(JointInfo(HEAD, index, label, urdf_joint, bus_ids))
    return joints


JOINTS: List[JointInfo] = _build_joints()

JOINTS_BY_SUBSYSTEM: Dict[str, List[JointInfo]] = {
    subsystem: [j for j in JOINTS if j.subsystem == subsystem]
    for subsystem in SERVO_SUBSYSTEMS
}

JOINT_COUNTS = {subsystem: len(js) for subsystem, js in JOINTS_BY_SUBSYSTEM.items()}


# ---------------------------------------------------------------------------
# URDF limits
# ---------------------------------------------------------------------------

def _urdf_path() -> Optional[str]:
    """Locate alfiebot.urdf in the installed alfie_urdf share directory."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('alfie_urdf'),
                            'urdf', 'alfiebot.urdf')
    except Exception:
        return None
    return path if os.path.isfile(path) else None


def apply_urdf_limits(logger=None) -> bool:
    """Override the joint travel limits from the URDF, in place.

    Falls back to the +/-pi defaults (and leaves continuous joints alone) when
    the URDF is not installed or a joint is missing from it. Returns True if
    any limit was applied.
    """
    path = _urdf_path()
    if path is None:
        if logger is not None:
            logger.warn('alfie_urdf not found; using default +/-pi joint limits')
        return False

    try:
        root = ET.parse(path).getroot()
    except Exception as exc:                       # malformed / unreadable URDF
        if logger is not None:
            logger.warn(f'could not parse {path}: {exc}; using default joint limits')
        return False

    limits: Dict[str, Tuple[float, float]] = {}
    for joint in root.iter('joint'):
        limit = joint.find('limit')
        name = joint.get('name')
        if limit is None or name is None:
            continue                               # continuous / fixed joints
        try:
            limits[name] = (float(limit.get('lower')), float(limit.get('upper')))
        except (TypeError, ValueError):
            continue

    applied = 0
    for joint in JOINTS:
        bounds = limits.get(joint.urdf_joint)
        if bounds is not None and bounds[0] < bounds[1]:
            joint.lower, joint.upper = bounds
            applied += 1

    if logger is not None:
        logger.info(f'applied URDF limits to {applied}/{len(JOINTS)} joints from {path}')
    return applied > 0
