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

# ---------------------------------------------------------------------------
# Physical bus servos
# ---------------------------------------------------------------------------
# The register map is addressed per PHYSICAL servo, not per logical joint. The
# arms run 7 servos behind 6 joints: bus 2 and bus 3 are the mirrored
# shoulder-pitch pair, and only bus 2 (the primary) has its feedback reported in
# ArmState. Bus 3 is invisible over the state topic - the register read is the
# only way to see it at all, which is exactly when you want this tool.


@dataclass
class BusServo:
    """One physical servo on a module's serial bus."""

    subsystem: str
    bus_id: int                 # 1-based id on the wire
    label: str
    joint_index: Optional[int]  # logical joint that drives it, None if unmapped
    mirrored: bool = False      # driven as the negated mirror of its joint
    reported: bool = True       # appears in ArmState/HeadState feedback

    @property
    def key(self) -> str:
        return f'{self.subsystem}.bus{self.bus_id}'


def _arm_bus_servos(subsystem: str) -> List[BusServo]:
    joints = JOINTS_BY_SUBSYSTEM[subsystem]
    return [
        BusServo(subsystem, 1, joints[0].label, 0),
        BusServo(subsystem, 2, f'{joints[1].label} (primary)', 1),
        BusServo(subsystem, 3, f'{joints[1].label} (derived)', 1,
                 mirrored=True, reported=False),
        BusServo(subsystem, 4, joints[2].label, 2),
        BusServo(subsystem, 5, joints[3].label, 3),
        BusServo(subsystem, 6, joints[4].label, 4),
        BusServo(subsystem, 7, joints[5].label, 5),
    ]


def _head_bus_servos() -> List[BusServo]:
    joints = JOINTS_BY_SUBSYSTEM[HEAD]
    return [BusServo(HEAD, i + 1, j.label, i) for i, j in enumerate(joints)]


BUS_SERVOS: Dict[str, List[BusServo]] = {
    LEFT_ARM: _arm_bus_servos(LEFT_ARM),
    RIGHT_ARM: _arm_bus_servos(RIGHT_ARM),
    HEAD: _head_bus_servos(),
}


def bus_servo(subsystem: str, bus_id: int) -> Optional[BusServo]:
    for servo in BUS_SERVOS.get(subsystem, []):
        if servo.bus_id == bus_id:
            return servo
    return None


# ---------------------------------------------------------------------------
# Register map presentation
# ---------------------------------------------------------------------------
# Field names match ServoMemoryMap.msg; addresses and semantics come from the
# ST3215 vendor table (alfie_tools/servotool/ST3215 memory register map-EN.csv).
#
# `kind` is what the UI keys its "is this live?" treatment off:
#   eeprom   - persistent, only changes when something writes it
#   sram     - volatile command registers; the firmware rewrites these every tick
#   feedback - measured, changes constantly
#
# `scale` renders the engineering value beside the raw register value: the servo
# stores 80 for 8.0 V, and reading "80 V" off a screen at 2 a.m. is how limits
# get set wrong.

# Lock flag semantics (register 0x37): writing 0 DISABLES the write lock, so
# 0 means EEPROM is writable and 1 means it is protected.
LOCK_UNLOCKED = 0
LOCK_LOCKED = 1


@dataclass
class Register:
    """One entry in the servo register map, as the UI shows it."""

    field: str                  # ServoMemoryMap field name
    address: int
    label: str
    group: str
    kind: str                   # eeprom | sram | feedback
    unit: str = ''
    scale: float = 0.0          # engineering value = raw * scale (0 = none)
    scaled_unit: str = ''
    note: str = ''
    # Writable registers must match the firmware's WRITABLE_REGS table exactly;
    # the firmware is the authority and rejects anything else, this just keeps
    # the UI from offering edits that will bounce.
    writable: bool = False
    vmin: int = 0
    vmax: int = 0


# Write outcome codes, mirroring SERVO_WRITE_* in the module firmware config
# and documented on ServoMemoryMap.msg.
WRITE_RESULTS = {
    0: 'no write attempted',
    1: 'written',
    2: 'that register is not writable',
    3: 'EEPROM is locked - unlock it first',
    4: 'torque is on - turn it off before writing EEPROM',
    5: 'the servo did not acknowledge the write',
    6: 'wrote, but the read-back disagrees',
    7: 'value does not fit the register',
}
WRITE_OK = 1


REGISTER_GROUPS = [
    ('identity', 'Identity & bus'),
    ('travel', 'Travel limits'),
    ('protection', 'Protection'),
    ('pid', 'Position loop'),
    ('mode', 'Mode & speed loop'),
    ('sram', 'SRAM (command)'),
    ('feedback', 'Feedback'),
]

REGISTERS: List[Register] = [
    # -- identity -----------------------------------------------------------
    Register('firmwaremajor', 0, 'Firmware major', 'identity', 'eeprom'),
    Register('firmwaresub', 1, 'Firmware minor', 'identity', 'eeprom'),
    Register('servomajor', 3, 'Servo major', 'identity', 'eeprom'),
    Register('servosub', 4, 'Servo minor', 'identity', 'eeprom'),
    Register('servoid', 5, 'Servo ID', 'identity', 'eeprom',
             note='Set with picosetservoid, one servo alone on the bus.'),
    Register('baudrate', 6, 'Baud rate', 'identity', 'eeprom',
             note='0 = 1 Mbps, which is what the driver boards run.'),
    Register('returndelay', 7, 'Return delay', 'identity', 'eeprom', unit='2 us',
             writable=True, vmin=0, vmax=254),
    Register('responsestatuslevel', 8, 'Response level', 'identity', 'eeprom',
             writable=True, vmin=0, vmax=1),
    # -- travel -------------------------------------------------------------
    Register('minanglelimit', 9, 'Min angle limit', 'travel', 'eeprom', unit='counts',
             note='Cached by the firmware at boot; a change needs a board reset.',
             writable=True, vmin=0, vmax=4096),
    Register('maxanglelimit', 11, 'Max angle limit', 'travel', 'eeprom', unit='counts',
             note='Cached by the firmware at boot; a change needs a board reset.',
             writable=True, vmin=0, vmax=4096),
    Register('positioncorrection', 31, 'Position correction', 'travel', 'eeprom',
             unit='counts raw',
             note='Zero-point trim, sign-MAGNITUDE not two\'s complement: bit 11 '
                  'is the direction bit, so 0-2047 is positive and 2048+n means '
                  '-n (4095 = -2047). Written through raw.',
             writable=True, vmin=0, vmax=4095),
    Register('angularresolution', 30, 'Angular resolution', 'travel', 'eeprom',
             note='Sensor resolution multiplier, 1-3.',
             writable=True, vmin=1, vmax=3),
    # -- protection ---------------------------------------------------------
    Register('maxtemplimit', 13, 'Max temperature', 'protection', 'eeprom', unit='C',
             writable=True, vmin=0, vmax=100),
    Register('maxinputvoltage', 14, 'Max input voltage', 'protection', 'eeprom',
             unit='0.1 V', scale=0.1, scaled_unit='V',
             writable=True, vmin=0, vmax=254),
    Register('mininputvoltage', 15, 'Min input voltage', 'protection', 'eeprom',
             unit='0.1 V', scale=0.1, scaled_unit='V',
             writable=True, vmin=0, vmax=254),
    Register('maxtorque', 16, 'Max torque', 'protection', 'eeprom', unit='0.1 %',
             scale=0.1, scaled_unit='%',
             note='Power-on value of the torque limit register.',
             writable=True, vmin=0, vmax=1000),
    Register('protectioncurrent', 28, 'Protection current', 'protection', 'eeprom',
             unit='6.5 mA', scale=6.5, scaled_unit='mA',
             writable=True, vmin=0, vmax=511),
    Register('protectivetorque', 34, 'Protective torque', 'protection', 'eeprom', unit='%',
             note='Torque held after overload protection trips.',
             writable=True, vmin=0, vmax=100),
    Register('protectiontime', 35, 'Protection time', 'protection', 'eeprom',
             unit='10 ms', scale=0.01, scaled_unit='s',
             writable=True, vmin=0, vmax=254),
    Register('overloadtorque', 36, 'Overload torque', 'protection', 'eeprom', unit='%',
             writable=True, vmin=0, vmax=100),
    Register('overcurrentprotectiontime', 38, 'Overcurrent time', 'protection', 'eeprom',
             unit='10 ms', scale=0.01, scaled_unit='s',
             writable=True, vmin=0, vmax=254),
    Register('unloadingcondition', 19, 'Unloading condition', 'protection', 'eeprom',
             note='Bitfield: which faults torque off. Bits as in servo_status.',
             writable=True, vmin=0, vmax=63),
    Register('ledalarmcondition', 20, 'LED alarm condition', 'protection', 'eeprom',
             note='Bitfield: which faults light the servo LED.',
             writable=True, vmin=0, vmax=63),
    # -- position loop ------------------------------------------------------
    Register('pcoefficient', 21, 'P coefficient', 'pid', 'eeprom',
             writable=True, vmin=0, vmax=254),
    Register('dcoefficient', 22, 'D coefficient', 'pid', 'eeprom',
             writable=True, vmin=0, vmax=254),
    Register('icoefficient', 23, 'I coefficient', 'pid', 'eeprom',
             writable=True, vmin=0, vmax=254),
    Register('minstartupforce', 24, 'Min startup force', 'pid', 'eeprom', unit='0.1 %',
             scale=0.1, scaled_unit='%',
             writable=True, vmin=0, vmax=1000),
    Register('clockwiseinsensitivearea', 26, 'CW dead zone', 'pid', 'eeprom', unit='counts',
             writable=True, vmin=0, vmax=32),
    Register('counterclockwiseinsensitiveregion', 27, 'CCW dead zone', 'pid', 'eeprom',
             unit='counts',
             writable=True, vmin=0, vmax=32),
    # -- mode ---------------------------------------------------------------
    Register('operationmode', 33, 'Operation mode', 'mode', 'eeprom',
             note='0 position, 1 constant speed, 2 PWM open loop, 3 step.',
             writable=True, vmin=0, vmax=3),
    Register('phase', 18, 'Phase', 'mode', 'eeprom',
             writable=True, vmin=0, vmax=255),
    Register('speedclosedlooppcoefficient', 37, 'Speed loop P', 'mode', 'eeprom',
             writable=True, vmin=0, vmax=100),
    Register('velocityclosedloopicoefficient', 39, 'Speed loop I', 'mode', 'eeprom',
             writable=True, vmin=0, vmax=254),
    # -- SRAM (rewritten by the firmware every servo tick) ------------------
    Register('torqueswitch', 40, 'Torque switch', 'sram', 'sram'),
    Register('acceleration', 41, 'Acceleration', 'sram', 'sram', unit='100 count/s^2'),
    Register('targetlocation', 42, 'Target location', 'sram', 'sram', unit='counts'),
    Register('runningtime', 44, 'Running time', 'sram', 'sram'),
    Register('runningspeed', 46, 'Running speed', 'sram', 'sram', unit='count/s'),
    Register('torquelimit', 48, 'Torque limit', 'sram', 'sram', unit='0.1 %',
             scale=0.1, scaled_unit='%'),
    Register('lockmark', 55, 'Lock flag', 'sram', 'sram',
             note='0 unlocks EEPROM writes, 1 protects them.'),
    # -- feedback -----------------------------------------------------------
    Register('currentlocation', 56, 'Current location', 'feedback', 'feedback', unit='counts'),
    Register('currentspeed', 58, 'Current speed', 'feedback', 'feedback', unit='count/s'),
    Register('currentload', 60, 'Current load', 'feedback', 'feedback', unit='0.1 %',
             scale=0.1, scaled_unit='%'),
    Register('currentvoltage', 62, 'Current voltage', 'feedback', 'feedback', unit='0.1 V',
             scale=0.1, scaled_unit='V'),
    Register('currenttemperature', 63, 'Current temperature', 'feedback', 'feedback', unit='C'),
    Register('currentcurrent', 69, 'Current draw', 'feedback', 'feedback', unit='6.5 mA',
             scale=6.5, scaled_unit='mA'),
    Register('servostatus', 65, 'Servo status', 'feedback', 'feedback',
             note='Fault bitfield; see the decoded flags above.'),
    Register('mobilesign', 66, 'Moving flag', 'feedback', 'feedback'),
    Register('asyncwriteflag', 64, 'Async write flag', 'feedback', 'feedback'),
]

REGISTERS_BY_FIELD = {r.field: r for r in REGISTERS}


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
