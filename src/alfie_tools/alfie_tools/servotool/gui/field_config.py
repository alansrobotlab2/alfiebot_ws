"""Field configuration for servo parameters."""

from dataclasses import dataclass
from typing import Tuple


@dataclass
class FieldConfig:
    """Configuration for a single editable field."""
    widget_name: str
    address: int
    range: Tuple[int, int]


# All editable fields with their addresses and valid ranges
EDITABLE_FIELDS = [
    FieldConfig('txtReturnDelay', 7, (0, 254)),
    FieldConfig('txtMinAngle', 9, (-32766, 32767)),
    FieldConfig('txtMaxAngle', 11, (-32766, 32767)),
    FieldConfig('txtMaxTemperature', 13, (0, 100)),
    FieldConfig('txtMaxVoltage', 14, (0, 254)),
    FieldConfig('txtMinVoltage', 15, (0, 254)),
    FieldConfig('txtMaxTorque', 16, (0, 1000)),
    FieldConfig('txtPCoefficient', 21, (0, 254)),
    FieldConfig('txtDCoefficient', 22, (0, 254)),
    FieldConfig('txtICoefficient', 23, (0, 254)),
    FieldConfig('txtMinimumStartForce', 24, (0, 1000)),
    FieldConfig('txtClockwiseInsensitiveRegion', 26, (0, 32)),
    FieldConfig('txtCounterClockwiseInsensitiveRegion', 27, (0, 32)),
    FieldConfig('txtProtectionCurrent', 28, (0, 511)),
    FieldConfig('txtAngularResolution', 30, (1, 3)),
    FieldConfig('txtPositionCorrection', 31, (-2047, 2047)),
    FieldConfig('txtOperationMode', 33, (0, 255)),
    FieldConfig('txtProtectiveTorque', 34, (0, 100)),
    FieldConfig('txtProtectionTime', 35, (0, 254)),
    FieldConfig('txtOverloadTorque', 36, (0, 100)),
    FieldConfig('txtSpeedPCoefficient', 37, (0, 100)),
    FieldConfig('txtOvercurrentProtection', 38, (0, 254)),
    FieldConfig('txtVelocityICoefficient', 39, (0, 254)),
    FieldConfig('txtAcceleration', 41, (0, 100)),
    FieldConfig('txtTargetLocation', 42, (-30719, 30719)),
    FieldConfig('txtRunningTime', 44, (0, 1000)),
    FieldConfig('txtRunningSpeed', 46, (0, 3400)),
    FieldConfig('txtTorqueLimit', 48, (0, 1000)),
]

# Servo dropdown configuration (display_name, bus, id)
SERVO_CONFIG = [
    ("driver1/servo01 - left shoulder yaw", 1, 1),
    ("driver1/servo02 - left shoulder1 pitch", 1, 2),
    ("driver1/servo03 - left shoulder2 pitch", 1, 3),
    ("driver1/servo04 - left elbow pitch", 1, 4),
    ("driver1/servo05 - left wrist pitch", 1, 5),
    ("driver1/servo06 - left wrist roll", 1, 6),
    ("driver1/servo07 - left hand", 1, 7),
    ("driver0/servo01 - right shoulder yaw", 0, 1),
    ("driver0/servo02 - right shoulder1 pitch", 0, 2),
    ("driver0/servo03 - right shoulder2 pitch", 0, 3),
    ("driver0/servo04 - right elbow pitch", 0, 4),
    ("driver0/servo05 - right wrist pitch", 0, 5),
    ("driver0/servo06 - right wrist roll", 0, 6),
    ("driver0/servo07 - right hand", 0, 7),
    ("driver0/servo08 - head yaw", 0, 8),
    ("driver0/servo09 - head pitch", 0, 9),
    ("driver0/servo10 - head roll", 0, 10),
]

# Field to address mapping (for easy lookup)
FIELD_ADDRESS_MAP = {field.widget_name: field.address for field in EDITABLE_FIELDS}

# Field to range mapping (for easy lookup)
FIELD_RANGE_MAP = {field.widget_name: field.range for field in EDITABLE_FIELDS}
