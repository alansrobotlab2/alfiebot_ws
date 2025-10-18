"""Field configuration for servo parameters with detailed documentation."""

from dataclasses import dataclass
from typing import Tuple


@dataclass
class FieldConfig:
    """Configuration for a single editable field."""
    widget_name: str
    address: int
    range: Tuple[int, int]
    label: str
    unit: str
    default: int
    description: str
    is_eprom: bool = True  # True for EPROM (persistent), False for SRAM (volatile)


# EPROM Fields (persistent storage)
EPROM_FIELDS = [
    FieldConfig(
        'return_delay', 7, (0, 254), 
        'Return Delay', '2μs', 0,
        'Response delay after receiving command. Max: 508μs',
        is_eprom=True
    ),
    FieldConfig(
        'min_angle', 9, (-32766, 32767), 
        'Min Angle Limit', 'steps', -4095,
        'Minimum angle limit for motion range. Must be less than max angle. Set to 0 for multi-turn control.',
        is_eprom=True
    ),
    FieldConfig(
        'max_angle', 11, (-32766, 32767), 
        'Max Angle Limit', 'steps', 4095,
        'Maximum angle limit for motion range. Must be greater than min angle. Set to 0 for multi-turn control.',
        is_eprom=True
    ),
    FieldConfig(
        'max_temperature', 13, (0, 100), 
        'Max Temperature', '°C', 85,
        'Maximum safe operating temperature before protection triggers',
        is_eprom=True
    ),
    FieldConfig(
        'max_voltage', 14, (0, 254), 
        'Max Voltage', '0.1V', 140,
        'Maximum input voltage before protection (e.g., 140 = 14.0V)',
        is_eprom=True
    ),
    FieldConfig(
        'min_voltage', 15, (0, 254), 
        'Min Voltage', '0.1V', 60,
        'Minimum input voltage before protection (e.g., 60 = 6.0V)',
        is_eprom=True
    ),
    FieldConfig(
        'max_torque', 16, (0, 1000), 
        'Max Torque', '1.0%', 1000,
        'Maximum output torque limit. 1000 = 100% of rated torque',
        is_eprom=True
    ),
    FieldConfig(
        'p_coefficient', 21, (0, 254), 
        'P Coefficient', '', 32,
        'Position loop Proportional gain for servo control',
        is_eprom=True
    ),
    FieldConfig(
        'd_coefficient', 22, (0, 254), 
        'D Coefficient', '', 32,
        'Position loop Derivative gain for servo control',
        is_eprom=True
    ),
    FieldConfig(
        'i_coefficient', 23, (0, 254), 
        'I Coefficient', '', 32,
        'Position loop Integral gain for servo control',
        is_eprom=True
    ),
    FieldConfig(
        'minimum_start_force', 24, (0, 1000), 
        'Minimum Start Force', '1.0%', 0,
        'Minimum PWM torque output (dead zone elimination). Range 0-100%',
        is_eprom=True
    ),
    FieldConfig(
        'cw_insensitive_region', 26, (0, 32), 
        'CW Insensitive Region', 'steps', 1,
        'Clockwise dead zone to prevent oscillation near target. Max 32 steps',
        is_eprom=True
    ),
    FieldConfig(
        'ccw_insensitive_region', 27, (0, 32), 
        'CCW Insensitive Region', 'steps', 1,
        'Counter-clockwise dead zone to prevent oscillation near target. Max 32 steps',
        is_eprom=True
    ),
    FieldConfig(
        'protection_current', 28, (0, 511), 
        'Protection Current', '6.5mA', 300,
        'Overcurrent threshold. Max = 511 × 6.5mA = 3321.5mA',
        is_eprom=True
    ),
    FieldConfig(
        'angular_resolution', 30, (1, 3), 
        'Angular Resolution', 'steps/degree', 1,
        'Steps per degree: 1=lowest, 2=medium, 3=highest resolution',
        is_eprom=True
    ),
    FieldConfig(
        'position_correction', 31, (-2047, 2047), 
        'Position Correction', 'steps', 0,
        'Position offset/trim for calibration. Adjusts zero position',
        is_eprom=True
    ),
    FieldConfig(
        'operation_mode', 33, (0, 3), 
        'Operation Mode', '', 0,
        'Mode: 0=position servo, 1=motor constant speed, 2=PWM open-loop, 3=step servo',
        is_eprom=True
    ),
    FieldConfig(
        'protective_torque', 34, (0, 100), 
        'Protective Torque', '1.0%', 20,
        'Output torque after entering overload protection (20 = 20%)',
        is_eprom=True
    ),
    FieldConfig(
        'protection_time', 35, (0, 254), 
        'Protection Time', '10ms', 200,
        'Duration for current load to exceed overload torque before protection triggers. Max 2.54s',
        is_eprom=True
    ),
    FieldConfig(
        'overload_torque', 36, (0, 100), 
        'Overload Torque', '1.0%', 80,
        'Maximum torque threshold for starting overload protection countdown (80 = 80%)',
        is_eprom=True
    ),
    FieldConfig(
        'speed_p_coefficient', 37, (0, 100), 
        'Speed P Coefficient', '', 10,
        'Speed loop Proportional coefficient for motor constant speed mode (mode 1)',
        is_eprom=True
    ),
    FieldConfig(
        'overcurrent_protection', 38, (0, 254), 
        'Overcurrent Protection', '10ms', 200,
        'Overcurrent protection time. Max 2540ms',
        is_eprom=True
    ),
    FieldConfig(
        'velocity_i_coefficient', 39, (0, 254), 
        'Velocity I Coefficient', '1/10', 10,
        'Speed loop Integral coefficient for motor constant speed mode (mode 1)',
        is_eprom=True
    ),
]

# SRAM Fields (volatile, reset on power cycle)
SRAM_FIELDS = [
    FieldConfig(
        'acceleration', 41, (0, 254), 
        'Acceleration', '100 step/s²', 0,
        'Acceleration rate (e.g., 10 = 1000 step/s²). 0 = no limit',
        is_eprom=False
    ),
    FieldConfig(
        'target_location', 42, (-30719, 30719), 
        'Target Location', 'steps', 0,
        'Target position for absolute position control. Each step = min resolution angle',
        is_eprom=False
    ),
    FieldConfig(
        'running_time', 44, (0, 1000), 
        'Running Time', '0.1%', 0,
        'Operation time for PWM open-loop speed control mode. BIT10 is direction bit',
        is_eprom=False
    ),
    FieldConfig(
        'running_speed', 46, (0, 3400), 
        'Running Speed', 'step/s', 0,
        'Number of steps per second (50 step/s = 0.732 RPM)',
        is_eprom=False
    ),
    FieldConfig(
        'torque_limit', 48, (0, 1000), 
        'Torque Limit', '1.0%', 1000,
        'Current torque limit. Power-on default assigned from max torque (addr 0x10)',
        is_eprom=False
    ),
]

# Combine all editable fields
EDITABLE_FIELDS = EPROM_FIELDS + SRAM_FIELDS

# Read-only status fields
STATUS_FIELDS = [
    'baudrate', 'servo_id', 'firmware_major', 'firmware_minor',
    'servo_main_version', 'servo_sub_version', 'lock_mark',
    'current_speed', 'servo_status', 'acceleration_status',
    'current_location', 'current_temperature', 'mobile_sign',
    'current_current', 'current_voltage', 'torque_switch', 'target_location_status'
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
