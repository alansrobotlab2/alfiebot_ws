#!/usr/bin/env python3
"""
Configuration class for Alfie Teleop VR node.
Centralizes all configurable values for easier management.
"""

from dataclasses import dataclass


@dataclass
class AlfieTeleopVRConfig:
    """Configuration values for the Alfie Teleop VR node."""
    
    # Velocity limits
    max_linear_vel: float = 0.15  # m/s - maximum linear velocity for joystick control
    max_angular_vel: float = 1.00  # rad/s - maximum angular velocity for joystick control
    
    # VR to robot workspace scale factors
    vr_x_scale: float = 40.0  # Scale for shoulder rotate (VR X movement)
    vr_y_scale: float = 20.0  # Scale for arm reach Y
    vr_z_scale: float = 20.0  # Scale for arm reach Z->X
    
    # Delta control parameters
    pos_scale: float = 0.05  # Position sensitivity scaling
    angle_scale: float = 0.07  # Angle sensitivity scaling (radians, ~4 degrees)
    wrist_flex_scale: float = 4.0  # Wrist flex (pitch) amplification factor
    wrist_roll_scale: float = 2.0  # Wrist roll amplification factor
    delta_limit: float = 0.02  # Maximum delta per update (meters)
    angle_limit: float = 0.14  # Maximum angle delta per update (radians, ~8 degrees)
    
    # Head scaling parameters
    head_yaw_scale: float = 2.0  # Scale factor for head yaw
    head_pitch_scale: float = 2.0  # Scale factor for head pitch
    head_roll_scale: float = 1.0  # Scale factor for head roll
