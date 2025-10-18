"""
Launch file for joydrive node with joy_node

This launch file starts both the joy_node (to read the USB joystick)
and the joydrive node (to convert joystick commands to robot drive commands).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Joy node to read USB joystick
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,  # Usually /dev/input/js0
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,  # 20 Hz
            }],
            output='screen',
        ),
        
        # Joydrive node to convert joy to robot commands (arcade-style control + head servos + eyes)
        Node(
            package='alfie_tools',
            executable='joydrive',
            name='joydrive_node',
            parameters=[{
                'throttle_axis': 1,   # Left stick Y-axis (forward/backward)
                'steering_axis': 0,   # Left stick X-axis (left/right turning)
                'head_yaw_axis': 3,   # Right stick X-axis (head left/right)
                'head_pitch_axis': 4, # Right stick Y-axis (head up/down)
                'eye_trigger_axis': 5, # Right trigger (eye brightness, axis 5 = RT)
                'max_pwm': 255,
                'deadzone': 0.05,
                'invert_throttle': False,
                'invert_steering': True,  # Invert so left = left turn, right = right turn
                'invert_head_yaw': False,
                'invert_head_pitch': False,
            }],
            output='screen',
        ),
    ])
