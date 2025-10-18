#!/usr/bin/env python3
"""
Joydrive Node - Arcade-style drive control using a USB joystick

This node subscribes to /joy topic and publishes RobotLowCmd messages
to control the robot using arcade-style controls.
- Left stick Y-axis: throttle (forward/backward)
- Left stick X-axis: steering (left/right turning)
- Right stick X-axis: head yaw (servo 14, left/right)
- Right stick Y-axis: head pitch (servo 15, up/down)
- Right trigger: eye brightness (1-255 based on trigger depression)

PWM values range from -255 to 255, where 0 is stop, positive is forward, negative is reverse.
Head servos range from -90 to 90 degrees (converted to radians).
Eye PWM ranges from 1 (idle) to 255 (full brightness when trigger pressed).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from alfie_msgs.msg import RobotLowCmd, ServoCmd
import math


class JoyDriveNode(Node):
    def __init__(self):
        super().__init__('joydrive_node')
        
        # Declare parameters for arcade-style control
        self.declare_parameter('throttle_axis', 1)  # Y-axis for forward/backward (axis 1 = left stick Y)
        self.declare_parameter('steering_axis', 0)  # X-axis for left/right turning (axis 0 = left stick X)
        self.declare_parameter('head_yaw_axis', 3)  # Right stick X-axis for head yaw
        self.declare_parameter('head_pitch_axis', 4)  # Right stick Y-axis for head pitch
        self.declare_parameter('eye_trigger_axis', 5)  # Right trigger for eye brightness (typical axis 5 = RT)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('deadzone', 0.1)  # Deadzone to ignore small stick movements
        self.declare_parameter('invert_throttle', False)
        self.declare_parameter('invert_steering', False)
        self.declare_parameter('invert_head_yaw', False)
        self.declare_parameter('invert_head_pitch', False)
        
        # Get parameters
        self.throttle_axis = self.get_parameter('throttle_axis').value
        self.steering_axis = self.get_parameter('steering_axis').value
        self.head_yaw_axis = self.get_parameter('head_yaw_axis').value
        self.head_pitch_axis = self.get_parameter('head_pitch_axis').value
        self.eye_trigger_axis = self.get_parameter('eye_trigger_axis').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.deadzone = self.get_parameter('deadzone').value
        self.invert_throttle = self.get_parameter('invert_throttle').value
        self.invert_steering = self.get_parameter('invert_steering').value
        self.invert_head_yaw = self.get_parameter('invert_head_yaw').value
        self.invert_head_pitch = self.get_parameter('invert_head_pitch').value
        
        # Create subscriber for joystick input
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Create publisher for robot low-level commands
        self.cmd_pub = self.create_publisher(
            RobotLowCmd,
            '/alfie/robotlowcmd',
            10
        )
        
        self.get_logger().info('Joydrive node started (Arcade-style control + Head servos + Eye brightness)')
        self.get_logger().info(f'Drive - Throttle axis: {self.throttle_axis}, Steering axis: {self.steering_axis}')
        self.get_logger().info(f'Head - Yaw axis: {self.head_yaw_axis}, Pitch axis: {self.head_pitch_axis}')
        self.get_logger().info(f'Eye - Trigger axis: {self.eye_trigger_axis}')
        self.get_logger().info(f'Max PWM: {self.max_pwm}, Deadzone: {self.deadzone}')
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def map_axis_to_pwm(self, axis_value):
        """
        Map joystick axis value (-1.0 to 1.0) to PWM value (-255 to 255)
        
        Joystick convention:
        - Forward (up): positive value (+1.0)
        - Backward (down): negative value (-1.0)
        
        PWM convention:
        - 255: full forward
        - 0: stop
        - -255: full backward
        """
        # Apply deadzone
        axis_value = self.apply_deadzone(axis_value)
        
        # Map from [-1, 1] to [-255, 255]
        # 1.0 (forward) -> 255
        # 0.0 (neutral) -> 0
        # -1.0 (backward) -> -255
        pwm = int(axis_value * 255)
        
        # Clamp to valid range
        pwm = max(-255, min(255, pwm))
        
        return pwm
    
    def joy_callback(self, msg):
        """Process joystick input and publish drive commands using arcade-style control"""
        # Check if we have enough axes
        required_axes = max(self.throttle_axis, self.steering_axis, 
                          self.head_yaw_axis, self.head_pitch_axis) + 1
        if len(msg.axes) < required_axes:
            self.get_logger().warn(
                f'Not enough axes in Joy message. Got {len(msg.axes)}, '
                f'need at least {required_axes}'
            )
            return
        
        # Get drive axis values
        throttle = msg.axes[self.throttle_axis]  # Forward/backward
        steering = msg.axes[self.steering_axis]   # Left/right turn
        
        # Apply inversions if configured
        if self.invert_throttle:
            throttle = -throttle
        if self.invert_steering:
            steering = -steering
        
        # Apply deadzone
        throttle = self.apply_deadzone(throttle)
        steering = self.apply_deadzone(steering)
        
        # Arcade drive: mix throttle and steering to get left and right motor values
        # throttle: positive = forward, negative = backward
        # steering: positive = turn right, negative = turn left
        left_value = throttle + steering
        right_value = throttle - steering
        
        # Clamp to [-1.0, 1.0] range
        left_value = max(-1.0, min(1.0, left_value))
        right_value = max(-1.0, min(1.0, right_value))
        
        # Convert to PWM values
        left_pwm = int(left_value * self.max_pwm)
        right_pwm = int(right_value * self.max_pwm)
        
        # Get head servo axis values
        head_yaw = msg.axes[self.head_yaw_axis]    # Left/right head movement
        head_pitch = msg.axes[self.head_pitch_axis]  # Up/down head movement
        
        # Apply inversions if configured
        if self.invert_head_yaw:
            head_yaw = -head_yaw
        if self.invert_head_pitch:
            head_pitch = -head_pitch
        
        # Apply deadzone
        head_yaw = self.apply_deadzone(head_yaw)
        head_pitch = self.apply_deadzone(head_pitch)
        
        # Map joystick values (-1 to 1) to angles (-90 to 90 degrees), then to radians
        head_yaw_deg = head_yaw * 90.0  # -90 to 90 degrees
        head_pitch_deg = head_pitch * 90.0  # -90 to 90 degrees
        
        head_yaw_rad = math.radians(head_yaw_deg)
        head_pitch_rad = math.radians(head_pitch_deg)
        
        # Get eye trigger value
        # Triggers typically range from -1.0 (not pressed) to 1.0 (fully pressed)
        # or sometimes 1.0 (not pressed) to -1.0 (fully pressed) depending on the controller
        eye_trigger = msg.axes[self.eye_trigger_axis] if len(msg.axes) > self.eye_trigger_axis else 1.0
        
        # Normalize trigger value to 0.0-1.0 range
        # If trigger goes from 1 to -1, map it: (1 = 0%, -1 = 100%)
        # Invert the mapping: not pressed (1.0) -> 0, fully pressed (-1.0) -> 1
        trigger_normalized = (1.0 - eye_trigger) / 2.0
        trigger_normalized = max(0.0, min(1.0, trigger_normalized))
        
        # Map trigger to eye PWM: 1 (idle) to 255 (fully pressed)
        # When trigger is at 0 (not pressed): eye_pwm = 1
        # When trigger is at 1 (fully pressed): eye_pwm = 255
        eye_pwm = int(1 + trigger_normalized * 254)  # 1 + (0 to 254)
        
        # Create and populate RobotLowCmd message
        cmd = RobotLowCmd()
        
        # Set eye PWM based on trigger (both eyes controlled together)
        cmd.eye_pwm = [eye_pwm, eye_pwm]
        
        # Set driver PWM values [left, right]
        cmd.driver_pwm = [left_pwm, right_pwm]
        
        # Initialize servo commands (15 servos, all disabled by default)
        cmd.servo_cmd = [ServoCmd() for _ in range(15)]
        for servo in cmd.servo_cmd:
            servo.enabled = False
            servo.acceleration = 0.0
            servo.target_location = 0.0
        
        # Configure head yaw servo (servo 14, index 13)
        cmd.servo_cmd[12].enabled = True
        cmd.servo_cmd[12].target_location = head_yaw_rad
        cmd.servo_cmd[12].acceleration = 0.0  # Set to desired acceleration if needed
        
        # Configure head pitch servo (servo 15, index 14)
        cmd.servo_cmd[13].enabled = True
        cmd.servo_cmd[13].target_location = head_pitch_rad
        cmd.servo_cmd[13].acceleration = 0.0  # Set to desired acceleration if needed

        # Publish the command
        self.cmd_pub.publish(cmd)
        
        # Log at debug level (only shown with --ros-args --log-level debug)
        self.get_logger().debug(
            f'Drive: Throttle={throttle:.2f} Steering={steering:.2f} -> PWM: L={left_pwm} R={right_pwm} | '
            f'Head: Yaw={head_yaw_deg:.1f}° Pitch={head_pitch_deg:.1f}° | Eye: {eye_pwm}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = JoyDriveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
