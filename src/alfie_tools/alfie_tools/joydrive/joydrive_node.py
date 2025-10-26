#!/usr/bin/env python3
"""
Joydrive Node - Mecanum drive control using a USB joystick

This node subscribes to /joy topic and publishes RobotLowCmd messages
to control the robot using mecanum drive controls.
- Left stick Y-axis: forward/backward (linear.x)
- Left stick X-axis: strafe left/right (linear.y)
- Right stick X-axis: rotation (angular.z)
- Right stick Y-axis: head pitch (servo 14, up/down)
- Right trigger: eye brightness (1-255 based on trigger depression)
- Left trigger: head roll (servo 15, -π/4 to π/4 radians)

Twist velocities: linear.x, linear.y, angular.z (in m/s and rad/s)
Head servos range from -90 to 90 degrees (converted to radians).
Eye PWM ranges from 1 (idle) to 255 (full brightness when trigger pressed).
Head roll ranges from -π/4 to π/4 radians (0 when trigger not pressed).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from alfie_msgs.msg import RobotLowCmd, ServoCmd
import math


class JoyDriveNode(Node):
    def __init__(self):
        super().__init__('joydrive_node')
        
        # Declare parameters for mecanum drive control
        self.declare_parameter('linear_x_axis', 1)  # Y-axis for forward/backward (axis 1 = left stick Y)
        self.declare_parameter('linear_y_axis', 0)  # X-axis for strafe left/right (axis 0 = left stick X)
        self.declare_parameter('angular_z_axis', 3)  # Right stick X-axis for rotation
        self.declare_parameter('head_pitch_axis', 4)  # Right stick Y-axis for head pitch
        self.declare_parameter('head_roll_axis', 2)  # Left trigger for head roll (typical axis 2 = LT)
        self.declare_parameter('eye_trigger_axis', 5)  # Right trigger for eye brightness (typical axis 5 = RT)
        self.declare_parameter('max_linear_velocity', 1.0)  # Max linear velocity in m/s
        self.declare_parameter('max_angular_velocity', 3.14)  # Max angular velocity in rad/s
        self.declare_parameter('deadzone', 0.1)  # Deadzone to ignore small stick movements
        self.declare_parameter('invert_linear_x', False)
        self.declare_parameter('invert_linear_y', False)
        self.declare_parameter('invert_angular_z', False)
        self.declare_parameter('invert_head_pitch', False)
        
        # Get parameters
        self.linear_x_axis = self.get_parameter('linear_x_axis').value
        self.linear_y_axis = self.get_parameter('linear_y_axis').value
        self.angular_z_axis = self.get_parameter('angular_z_axis').value
        self.head_pitch_axis = self.get_parameter('head_pitch_axis').value
        self.head_roll_axis = self.get_parameter('head_roll_axis').value
        self.eye_trigger_axis = self.get_parameter('eye_trigger_axis').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.deadzone = self.get_parameter('deadzone').value
        self.invert_linear_x = self.get_parameter('invert_linear_x').value
        self.invert_linear_y = self.get_parameter('invert_linear_y').value
        self.invert_angular_z = self.get_parameter('invert_angular_z').value
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
        
        self.get_logger().info('Joydrive node started (Mecanum drive control + Head servos + Eye brightness + Head roll)')
        self.get_logger().info(f'Drive - Linear X axis: {self.linear_x_axis}, Linear Y axis: {self.linear_y_axis}, Angular Z axis: {self.angular_z_axis}')
        self.get_logger().info(f'Head - Pitch axis: {self.head_pitch_axis}, Roll axis: {self.head_roll_axis}')
        self.get_logger().info(f'Eye - Trigger axis: {self.eye_trigger_axis}')
        self.get_logger().info(f'Max linear velocity: {self.max_linear_velocity} m/s, Max angular velocity: {self.max_angular_velocity} rad/s')
        self.get_logger().info(f'Deadzone: {self.deadzone}')
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def map_axis_to_velocity(self, axis_value, max_velocity):
        """
        Map joystick axis value (-1.0 to 1.0) to velocity value
        
        Args:
            axis_value: Joystick axis value from -1.0 to 1.0
            max_velocity: Maximum velocity (linear or angular)
        
        Returns:
            Velocity value scaled by max_velocity
        """
        # Apply deadzone
        axis_value = self.apply_deadzone(axis_value)
        
        # Map from [-1, 1] to [-max_velocity, max_velocity]
        velocity = axis_value * max_velocity
        
        # Clamp to valid range
        velocity = max(-max_velocity, min(max_velocity, velocity))
        
        return velocity
    
    def joy_callback(self, msg):
        """Process joystick input and publish drive commands using mecanum drive control"""
        # Check if we have enough axes
        required_axes = max(self.linear_x_axis, self.linear_y_axis, 
                          self.angular_z_axis, self.head_pitch_axis) + 1
        if len(msg.axes) < required_axes:
            self.get_logger().warn(
                f'Not enough axes in Joy message. Got {len(msg.axes)}, '
                f'need at least {required_axes}'
            )
            return
        
        # Get drive axis values
        linear_x = msg.axes[self.linear_x_axis]  # Forward/backward
        linear_y = msg.axes[self.linear_y_axis]  # Strafe left/right
        angular_z = msg.axes[self.angular_z_axis]  # Rotation
        
        # Apply inversions if configured
        if self.invert_linear_x:
            linear_x = -linear_x
        if self.invert_linear_y:
            linear_y = -linear_y
        if self.invert_angular_z:
            angular_z = -angular_z
        
        # Map axis values to velocities
        linear_x_vel = self.map_axis_to_velocity(linear_x, self.max_linear_velocity)
        linear_y_vel = self.map_axis_to_velocity(linear_y, self.max_linear_velocity)
        angular_z_vel = self.map_axis_to_velocity(angular_z, self.max_angular_velocity)
        
        # Get head pitch axis value
        head_pitch = msg.axes[self.head_pitch_axis]  # Up/down head movement
        
        # Apply inversion if configured
        if self.invert_head_pitch:
            head_pitch = -head_pitch
        
        # Apply deadzone
        head_pitch = self.apply_deadzone(head_pitch)
        
        # Map joystick value (-1 to 1) to angle (-90 to 90 degrees), then to radians
        head_pitch_deg = head_pitch * 90.0  # -90 to 90 degrees
        head_pitch_rad = math.radians(head_pitch_deg)
        
        # Get head roll trigger value (left trigger)
        # Trigger default value is 1.0 (not pressed)
        # When trigger is pressed, value goes from 1.0 to -1.0 (or 0.0 depending on controller)
        head_roll_trigger = msg.axes[self.head_roll_axis] if len(msg.axes) > self.head_roll_axis else 1.0
        
        # Map head roll: when trigger = 1.0 (not pressed) -> 0.0 radians
        # When trigger is pressed (1.0 to -1.0), map to -π/4 to π/4
        if abs(head_roll_trigger - 1.0) < 0.01:  # Trigger not pressed (at default value 1.0)
            head_roll_rad = 0.0
        else:
            # Map trigger value from [1.0, -1.0] to [-π/4, π/4]
            # When trigger = 1.0 -> 0 radians
            # When trigger = -1.0 -> π/4 radians (or -π/4, depending on desired direction)
            # Normalize from [1.0, -1.0] to [0.0, 1.0]
            roll_normalized = (1.0 - head_roll_trigger) / 2.0
            roll_normalized = max(0.0, min(1.0, roll_normalized))
            # Map to [-π/4, π/4] range, centered at 0
            # 0 -> -π/4, 0.5 -> 0, 1 -> π/4
            head_roll_rad = (roll_normalized - 0.5) * math.pi / 2.0
        
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
        
        # Create and set Twist message for mecanum drive
        cmd.cmd_vel = Twist()
        cmd.cmd_vel.linear.x = linear_x_vel
        cmd.cmd_vel.linear.y = linear_y_vel
        cmd.cmd_vel.angular.z = angular_z_vel
        
        # Initialize servo commands (15 servos, all disabled by default)
        cmd.servo_cmd = [ServoCmd() for _ in range(15)]
        for servo in cmd.servo_cmd:
            servo.enabled = False
            servo.acceleration = 0.0
            servo.target_location = 0.0
        
        # Configure head pitch servo (servo 14, index 13)
        cmd.servo_cmd[13].enabled = True
        cmd.servo_cmd[13].target_location = head_pitch_rad
        cmd.servo_cmd[13].acceleration = 0.0  # Set to desired acceleration if needed

        # Configure head roll servo (servo 15, index 14)
        cmd.servo_cmd[14].enabled = True
        cmd.servo_cmd[14].target_location = head_roll_rad
        cmd.servo_cmd[14].acceleration = 0.0  # Set to desired acceleration if needed


        # Publish the command
        self.cmd_pub.publish(cmd)
        
        # Log at debug level (only shown with --ros-args --log-level debug)
        self.get_logger().debug(
            f'Drive: X={linear_x_vel:.2f} Y={linear_y_vel:.2f} AngZ={angular_z_vel:.2f} | '
            f'Head: Pitch={head_pitch_deg:.1f}° Roll={math.degrees(head_roll_rad):.1f}° | Eye: {eye_pwm}'
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
