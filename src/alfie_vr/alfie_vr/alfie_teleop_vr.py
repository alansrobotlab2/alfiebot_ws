#!/usr/bin/env python3
"""
VR Head Tracker - ROS2 node that publishes head tracking data from VR headset
Uses the VRMonitor class for VR communication.
"""

# Standard library imports
import asyncio
import math
import threading
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from alfie_msgs.msg import RobotLowCmd, ServoCmd
from geometry_msgs.msg import Twist

# Local imports
from alfie_vr.vr_monitor import VRMonitor


class HeadTrackerNode(Node):
    """ROS2 node that publishes head tracking data from VR headset"""
    
    def __init__(self):
        super().__init__('head_tracker')
        
        # Create publisher for robot low-level commands
        self.cmd_publisher = self.create_publisher(
            RobotLowCmd,
            '/alfie/robotlowcmd',
            10
        )
        
        # Create timer for 100Hz publishing
        self.timer = self.create_timer(0.01, self.publish_robotlowcmd)  # 100Hz = 0.01s period
        
        # Create VR monitor
        self.vr_monitor = VRMonitor()
        
        # Thread for running VR monitor async loop
        self.vr_thread = None
        self.vr_loop = None
        
        # Track headset connection status
        self.headset_connected = False
        
        self.get_logger().info('Head Tracker Node initialized')
        
        # Start VR monitor in separate thread
        self.start_vr_monitor()
    
    def start_vr_monitor(self):
        """Start VR monitor in a separate thread"""
        # Initialize VR monitor
        self.get_logger().info('Initializing VR monitor...')
        if not self.vr_monitor.initialize():
            self.get_logger().error('VR monitor initialization failed')
            return
        
        def run_vr_loop():
            self.vr_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.vr_loop)
            try:
                self.vr_loop.run_until_complete(self.vr_monitor.start_monitoring())
            except Exception as e:
                self.get_logger().error(f'VR monitor error: {e}')
        
        self.vr_thread = threading.Thread(target=run_vr_loop, daemon=True)
        self.vr_thread.start()
        self.get_logger().info('VR monitor started in background thread')
    
    def publish_robotlowcmd(self):
        """Publish commands at 100Hz only when headset data is available"""
        # Get headset and controller goals from VR monitor
        dual_goals = self.vr_monitor.get_latest_goal_nowait()
        headset_goal = dual_goals.get("headset") if dual_goals else None
        left_controller_goal = dual_goals.get("left") if dual_goals else None
        right_controller_goal = dual_goals.get("right") if dual_goals else None
        
        # Track connection status changes
        if headset_goal is None:
            if self.headset_connected:
                self.headset_connected = False
                self.get_logger().info('Headset disconnected - pausing publishing')
            return
        
        # Log when headset connects
        if not self.headset_connected:
            self.headset_connected = True
            self.get_logger().info('Headset connected - publishing at 100Hz')
        
        # Create RobotLowCmd message
        cmd = RobotLowCmd()
        
        # Initialize all servo commands (15 servos)
        cmd.servo_cmd = [ServoCmd() for _ in range(15)]
        
        # Set all servos to disabled and zero by default
        for i in range(15):
            cmd.servo_cmd[i].enabled = False
            cmd.servo_cmd[i].target_location = 0.0
            cmd.servo_cmd[i].target_speed = 0.0
            cmd.servo_cmd[i].target_acceleration = 0.0
            cmd.servo_cmd[i].target_torque = 0.0
        
        # Extract rotation angles from headset goal metadata
        # The VR system sends rotation as {'x': pitch, 'y': yaw, 'z': roll}
        if hasattr(headset_goal, 'metadata') and headset_goal.metadata:
            rotation = headset_goal.metadata.get('rotation', {})
            
            if rotation:
                # Servo 12: Head Pan (yaw - rotation around Z-axis)
                # VR sends this as 'y' key
                if 'y' in rotation:
                    yaw_deg = float(rotation['y'])
                    yaw_rad = math.radians(yaw_deg)
                    cmd.servo_cmd[12].enabled = True
                    cmd.servo_cmd[12].target_location = yaw_rad
                
                # Servo 13: Head Tilt (pitch - rotation around Y-axis)
                # VR sends this as 'x' key
                if 'x' in rotation:
                    pitch_deg = float(rotation['x'])
                    pitch_rad = math.radians(pitch_deg)
                    cmd.servo_cmd[13].enabled = True
                    cmd.servo_cmd[13].target_location = pitch_rad
                
                # Servo 14: Head Roll (roll - rotation around X-axis)
                # VR sends this as 'z' key
                if 'z' in rotation:
                    roll_deg = float(rotation['z'])
                    roll_rad = math.radians(roll_deg)
                    cmd.servo_cmd[14].enabled = True
                    cmd.servo_cmd[14].target_location = roll_rad
        
        # Initialize other fields
        cmd.eye_pwm = [0, 0]
        cmd.shoulder_height = 0.0
        cmd.cmd_vel = Twist()
        
        # Process joystick input for cmd_vel
        # Velocity limits
        MAX_LINEAR_VEL = 0.5  # m/s
        MAX_ANGULAR_VEL = 0.5  # rad/s
        
        # Debug: Log controller goal availability
        # if left_controller_goal:
        #     self.get_logger().info(f'Left controller metadata: {left_controller_goal.metadata if hasattr(left_controller_goal, "metadata") else "No metadata"}', throttle_duration_sec=1.0)
        # if right_controller_goal:
        #     self.get_logger().info(f'Right controller metadata: {right_controller_goal.metadata if hasattr(right_controller_goal, "metadata") else "No metadata"}', throttle_duration_sec=1.0)
        
        # Left joystick controls linear velocity (forward/back and strafe left/right)
        if left_controller_goal and hasattr(left_controller_goal, 'metadata') and left_controller_goal.metadata:
            thumbstick = left_controller_goal.metadata.get('thumbstick', {})
            if thumbstick:
                # X-axis: strafe (positive = right, negative = left)
                # Y-axis: forward/back (positive = forward, negative = back)
                joy_x = float(thumbstick.get('x', 0.0))
                joy_y = float(thumbstick.get('y', 0.0))
                
                # Map joystick values (-1 to 1) to velocity limits
                cmd.cmd_vel.linear.x = -joy_y * MAX_LINEAR_VEL  # forward/back (negated)
                cmd.cmd_vel.linear.y = joy_x * MAX_LINEAR_VEL  # strafe left/right
                
                # Debug log
                if abs(joy_x) > 0.1 or abs(joy_y) > 0.1:
                    self.get_logger().info(f'Left thumbstick: x={joy_x:.2f}, y={joy_y:.2f} -> linear.x={cmd.cmd_vel.linear.x:.2f}, linear.y={cmd.cmd_vel.linear.y:.2f}')
        
        # Right joystick X-axis controls angular velocity (rotate left/right)
        if right_controller_goal and hasattr(right_controller_goal, 'metadata') and right_controller_goal.metadata:
            thumbstick = right_controller_goal.metadata.get('thumbstick', {})
            if thumbstick:
                # X-axis: rotation (positive = rotate right, negative = rotate left)
                joy_x = float(thumbstick.get('x', 0.0))
                
                # Map joystick value (-1 to 1) to angular velocity limit
                cmd.cmd_vel.angular.z = joy_x * MAX_ANGULAR_VEL  # Rotation (removed negation)
                
                # Debug log
                if abs(joy_x) > 0.1:
                    self.get_logger().info(f'Right thumbstick: x={joy_x:.2f} -> angular.z={cmd.cmd_vel.angular.z:.2f}')
        
        # Publish the command
        self.cmd_publisher.publish(cmd)
    
    def shutdown(self):
        """Shutdown the node and VR monitor"""
        self.get_logger().info('Shutting down head tracker...')
        if self.vr_loop:
            self.vr_loop.call_soon_threadsafe(self.vr_loop.stop)
        if self.vr_thread:
            self.vr_thread.join(timeout=2.0)


def main():
    """Main function"""
    print("🎮 Alfie Teleop- VR Headset to Robot Control")
    print("=" * 60)
    
    # Initialize ROS2
    rclpy.init()
    
    # Create node
    node = HeadTrackerNode()
    
    try:
        # Spin the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Teleop stopped by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()