import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from alfie_msgs.msg import RobotLowState, RobotLowCmd
from alfie_msgs.msg import DriverState, DriverCmd
from alfie_msgs.msg import ServoCmd
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
import os
from ament_index_python.packages import get_package_share_directory



class MasterTopicsNode(Node):
    def __init__(self):
        super().__init__('master_topics_node')
        # Use BEST_EFFORT QoS for driver state subscriptions (to match driver publishers)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Use RELIABLE QoS for command and robot state topics
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Initialize state variables
        self.driver0_state = None
        self.driver1_state = None
        
        # Initialize command variables and timestamp
        self.driver0_cmd = None
        self.driver1_cmd = None
        self.last_robot_cmd_time = None
        self.cmd_timeout = 0.1  # 100 milliseconds
        self.last_warn_time = None  # For throttling warnings
        
        # Subscribe to driver states (use BEST_EFFORT to match driver publishers)
        self.driver0_sub = self.create_subscription(
            DriverState,
            'driver0state',
            self.driver0_callback,
            qos_best_effort
        )
        
        self.driver1_sub = self.create_subscription(
            DriverState,
            'driver1state',
            self.driver1_callback,
            qos_best_effort
        )
        
        # Publisher for robot low state (use RELIABLE for subscribers)
        self.robot_state_pub = self.create_publisher(
            RobotLowState,
            'robotlowstate',
            qos_reliable
        )
        
        # Subscribe to robot low command (use RELIABLE)
        self.robot_cmd_sub = self.create_subscription(
            RobotLowCmd,
            'robotlowcmd',
            self.robot_cmd_callback,
            qos_reliable
        )
        
        # Publishers for driver commands (use RELIABLE)
        self.driver0_cmd_pub = self.create_publisher(
            DriverCmd,
            'driver0cmd',
            qos_reliable
        )
        
        self.driver1_cmd_pub = self.create_publisher(
            DriverCmd,
            'driver1cmd',
            qos_reliable
        )
        
        # Create timers for 100Hz publishing (0.01 seconds = 10ms)
        self.state_timer = self.create_timer(0.01, self.publish_robot_state)
        # stagger the timers by 50ms to avoid congestion
        time.sleep(0.05)
        self.cmd_timer = self.create_timer(0.01, self.publish_driver_commands)
        
        self.get_logger().info('Master Topics Node started - publishing at 100Hz')
    
    def driver0_callback(self, msg):
        """Callback for driver0state"""
        self.driver0_state = msg
    
    def driver1_callback(self, msg):
        """Callback for driver1state"""
        self.driver1_state = msg
    
    def robot_cmd_callback(self, msg):
        """Decompose RobotLowCmd into DriverCmd messages"""
        # Update timestamp
        self.last_robot_cmd_time = self.get_clock().now()
        
        # Create DriverCmd for driver0 (motors are on driver0)
        driver0_cmd = DriverCmd()
        driver0_cmd.driver_pwm = msg.driver_pwm  # 2 motors on driver0
        # Driver0 gets servos 8-17 (indices 7-16 in the array, which is 10 servos)
        driver0_cmd.servo_cmd = list(msg.servo_cmd[7:17])
        
        # Create DriverCmd for driver1 (eye_lights are on driver1)
        driver1_cmd = DriverCmd()
        driver1_cmd.driver_pwm = msg.eye_pwm  # 2 eye lights on driver1
        # Driver1 gets servos 1-7 (indices 0-6, which is 7 servos)
        # Need to fill remaining 3 servo slots with defaults to make 10 total
        servo_list = list(msg.servo_cmd[0:7])
        # Pad with 3 default ServoCmd for the remaining servos
        for _ in range(3):
            servo_list.append(ServoCmd())
        driver1_cmd.servo_cmd = servo_list
        
        # Store the commands (they will be published by the timer)
        self.driver0_cmd = driver0_cmd
        self.driver1_cmd = driver1_cmd
        self.get_logger().debug('Received robot command')
    
    def publish_driver_commands(self):
        """Publish driver commands at 100Hz with failsafe timeout"""
        # Check if we have commands and they're recent enough
        if self.driver0_cmd is None or self.driver1_cmd is None:
            return
        
        if self.last_robot_cmd_time is None:
            return
        
        # Check if the last command is within the timeout period (100ms)
        time_since_last_cmd = (self.get_clock().now() - self.last_robot_cmd_time).nanoseconds / 1e9
        
        if time_since_last_cmd > self.cmd_timeout:
            # Throttle warning to once per second
            current_time = self.get_clock().now()
            if self.last_warn_time is None or (current_time - self.last_warn_time).nanoseconds / 1e9 >= 1.0:
                self.get_logger().warn(
                    f'No robot command received for {time_since_last_cmd:.3f}s - NOT publishing driver commands')
                self.last_warn_time = current_time
            return
        
        # Publish the commands
        self.driver0_cmd_pub.publish(self.driver0_cmd)
        self.driver1_cmd_pub.publish(self.driver1_cmd)
        self.get_logger().debug('Published driver commands')
    
    def publish_robot_state(self):
        """Combine driver states and publish robot state"""
        # Only publish if we have data from both drivers
        if self.driver0_state is None or self.driver1_state is None:
            return
        
        robot_state = RobotLowState()
        
        # Populate header
        robot_state.header.stamp = self.get_clock().now().to_msg()
        robot_state.header.frame_id = 'base_link'
        
        # Combine board temperatures
        robot_state.board_temp = [
            self.driver0_state.board_temp,
            self.driver1_state.board_temp
        ]
        
        # Combine IMU data
        robot_state.imu = [
            self.driver0_state.imu,
            self.driver1_state.imu
        ]
        
        # Combine magnetic field data
        robot_state.magnetic_field = [
            self.driver0_state.magnetic_field,
            self.driver1_state.magnetic_field
        ]

        # Use shoulder limit state from driver1 (or combine logic as needed)
        robot_state.shoulder_limit_state = self.driver1_state.shoulder_limit_state
        
        # Combine driver diagnostics
        robot_state.driver_diagnostics = [
            self.driver0_state.driver_diagnostics,
            self.driver1_state.driver_diagnostics
        ]
        
        # Combine motor states (2 motors per driver = 4 total, but RobotLowState expects 2)
        # Assuming we take motors from both drivers
        robot_state.eye_state = self.driver1_state.motor_state  # 2 motors from driver1 for eyes
        robot_state.motor_state = self.driver0_state.motor_state  # 2 motors from driver0
        
        # Combine servo states (10 servos per driver = 20 total, but RobotLowState expects 17)
        # You may need to adjust this logic based on your robot's configuration
        robot_state.servo_state = (
            list(self.driver1_state.servo_state[:7])   # Take first 7 from driver1 to make 17 total
            + list(self.driver0_state.servo_state)
        )
        
        self.robot_state_pub.publish(robot_state)
        self.get_logger().debug('Published robot state')
       

def main(args=None):
    rclpy.init(args=args)
    node = MasterTopicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
