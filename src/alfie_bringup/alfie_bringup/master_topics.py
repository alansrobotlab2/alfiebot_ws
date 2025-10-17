import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from alfie_msgs.msg import RobotLowState, RobotLowCmd
from alfie_msgs.msg import GDBState, GDBCmd
from alfie_msgs.msg import GDBServoCmd, ServoState
from alfie_msgs.msg import MotorState
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
import os
from ament_index_python.packages import get_package_share_directory



class MasterTopicsNode(Node):
    def __init__(self):
        super().__init__('master_topics_node')
        # Use BEST_EFFORT QoS for gdb state subscriptions (to match gdb publishers)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Use RELIABLE QoS for command and robot state topics
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Initialize state variables
        self.gdb0_state = None
        self.gdb1_state = None
        self.oak_imu = None
        
        # Initialize command variables and timestamp
        self.gdb0_cmd = None
        self.gdb1_cmd = None
        self.last_robot_cmd_time = None
        self.cmd_timeout = 0.1  # 100 milliseconds
        self.last_warn_time = None  # For throttling warnings
        
        # Subscribe to gdb states (use BEST_EFFORT to match gdb publishers)
        self.gdb0_sub = self.create_subscription(
            GDBState,
            'gdb0state',
            self.gdb0_callback,
            qos_best_effort
        )
        
        self.gdb1_sub = self.create_subscription(
            GDBState,
            'gdb1state',
            self.gdb1_callback,
            qos_best_effort
        )
        
        # Subscribe to Oak IMU data (use BEST_EFFORT to match typical IMU publishers)
        self.oak_imu_sub = self.create_subscription(
            Imu,
            'oak/imu/data',
            self.oak_imu_callback,
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
        
        # Publishers for gdb commands (use RELIABLE)
        self.gdb0_cmd_pub = self.create_publisher(
            GDBCmd,
            'gdb0cmd',
            qos_reliable
        )
        
        self.gdb1_cmd_pub = self.create_publisher(
            GDBCmd,
            'gdb1cmd',
            qos_reliable
        )
        
        # Create timers for 100Hz publishing (0.01 seconds = 10ms)
        self.state_timer = self.create_timer(0.01, self.publish_robot_state)
        # stagger the timers by 50ms to avoid congestion
        time.sleep(0.05)
        self.cmd_timer = self.create_timer(0.01, self.publish_gdb_commands)
        
        self.get_logger().info('Master Topics Node started - publishing at 100Hz')
    
    def gdb0_callback(self, msg):
        """Callback for gdb0state"""
        self.gdb0_state = msg
    
    def gdb1_callback(self, msg):
        """Callback for gdb1state"""
        self.gdb1_state = msg
    
    def oak_imu_callback(self, msg):
        """Callback for Oak IMU data"""
        self.oak_imu = msg
    
    def robot_cmd_callback(self, msg):
        """Decompose RobotLowCmd into GDBCmd messages"""
        # Update timestamp
        self.last_robot_cmd_time = self.get_clock().now()
        
        # Create GDBCmd for gdb0 (motors are on gdb0)
        gdb0_cmd = GDBCmd()
        gdb0_cmd.driver_pwm = msg.driver_pwm  # 2 motors on gdb0
        # gdb0 gets servos 8-17 (indices 7-16 in the array, which is 10 servos)
        gdb0_cmd.servo_cmd = list(msg.servo_cmd[7:17])
        
        # Create GDBCmd for gdb1 (eye_lights are on gdb1)
        gdb1_cmd = GDBCmd()
        gdb1_cmd.driver_pwm = msg.eye_pwm  # 2 eye lights on gdb1
        # gdb1 gets servos 1-7 (indices 0-6, which is 7 servos)
        # Need to fill remaining 3 servo slots with defaults to make 10 total
        servo_list = list(msg.servo_cmd[0:7])
        # Pad with 3 default GDBServoCmd for the remaining servos
        for _ in range(3):
            servo_list.append(GDBServoCmd())
        gdb1_cmd.servo_cmd = servo_list
        
        # Store the commands (they will be published by the timer)
        self.gdb0_cmd = gdb0_cmd
        self.gdb1_cmd = gdb1_cmd
        self.get_logger().debug('Received robot command')
    
    def publish_gdb_commands(self):
        """Publish gdb commands at 100Hz with failsafe timeout"""
        # Check if we have commands and they're recent enough
        if self.gdb0_cmd is None or self.gdb1_cmd is None:
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
                    f'No robot command received for {time_since_last_cmd:.3f}s - NOT publishing gdb commands')
                self.last_warn_time = current_time
            return
        
        # Publish the commands
        self.gdb0_cmd_pub.publish(self.gdb0_cmd)
        self.gdb1_cmd_pub.publish(self.gdb1_cmd)
        self.get_logger().debug('Published gdb commands')
    
    def publish_robot_state(self):
        """Combine gdb states and publish robot state"""
        # Only publish if we have data from both gdbs
        if self.gdb0_state is None or self.gdb1_state is None:
            self.get_logger().debug('Waiting for both gdb states...')
            return
        
        robot_state = RobotLowState()
        
        # Populate header
        robot_state.header.stamp = self.get_clock().now().to_msg()
        robot_state.header.frame_id = 'base_link'
        
        # Populate IMU data (3 total: Oak IMU in head, upper gdb board in base, lower gdb board in base)
        # Order: Oak IMU (head), gdb0 IMU (upper board), gdb1 IMU (lower board)
        
        # If Oak IMU is available, use it; otherwise create an empty IMU message
        if self.oak_imu is not None:
            # Copy the Oak IMU and update the frame_id to be consistent
            oak_imu_copy = Imu()
            oak_imu_copy.header.stamp = self.oak_imu.header.stamp
            oak_imu_copy.header.frame_id = 'oak_imu_frame'
            oak_imu_copy.orientation = self.oak_imu.orientation
            oak_imu_copy.orientation_covariance = self.oak_imu.orientation_covariance
            oak_imu_copy.angular_velocity = self.oak_imu.angular_velocity
            oak_imu_copy.angular_velocity_covariance = self.oak_imu.angular_velocity_covariance
            oak_imu_copy.linear_acceleration = self.oak_imu.linear_acceleration
            oak_imu_copy.linear_acceleration_covariance = self.oak_imu.linear_acceleration_covariance
            robot_state.imu = [
                oak_imu_copy,  # Oak IMU (head/camera)
                self.convert_gdb_imu_to_imu(self.gdb0_state.imu, 'gdb0_imu'),  # Upper GDB board
                self.convert_gdb_imu_to_imu(self.gdb1_state.imu, 'gdb1_imu')   # Lower GDB board
            ]
        else:
            # Oak IMU not available yet, create placeholder
            oak_imu_placeholder = Imu()
            oak_imu_placeholder.header.stamp = robot_state.header.stamp
            oak_imu_placeholder.header.frame_id = 'oak_imu_frame'
            robot_state.imu = [
                oak_imu_placeholder,  # Oak IMU placeholder
                self.convert_gdb_imu_to_imu(self.gdb0_state.imu, 'gdb0_imu'),  # Upper GDB board
                self.convert_gdb_imu_to_imu(self.gdb1_state.imu, 'gdb1_imu')   # Lower GDB board
            ]
        
        # Populate magnetic field data (2 total: upper gdb board, lower gdb board)
        robot_state.magnetic_field = [
            self.convert_gdb_magnetic_field_to_magnetic_field(self.gdb0_state.magnetic_field, 'gdb0_mag'),
            self.convert_gdb_magnetic_field_to_magnetic_field(self.gdb1_state.magnetic_field, 'gdb1_mag')
        ]

        # Shoulder limit state (from gdb1)
        robot_state.shoulder_limit_state = self.gdb1_state.shoulder_limit_state
        
        # Eye state (2 PWM values from gdb1 motor states)
        # Convert motor PWM commands to uint8 (0-255)
        robot_state.eye_state = [
            self.convert_pwm_to_uint8(self.gdb1_state.motor_state[0].pwm_cmd),
            self.convert_pwm_to_uint8(self.gdb1_state.motor_state[1].pwm_cmd)
        ]
        
        # Motor states (2 driver motors from gdb0)
        # Convert from GDBMotorState to MotorState
        robot_state.motor_state = [
            self.convert_gdb_motor_to_motor(self.gdb0_state.motor_state[0]),
            self.convert_gdb_motor_to_motor(self.gdb0_state.motor_state[1])
        ]
        
        # Servo states (17 total: 0-6 left arm, 7-13 right arm, 14-16 head)
        # gdb1 has servos 0-6 (left arm - first 7 servos)
        # gdb0 has servos 7-16 (right arm 7-13, head 14-16 - 10 servos)
        # Build complete list first, then assign all at once
        servo_states = []
        for i in range(7):
            servo_states.append(
                self.convert_gdb_servo_to_servo(self.gdb1_state.servo_state[i])
            )
        for i in range(10):
            servo_states.append(
                self.convert_gdb_servo_to_servo(self.gdb0_state.servo_state[i])
            )
        robot_state.servo_state = servo_states
        
        self.robot_state_pub.publish(robot_state)
        # self.get_logger().debug('Published robot state')
    
    def convert_gdb_imu_to_imu(self, gdb_imu, frame_id):
        """Convert GDBImu to sensor_msgs/Imu"""
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = frame_id
        
        # Populate orientation (quaternion)
        imu.orientation.x = gdb_imu.orientation_x
        imu.orientation.y = gdb_imu.orientation_y
        imu.orientation.z = gdb_imu.orientation_z
        imu.orientation.w = gdb_imu.orientation_w
        
        # Populate angular velocity (rad/s)
        imu.angular_velocity.x = gdb_imu.angular_velocity_x
        imu.angular_velocity.y = gdb_imu.angular_velocity_y
        imu.angular_velocity.z = gdb_imu.angular_velocity_z
        
        # Populate linear acceleration (m/s^2)
        imu.linear_acceleration.x = gdb_imu.linear_acceleration_x
        imu.linear_acceleration.y = gdb_imu.linear_acceleration_y
        imu.linear_acceleration.z = gdb_imu.linear_acceleration_z
        
        # Set covariance to zeros (unknown covariance)
        imu.orientation_covariance = [0.0] * 9
        imu.angular_velocity_covariance = [0.0] * 9
        imu.linear_acceleration_covariance = [0.0] * 9
        
        return imu
    
    def convert_gdb_magnetic_field_to_magnetic_field(self, gdb_mag, frame_id):
        """Convert GDBMagneticField to sensor_msgs/MagneticField"""
        mag = MagneticField()
        mag.header.stamp = self.get_clock().now().to_msg()
        mag.header.frame_id = frame_id
        
        # Populate magnetic field vector (Tesla)
        mag.magnetic_field.x = gdb_mag.magnetic_field_x
        mag.magnetic_field.y = gdb_mag.magnetic_field_y
        mag.magnetic_field.z = gdb_mag.magnetic_field_z
        
        # Set covariance to zeros (unknown covariance)
        mag.magnetic_field_covariance = [0.0] * 9
        
        return mag
    
    def convert_pwm_to_uint8(self, pwm_cmd):
        """Convert PWM command (-255 to 255) to uint8 (0 to 255)"""
        # Take absolute value and clamp to 0-255
        return min(255, max(0, abs(pwm_cmd)))
    
    def convert_gdb_motor_to_motor(self, gdb_motor):
        """Convert GDBMotorState to MotorState"""
        motor = MotorState()
        motor.is_moving = gdb_motor.is_moving
        motor.pwm_cmd = gdb_motor.pwm_cmd
        motor.velocity = gdb_motor.velocity
        motor.acceleration = gdb_motor.acceleration
        motor.pulse_count = gdb_motor.pulse_count
        
        return motor
    

    def convert_gdb_servo_to_servo(self, gdb_servo):
        """Convert GDBServoState to ServoState"""
        
        servo = ServoState()
        
        # enabled: true if torqueswitch is 1, false if 0
        
        # acceleration: convert from units to radians per second squared
        # acceleration: convert from units to radians per second squared (rad/s²)
        # Each unit = 100 steps/s², and 4096 steps = 2π radians
        servo.acceleration = gdb_servo.acceleration * 100.0 * (2.0 * np.pi / 4096.0)  # rad/s²
        # target_location and current_location: convert from 0-4096 to -π to +π radians
        servo.target_location = self.convert_servo_position_to_radians(gdb_servo.targetlocation)
        servo.current_location = self.convert_servo_position_to_radians(gdb_servo.currentlocation)
        
        # current_speed: convert from steps per second to radians per second
        # 4096 steps = 2π radians (one full rotation)
        servo.current_speed = gdb_servo.currentspeed * (2.0 * np.pi / 4096.0)
        
        # current_load: convert from 0.1% units to percentage
        servo.current_load = gdb_servo.currentload * 10.0
        
        servo.current_temperature = gdb_servo.currenttemperature
        servo.servo_status = gdb_servo.servostatus
        servo.is_moving = (gdb_servo.mobilesign != 0)
        
        # current_voltage: convert from 0.1V units to volts
        servo.current_voltage = gdb_servo.currentvoltage / 10.0
        
        # current_current: convert from 6.5mA units to milliamps
        servo.current_current = gdb_servo.currentcurrent * 6.5
        
        return servo
    
    def convert_servo_position_to_radians(self, raw_position):
        """Convert servo position from 0-4096 range to -π to +π radians"""
        # Map 0-4096 to -π to +π
        # First normalize to 0-1, then scale to -π to +π
        normalized = raw_position / 4096.0
        radians = (normalized * 2.0 * np.pi) - np.pi
        return radians
       

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
