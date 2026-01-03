import time
from typing import Dict, List, Optional
import rclpy
from rclpy.node import Node
from alfie_msgs.msg import RobotLowState, GDBState, BackState, JetsonState
from alfie_msgs.msg import ServoState, MotorState
from sensor_msgs.msg import Imu, MagneticField, JointState
from geometry_msgs.msg import Twist
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from .servo_config import SERVO_POLARITY
from .watchdog_checks import create_health_checks, HealthCheck


# ============================================================================
# Constants and Configuration
# ============================================================================

# Publishing configuration
PUBLISH_RATE_HZ = 100
PUBLISH_PERIOD_SEC = 1.0 / PUBLISH_RATE_HZ  # 0.01 seconds

# Watchdog check rate (slower than publish rate)
WATCHDOG_RATE_HZ = 1.0  # Check once per second
WATCHDOG_PERIOD_SEC = 1.0 / WATCHDOG_RATE_HZ

# Servo mapping
NUM_SERVOS_GDB1 = 7  # Left arm servos (0-6), but skipping index 2
NUM_SERVOS_GDB0 = 10  # Right arm + head servos (7-16), but skipping index 2
TOTAL_SERVOS = 15  # 6 from gdb1 + 9 from gdb0 (skipping 3rd servo from each)

# Unit conversion constants
SERVO_STEPS_PER_REVOLUTION = 4096
STEPS_TO_RADIANS = 2.0 * np.pi / SERVO_STEPS_PER_REVOLUTION
SERVO_ACCEL_UNITS_TO_STEPS_PER_SEC2 = 100.0

# Motor encoder constants
MOTOR_PULSES_PER_REVOLUTION = 22  # Encoder pulses per motor shaft revolution
MOTOR_PULSES_TO_RADIANS = 2.0 * np.pi / MOTOR_PULSES_PER_REVOLUTION

# Servo unit conversions
SERVO_LOAD_SCALE = 10.0  # 0.1% units to percentage
SERVO_VOLTAGE_SCALE = 10.0  # 0.1V units to volts
SERVO_CURRENT_SCALE = 6.5  # 6.5mA units to milliamps

# Joint names mapping (matching URDF joint names)
# Servo index to joint name mapping (indices 2 and 9 are derived servos - not published directly)
SERVO_JOINT_NAMES = [
    'left_shoulder_yaw_joint',      # Servo 0
    'left_shoulder_pitch_joint',    # Servo 1
    None,                           # Servo 2 (derived - left_shoulder2_pitch, mirrored from servo 1)
    'left_elbow_pitch_joint',       # Servo 3
    'left_wrist_pitch_joint',       # Servo 4
    'left_wrist_roll_joint',        # Servo 5
    'left_gripper_active_joint',    # Servo 6
    'right_shoulder_yaw_joint',     # Servo 7
    'right_shoulder_pitch_joint',   # Servo 8
    None,                           # Servo 9 (derived - right_shoulder2_pitch, mirrored from servo 8)
    'right_elbow_pitch_joint',      # Servo 10
    'right_wrist_pitch_joint',      # Servo 11
    'right_wrist_roll_joint',       # Servo 12
    'right_gripper_active_joint',   # Servo 13
    'head_yaw_joint',               # Servo 14
    'head_pitch_joint',             # Servo 15
    'head_roll_joint',              # Servo 16
]

# Motor joint names (mecanum wheels)
MOTOR_JOINT_NAMES = [
    'mecanum_fl_joint',  # Motor 0 - Front Left
    'mecanum_fr_joint',  # Motor 1 - Front Right
    'mecanum_rl_joint',  # Motor 2 - Rear Left
    'mecanum_rr_joint',  # Motor 3 - Rear Right
]

# Other joints from URDF
OTHER_JOINT_NAMES = [
    'back_joint',  # Prismatic joint for back lift
]


# ============================================================================
# MasterStatusNode Class
# ============================================================================

class MasterStatusNode(Node):
    def __init__(self):
        super().__init__('master_status_node')
        # Use BEST_EFFORT QoS for all communication
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Use shared servo polarity configuration
        self.servo_polarity = SERVO_POLARITY
        
        # Initialize state variables
        self.gdb0_state: Optional[GDBState] = None
        self.gdb1_state: Optional[GDBState] = None
        self.oak_imu: Optional[Imu] = None
        self.back_state: Optional[BackState] = None
        self.jetson_state: Optional[JetsonState] = None
        self.current_cmd_vel: Optional[Twist] = None
        self.command_cmd_vel: Optional[Twist] = None
        
        # Initialize watchdog health checks
        self.health_checks: Dict[str, HealthCheck] = create_health_checks()
        
        # Subscribe to gdb states (use BEST_EFFORT to match gdb publishers)
        self.gdb0_sub = self.create_subscription(
            GDBState,
            'low/gdb0state',
            self.gdb0_callback,
            qos_best_effort
        )
        
        self.gdb1_sub = self.create_subscription(
            GDBState,
            'low/gdb1state',
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
        
        # Subscribe to back state (use BEST_EFFORT QoS to match micro-ROS publisher)
        self.back_state_sub = self.create_subscription(
            BackState,
            'low/backstate',
            self.back_state_callback,
            qos_best_effort
        )
        
        # Subscribe to Jetson state (use BEST_EFFORT)
        self.jetson_sub = self.create_subscription(
            JetsonState,
            'low/jetsonstate',
            self.jetson_callback,
            qos_best_effort
        )

        # Subscribe to velocity feedback from master_cmd
        self.current_cmd_vel_sub = self.create_subscription(
            Twist, 'low/current_cmd_vel',
            self.current_cmd_vel_callback, qos_best_effort)
        self.command_cmd_vel_sub = self.create_subscription(
            Twist, 'low/command_cmd_vel',
            self.command_cmd_vel_callback, qos_best_effort)

        # Publisher for robot low state (use BEST_EFFORT)
        self.robot_state_pub = self.create_publisher(
            RobotLowState,
            'robotlowstate',
            qos_best_effort
        )
        
        # Publisher for joint states (use BEST_EFFORT)
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            qos_best_effort
        )
        
        # Create timer for 100Hz publishing (0.01 seconds = 10ms)
        self.state_timer = self.create_timer(PUBLISH_PERIOD_SEC, self.publish_robot_state)
        
        # Create watchdog timer for 1Hz health checks
        self.watchdog_timer = self.create_timer(WATCHDOG_PERIOD_SEC, self.run_health_checks)
        
        self.get_logger().info(f'Master Status Node started - publishing at {PUBLISH_RATE_HZ}Hz, watchdog at {WATCHDOG_RATE_HZ}Hz')
    
    # ========================================================================
    # Callback Methods
    # ========================================================================
    
    def gdb0_callback(self, msg: GDBState) -> None:
        """Callback for gdb0state"""
        self.gdb0_state = msg
        
        # Update watchdog health checks
        self.health_checks['gdb0_rate'].update()
        self.health_checks['gdb0_voltage'].update(msg.servo_state)
        self.health_checks['gdb0_timing'].update(msg.driver_diagnostics)
        self.health_checks['gdb0_servos'].update(msg.servo_state)
        self.health_checks['gdb0_board_temp'].update(float(msg.board_temp))
    
    def gdb1_callback(self, msg: GDBState) -> None:
        """Callback for gdb1state"""
        self.gdb1_state = msg
        
        # Update watchdog health checks
        self.health_checks['gdb1_rate'].update()
        self.health_checks['gdb1_voltage'].update(msg.servo_state)
        self.health_checks['gdb1_timing'].update(msg.driver_diagnostics)
        self.health_checks['gdb1_servos'].update(msg.servo_state)
        self.health_checks['gdb1_board_temp'].update(float(msg.board_temp))
    
    def oak_imu_callback(self, msg: Imu) -> None:
        """Callback for Oak IMU data"""
        self.oak_imu = msg
    
    def back_state_callback(self, msg: BackState) -> None:
        """Callback for back state"""
        self.back_state = msg
    
    def jetson_callback(self, msg: JetsonState) -> None:
        """Callback for Jetson state"""
        self.jetson_state = msg

        # Update watchdog health checks for Jetson
        self.health_checks['jetson_cpu_temp'].update(msg.cpu_temp)
        self.health_checks['jetson_gpu_temp'].update(msg.gpu_temp)
        self.health_checks['jetson_thermal'].update(msg.thermal_temp)
        self.health_checks['jetson_ram'].update(msg.ram_usage_percent)
        self.health_checks['jetson_swap'].update(msg.swap_usage_percent)
        self.health_checks['jetson_disk'].update(msg.disk_usage_percent)
        self.health_checks['jetson_cpu_load'].update(msg.cpu_usage_percent)
        self.health_checks['jetson_wifi'].update(
            connected=msg.wifi_connected,
            signal_dbm=msg.wifi_signal_dbm,
            ssid=msg.wifi_ssid
        )

    def current_cmd_vel_callback(self, msg: Twist) -> None:
        """Callback for rate-limited cmd_vel from master_cmd"""
        self.current_cmd_vel = msg

    def command_cmd_vel_callback(self, msg: Twist) -> None:
        """Callback for original commanded cmd_vel from master_cmd"""
        self.command_cmd_vel = msg

    # ========================================================================
    # Publishing Methods
    # ========================================================================
    
    def publish_robot_state(self) -> None:
        """Combine gdb states and publish robot state"""
        # Only publish if we have data from both gdbs
        if self.gdb0_state is None or self.gdb1_state is None:
            self.get_logger().debug('Waiting for both gdb states...')
            return
        
        robot_state = RobotLowState()
        
        # Populate header
        robot_state.header.stamp = self.get_clock().now().to_msg()
        robot_state.header.frame_id = 'base_link'
        
        # Populate sensor data
        robot_state.imu = self._build_imu_list()
        robot_state.magnetic_field = self._build_magnetic_field_list()
        
        # Back state (from back controller)
        if self.back_state is not None:
            robot_state.back_state = self.back_state
        else:
            # Create placeholder if back state not available
            robot_state.back_state = BackState()
        
        # Eye state (2 PWM values from gdb1 motor states)
        robot_state.eye_state = self._build_eye_state()
        
        # Motor states (4 driver motors: FL, FR, RL, RR)
        robot_state.motor_state = self._build_motor_states()
        
        # Servo states (17 total: 0-6 left arm, 7-13 right arm, 14-16 head)
        robot_state.servo_state = self._build_servo_states()

        # Velocity command state from master_cmd
        if self.current_cmd_vel is not None:
            robot_state.current_cmd_vel = self.current_cmd_vel
        if self.command_cmd_vel is not None:
            robot_state.command_cmd_vel = self.command_cmd_vel

        self.robot_state_pub.publish(robot_state)
        
        # Publish joint states for visualization and TF
        joint_state = self._build_joint_state(robot_state)
        self.joint_state_pub.publish(joint_state)
        # self.get_logger().debug('Published robot state')
    
    # ========================================================================
    # Helper Methods for Building State Messages
    # ========================================================================
    
    def _create_oak_imu_msg(self) -> Imu:
        """Create Oak IMU message or placeholder
        
        Returns:
            Imu message from Oak camera if available, otherwise empty placeholder
        """
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
            return oak_imu_copy
        else:
            # Oak IMU not available yet, create placeholder
            oak_imu_placeholder = Imu()
            oak_imu_placeholder.header.stamp = self.get_clock().now().to_msg()
            oak_imu_placeholder.header.frame_id = 'oak_imu_frame'
            return oak_imu_placeholder
    
    def _build_imu_list(self) -> List[Imu]:
        """Build list of IMU messages from all sensors
        
        Returns:
            List of 3 IMU messages: Oak IMU (head), gdb0 IMU (upper board), gdb1 IMU (lower board)
        """
        return [
            self._create_oak_imu_msg(),  # Oak IMU (head/camera)
            self.convert_gdb_imu_to_imu(self.gdb0_state.imu, 'gdb0_imu'),  # Upper GDB board
            self.convert_gdb_imu_to_imu(self.gdb1_state.imu, 'gdb1_imu')   # Lower GDB board
        ]
    
    def _build_magnetic_field_list(self) -> List[MagneticField]:
        """Build list of magnetic field messages
        
        Returns:
            List of 2 MagneticField messages from gdb0 and gdb1
        """
        return [
            self.convert_gdb_magnetic_field_to_magnetic_field(self.gdb0_state.magnetic_field, 'gdb0_mag'),
            self.convert_gdb_magnetic_field_to_magnetic_field(self.gdb1_state.magnetic_field, 'gdb1_mag')
        ]
    
    def _build_eye_state(self) -> List[int]:
        """Build eye state - currently unpopulated
        
        Returns:
            List of 2 uint8 values (0-255) representing eye PWM states
        """
        return [0, 0]
    
    def _build_motor_states(self) -> List[MotorState]:
        """Build motor state list from all 4 drive motors
        
        Returns:
            List of 4 MotorState messages in order: FL, FR, RL, RR
            - motor_state[0] = Front Left  (gdb1.motor_state[0])
            - motor_state[1] = Front Right (gdb1.motor_state[1])
            - motor_state[2] = Rear Left   (gdb0.motor_state[0])
            - motor_state[3] = Rear Right  (gdb0.motor_state[1])
        """
        return [
            self.convert_gdb_motor_to_motor(self.gdb1_state.motor_state[0]),  # FL
            self.convert_gdb_motor_to_motor(self.gdb1_state.motor_state[1]),  # FR
            self.convert_gdb_motor_to_motor(self.gdb0_state.motor_state[0]),  # RL
            self.convert_gdb_motor_to_motor(self.gdb0_state.motor_state[1])   # RR
        ]
    
    def _build_servo_states(self) -> List[ServoState]:
        """Build complete servo state list from both GDB boards
        
        Returns:
            List of 15 ServoState messages (skipping 3rd servo from each GDB board)
        """
        servo_states = []
        
        # gdb1 has servos 0-6 (left arm - first 7 servos)
        # Skip index 2 (3rd servo which is derived)
        # Map gdb1 indices to polarity indices: 0->0, 1->1, 3->3, 4->4, 5->5, 6->6
        for i in range(NUM_SERVOS_GDB1):
            if i == 2:  # Skip the 3rd servo (index 2) - it's derived
                continue
            # Polarity index matches gdb1 servo index for non-derived servos
            polarity_index = i
            servo_states.append(
                self.convert_gdb_servo_to_servo(self.gdb1_state.servo_state[i], polarity_index)
            )
        
        # gdb0 has servos 7-16 (right arm 7-13, head 14-16 - 10 servos)
        # Skip index 2 (3rd servo which is derived)
        # Map gdb0 indices to polarity indices: 0->7, 1->8, 3->10, 4->11, 5->12, 6->13, 7->14, 8->15, 9->16
        for i in range(NUM_SERVOS_GDB0):
            if i == 2:  # Skip the 3rd servo (index 2) - it's derived
                continue
            # gdb0 servo index i maps directly to polarity index 7+i
            # (since we skip i=2 in the loop, which corresponds to derived polarity index 9)
            polarity_index = 7 + i
            servo_states.append(
                self.convert_gdb_servo_to_servo(self.gdb0_state.servo_state[i], polarity_index)
            )
        
        return servo_states
    
    def _build_joint_state(self, robot_state: RobotLowState) -> JointState:
        """Build JointState message from robot state for visualization and TF
        
        Args:
            robot_state: The complete robot low state message
            
        Returns:
            JointState message with all joint positions
        """
        joint_state = JointState()
        joint_state.header.stamp = robot_state.header.stamp
        joint_state.header.frame_id = ''
        
        names = []
        positions = []
        
        # Track gripper active joint positions for passive joint synthesis
        left_gripper_active_pos = None
        right_gripper_active_pos = None
        
        # Add servo joints (skip derived servos at indices 2 and 9)
        servo_state_idx = 0
        for i, joint_name in enumerate(SERVO_JOINT_NAMES):
            if joint_name is None:
                # Derived servo - skip (not in servo_state list)
                continue
            
            if servo_state_idx < len(robot_state.servo_state):
                servo = robot_state.servo_state[servo_state_idx]
                names.append(joint_name)
                positions.append(servo.current_location)
                
                # Track gripper active joint positions
                if joint_name == 'left_gripper_active_joint':
                    left_gripper_active_pos = servo.current_location
                elif joint_name == 'right_gripper_active_joint':
                    right_gripper_active_pos = servo.current_location
                
                servo_state_idx += 1
        
        # Add synthesized passive gripper joints (negated from active)
        if left_gripper_active_pos is not None:
            names.append('left_gripper_passive_joint')
            positions.append(-left_gripper_active_pos)
        
        if right_gripper_active_pos is not None:
            names.append('right_gripper_passive_joint')
            positions.append(-right_gripper_active_pos)
        
        # Add motor joints (mecanum wheels)
        for i, joint_name in enumerate(MOTOR_JOINT_NAMES):
            if i < len(robot_state.motor_state):
                motor = robot_state.motor_state[i]
                names.append(joint_name)
                # Convert pulse count to radians (22 pulses per revolution)
                positions.append(float(motor.pulse_count) * MOTOR_PULSES_TO_RADIANS)
        
        # Add back joint (prismatic)
        if robot_state.back_state is not None:
            names.append('back_joint')
            # back_state.current_position is in meters for prismatic joint
            positions.append(robot_state.back_state.current_position)
        
        joint_state.name = names
        joint_state.position = positions
        
        return joint_state
    
    # ========================================================================
    # Conversion Methods
    # ========================================================================
    
    def convert_gdb_imu_to_imu(self, gdb_imu, frame_id: str) -> Imu:
        """Convert GDBImu to sensor_msgs/Imu
        
        Args:
            gdb_imu: GDB IMU data
            frame_id: Frame ID for the IMU message
            
        Returns:
            Converted Imu message
        """
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
    
    def convert_gdb_magnetic_field_to_magnetic_field(self, gdb_mag, frame_id: str) -> MagneticField:
        """Convert GDBMagneticField to sensor_msgs/MagneticField
        
        Args:
            gdb_mag: GDB magnetic field data
            frame_id: Frame ID for the magnetic field message
            
        Returns:
            Converted MagneticField message
        """
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
    
    def convert_gdb_motor_to_motor(self, gdb_motor) -> MotorState:
        """Convert GDBMotorState to MotorState
        
        Args:
            gdb_motor: GDB motor state
            
        Returns:
            Converted MotorState message
        """
        motor = MotorState()
        motor.is_moving = gdb_motor.is_moving
        motor.pwm_cmd = gdb_motor.pwm_cmd
        motor.velocity = gdb_motor.velocity
        motor.acceleration = gdb_motor.acceleration
        motor.pulse_count = gdb_motor.pulse_count
        
        return motor
    

    def convert_gdb_servo_to_servo(self, gdb_servo, servo_index: int) -> ServoState:
        """Convert GDBServoState to ServoState with polarity correction
        
        Args:
            gdb_servo: The GDB servo state to convert
            servo_index: Index (0-16) of the servo in the RobotLowState servo array
            
        Returns:
            Converted ServoState with polarity applied to position, velocity, and acceleration
        """
        servo = ServoState()
        
        # Get the polarity for this servo
        polarity = self.servo_polarity[servo_index]
        
        # Enabled state (torque switch)
        servo.enabled = (gdb_servo.torque_switch == 1)
        
        # Target location: convert from -2048 to +2048 to -π to +π radians and apply polarity
        servo.target_location = polarity * self.convert_servo_position_to_radians(gdb_servo.target_location)
        
        # Target speed: convert from steps/s to rad/s
        # Speed is 0-3400 steps/s in GDB units
        # Convert steps/s to rad/s: speed_steps_s * (2π rad / 4096 steps)
        servo.target_speed = gdb_servo.target_speed * STEPS_TO_RADIANS
        
        # Target acceleration: convert from servo units to rad/s²
        # Each unit = 100 steps/s², and 4096 steps = 2π radians
        accel_conversion = SERVO_ACCEL_UNITS_TO_STEPS_PER_SEC2 * STEPS_TO_RADIANS
        servo.target_acceleration = gdb_servo.target_acceleration * accel_conversion
        
        # Target torque: convert from 0-1000 to 0.0-1.0
        servo.target_torque = gdb_servo.target_torque / 1000.0
        
        # Current location: convert from -2048 to +2048 to -π to +π radians and apply polarity
        servo.current_location = polarity * self.convert_servo_position_to_radians(gdb_servo.current_location)
        
        # Current speed: convert from steps/s to rad/s and apply polarity
        # current_speed is int16 (can be negative)
        servo.current_speed = polarity * gdb_servo.current_speed * STEPS_TO_RADIANS
        
        # Current load: convert from raw units to percentage
        # Note: Check if SERVO_LOAD_SCALE is still correct for the new msg format
        servo.current_load = gdb_servo.current_load / 10.0  # Assuming still 0.1% units
        
        # Temperature (direct copy)
        servo.current_temperature = gdb_servo.current_temperature
        
        # Servo status (direct copy)
        servo.servo_status = gdb_servo.servo_status
        
        # Is moving flag
        servo.is_moving = (gdb_servo.mobile_sign != 0)
        
        # Voltage: convert from raw units to volts
        # Assuming still 0.1V units
        servo.current_voltage = gdb_servo.current_voltage / 10.0
        
        # Current: convert from raw units to milliamps
        # Assuming still 6.5mA units
        servo.current_current = gdb_servo.current_current * 6.5
        
        return servo
    
    def convert_servo_position_to_radians(self, raw_position: int) -> float:
        """Convert servo position from -2048 to +2048 range to -π to +π radians
        
        Args:
            raw_position: Raw servo position (-2048 to +2048 steps)
            
        Returns:
            Position in radians (-π to +π)
        """
        # Map -2048 to +2048 to -π to +π
        # -2048 steps = -π radians, +2048 steps = +π radians
        # radians = raw_position * (2π / 4096) = raw_position * π / 2048
        radians = raw_position * np.pi / 2048.0
        return radians
    
    # ========================================================================
    # Watchdog Health Check Runner
    # ========================================================================
    
    def run_health_checks(self) -> None:
        """Run all registered health checks and log errors"""
        for check_name, check in self.health_checks.items():
            error_msg = check.check()
            if error_msg:
                self.get_logger().error(error_msg)


# ============================================================================
# Main Entry Point
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MasterStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
