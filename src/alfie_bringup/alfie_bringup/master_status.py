import time
from typing import List, Optional
import rclpy
from rclpy.node import Node
from alfie_msgs.msg import RobotLowState, GDBState
from alfie_msgs.msg import ServoState, MotorState
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy


# ============================================================================
# Constants and Configuration
# ============================================================================

# Publishing configuration
PUBLISH_RATE_HZ = 100
PUBLISH_PERIOD_SEC = 1.0 / PUBLISH_RATE_HZ  # 0.01 seconds

# Servo mapping
NUM_SERVOS_GDB1 = 7  # Left arm servos (0-6), but skipping index 2
NUM_SERVOS_GDB0 = 10  # Right arm + head servos (7-16), but skipping index 2
TOTAL_SERVOS = 15  # 6 from gdb1 + 9 from gdb0 (skipping 3rd servo from each)

# Unit conversion constants
SERVO_STEPS_PER_REVOLUTION = 4096
STEPS_TO_RADIANS = 2.0 * np.pi / SERVO_STEPS_PER_REVOLUTION
SERVO_ACCEL_UNITS_TO_STEPS_PER_SEC2 = 100.0
PWM_RANGE = 255

# Servo unit conversions
SERVO_LOAD_SCALE = 10.0  # 0.1% units to percentage
SERVO_VOLTAGE_SCALE = 10.0  # 0.1V units to volts
SERVO_CURRENT_SCALE = 6.5  # 6.5mA units to milliamps


# ============================================================================
# MasterStatusNode Class
# ============================================================================

class MasterStatusNode(Node):
    def __init__(self):
        super().__init__('master_status_node')
        # Use BEST_EFFORT QoS for gdb state subscriptions (to match gdb publishers)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Use RELIABLE QoS for robot state topic
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Servo polarity array - converts between physical servo control and actual controls
        # Positive (1) means no inversion, Negative (-1) means inverted
        # Index corresponds to servo number in RobotLowState (0-14, skipping 3rd servo from each GDB board)
        # Note: The array still has 17 entries to match original GDB indices, but indices 2 and 9 are skipped
        self.servo_polarity = [
            1,   # Servo 0:  driver1/servo01 - left shoulder yaw
            1,   # Servo 1:  driver1/servo02 - left shoulder1 pitch
            1,   # Servo 2:  driver1/servo03 - left shoulder2 pitch (SKIPPED)
            1,   # Servo 3:  driver1/servo04 - left elbow pitch
            1,   # Servo 4:  driver1/servo05 - left wrist pitch
            1,   # Servo 5:  driver1/servo06 - left wrist roll
            1,   # Servo 6:  driver1/servo07 - left hand
            1,   # Servo 7:  driver0/servo01 - right shoulder yaw
            1,   # Servo 8:  driver0/servo02 - right shoulder1 pitch
            1,   # Servo 9:  driver0/servo03 - right shoulder2 pitch (SKIPPED)
            1,   # Servo 10: driver0/servo04 - right elbow pitch
            1,   # Servo 11: driver0/servo05 - right wrist pitch
            1,   # Servo 12: driver0/servo06 - right wrist roll
            1,   # Servo 13: driver0/servo07 - right hand
            1,   # Servo 14: driver0/servo08 - head yaw
            1,   # Servo 15: driver0/servo09 - head pitch
            1,   # Servo 16: driver0/servo10 - head roll
        ]
        
        # Initialize state variables
        self.gdb0_state: Optional[GDBState] = None
        self.gdb1_state: Optional[GDBState] = None
        self.oak_imu: Optional[Imu] = None
        
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
        
        # Create timer for 100Hz publishing (0.01 seconds = 10ms)
        self.state_timer = self.create_timer(PUBLISH_PERIOD_SEC, self.publish_robot_state)
        
        self.get_logger().info(f'Master Status Node started - publishing at {PUBLISH_RATE_HZ}Hz')
    
    # ========================================================================
    # Callback Methods
    # ========================================================================
    
    def gdb0_callback(self, msg: GDBState) -> None:
        """Callback for gdb0state"""
        self.gdb0_state = msg
    
    def gdb1_callback(self, msg: GDBState) -> None:
        """Callback for gdb1state"""
        self.gdb1_state = msg
    
    def oak_imu_callback(self, msg: Imu) -> None:
        """Callback for Oak IMU data"""
        self.oak_imu = msg
    
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
        
        # Shoulder limit state (from gdb1)
        robot_state.shoulder_limit_state = self.gdb1_state.shoulder_limit_state
        
        # Eye state (2 PWM values from gdb1 motor states)
        robot_state.eye_state = self._build_eye_state()
        
        # Motor states (2 driver motors from gdb0)
        robot_state.motor_state = self._build_motor_states()
        
        # Servo states (17 total: 0-6 left arm, 7-13 right arm, 14-16 head)
        robot_state.servo_state = self._build_servo_states()
        
        self.robot_state_pub.publish(robot_state)
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
        """Build eye state from gdb1 motor PWM commands
        
        Returns:
            List of 2 uint8 values (0-255) representing eye PWM states
        """
        return [
            self.convert_pwm_to_uint8(self.gdb1_state.motor_state[0].pwm_cmd),
            self.convert_pwm_to_uint8(self.gdb1_state.motor_state[1].pwm_cmd)
        ]
    
    def _build_motor_states(self) -> List[MotorState]:
        """Build motor state list from gdb0 motors
        
        Returns:
            List of 2 MotorState messages for the driver motors
        """
        return [
            self.convert_gdb_motor_to_motor(self.gdb0_state.motor_state[0]),
            self.convert_gdb_motor_to_motor(self.gdb0_state.motor_state[1])
        ]
    
    def _build_servo_states(self) -> List[ServoState]:
        """Build complete servo state list from both GDB boards
        
        Returns:
            List of 15 ServoState messages (skipping 3rd servo from each GDB board)
        """
        servo_states = []
        
        # gdb1 has servos 0-6 (left arm - first 7 servos)
        # Skip index 2 (3rd servo)
        for i in range(NUM_SERVOS_GDB1):
            if i == 2:  # Skip the 3rd servo (index 2)
                continue
            servo_states.append(
                self.convert_gdb_servo_to_servo(self.gdb1_state.servo_state[i], len(servo_states))
            )
        
        # gdb0 has servos 7-16 (right arm 7-13, head 14-16 - 10 servos)
        # Skip index 2 (3rd servo)
        for i in range(NUM_SERVOS_GDB0):
            if i == 2:  # Skip the 3rd servo (index 2)
                continue
            servo_states.append(
                self.convert_gdb_servo_to_servo(self.gdb0_state.servo_state[i], len(servo_states))
            )
        
        return servo_states
    
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
    
    def convert_pwm_to_uint8(self, pwm_cmd: int) -> int:
        """Convert PWM command (-255 to 255) to uint8 (0 to 255)
        
        Args:
            pwm_cmd: PWM command value
            
        Returns:
            Absolute value of PWM clamped to 0-255 range
        """
        # Take absolute value and clamp to 0-255
        return min(PWM_RANGE, max(0, abs(pwm_cmd)))
    
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
        
        # Acceleration: convert from units to radians per second squared (rad/s²)
        # Each unit = 100 steps/s², and 4096 steps = 2π radians
        # Apply polarity to acceleration
        accel_conversion = SERVO_ACCEL_UNITS_TO_STEPS_PER_SEC2 * STEPS_TO_RADIANS
        servo.acceleration = polarity * gdb_servo.acceleration * accel_conversion
        
        # Position: convert from 0-4096 to -π to +π radians and apply polarity
        servo.target_location = polarity * self.convert_servo_position_to_radians(gdb_servo.targetlocation)
        servo.current_location = polarity * self.convert_servo_position_to_radians(gdb_servo.currentlocation)
        
        # Velocity: convert from steps per second to radians per second and apply polarity
        servo.current_speed = polarity * gdb_servo.currentspeed * STEPS_TO_RADIANS
        
        # Load: convert from 0.1% units to percentage
        servo.current_load = gdb_servo.currentload * SERVO_LOAD_SCALE
        
        servo.current_temperature = gdb_servo.currenttemperature
        servo.servo_status = gdb_servo.servostatus
        servo.is_moving = (gdb_servo.mobilesign != 0)
        
        # Voltage: convert from 0.1V units to volts
        servo.current_voltage = gdb_servo.currentvoltage / SERVO_VOLTAGE_SCALE
        
        # Current: convert from 6.5mA units to milliamps
        servo.current_current = gdb_servo.currentcurrent * SERVO_CURRENT_SCALE
        
        return servo
    
    def convert_servo_position_to_radians(self, raw_position: int) -> float:
        """Convert servo position from 0-4096 range to -π to +π radians
        
        Args:
            raw_position: Raw servo position (0-4096 steps)
            
        Returns:
            Position in radians (-π to +π)
        """
        # Map 0-4096 to -π to +π
        # First normalize to 0-1, then scale to -π to +π
        normalized = raw_position / SERVO_STEPS_PER_REVOLUTION
        radians = (normalized * 2.0 * np.pi) - np.pi
        return radians


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
