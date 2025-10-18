import time
from typing import Optional
import rclpy
from rclpy.node import Node
from alfie_msgs.msg import RobotLowCmd, GDBCmd, GDBServoCmd, ServoCmd
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np


# ============================================================================
# Constants and Configuration
# ============================================================================

# Publishing configuration
PUBLISH_RATE_HZ = 100
PUBLISH_PERIOD_SEC = 1.0 / PUBLISH_RATE_HZ  # 0.01 seconds
CMD_TIMEOUT_SEC = 0.1
WARNING_THROTTLE_SEC = 1.0

# Servo mapping
NUM_SERVOS_GDB1 = 7  # Left arm servos (0-6), but 3rd is derived from 2nd
NUM_SERVOS_GDB0 = 10  # Right arm + head servos (7-16), but 3rd is derived from 2nd
TOTAL_SERVOS_IN_CMD = 15  # Total servos in RobotLowCmd (skipping 3rd from each GDB)

# Unit conversion constants
SERVO_STEPS_PER_REVOLUTION = 4096
RADIANS_TO_STEPS = SERVO_STEPS_PER_REVOLUTION / (2.0 * np.pi)
SERVO_ACCEL_UNITS_TO_STEPS_PER_SEC2 = 100.0


# ============================================================================
# MasterCmdNode Class
# ============================================================================

class MasterCmdNode(Node):
    def __init__(self):
        super().__init__('master_cmd_node')
        # Use RELIABLE QoS for command topics
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Servo polarity array - converts between physical servo control and actual controls
        # Positive (1) means no inversion, Negative (-1) means inverted
        # Index corresponds to GDB servo indices (0-6 for gdb1, 0-9 for gdb0)
        # Note: Index 2 for each GDB is derived from index 1, not from RobotLowCmd
        self.servo_polarity = [
            1,   # Servo 0:  driver1/servo01 - left shoulder yaw
            1,   # Servo 1:  driver1/servo02 - left shoulder1 pitch
            1,   # Servo 2:  driver1/servo03 - left shoulder2 pitch (DERIVED from index 1)
            1,   # Servo 3:  driver1/servo04 - left elbow pitch
            1,   # Servo 4:  driver1/servo05 - left wrist pitch
            1,   # Servo 5:  driver1/servo06 - left wrist roll
            1,   # Servo 6:  driver1/servo07 - left hand
            1,   # Servo 7:  driver0/servo01 - right shoulder yaw
            1,   # Servo 8:  driver0/servo02 - right shoulder1 pitch
            1,   # Servo 9:  driver0/servo03 - right shoulder2 pitch (DERIVED from index 1)
            1,   # Servo 10: driver0/servo04 - right elbow pitch
            1,   # Servo 11: driver0/servo05 - right wrist pitch
            1,   # Servo 12: driver0/servo06 - right wrist roll
            1,   # Servo 13: driver0/servo07 - right hand
            1,   # Servo 14: driver0/servo08 - head yaw
            1,   # Servo 15: driver0/servo09 - head pitch
            1,   # Servo 16: driver0/servo10 - head roll
        ]
        
        # Initialize command variables and timestamp
        self.gdb0_cmd: Optional[GDBCmd] = None
        self.gdb1_cmd: Optional[GDBCmd] = None
        self.last_robot_cmd_time = None
        self.last_warn_time = None  # For throttling warnings
        
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
        
        # Create timer for 100Hz publishing (0.01 seconds = 10ms)
        self.cmd_timer = self.create_timer(PUBLISH_PERIOD_SEC, self.publish_gdb_commands)
        
        self.get_logger().info(f'Master Cmd Node started - publishing at {PUBLISH_RATE_HZ}Hz')
    
    # ========================================================================
    # Callback Methods
    # ========================================================================
    
    def robot_cmd_callback(self, msg: RobotLowCmd) -> None:
        """Decompose RobotLowCmd into GDBCmd messages with unit conversions
        
        RobotLowCmd has 15 servo commands. We derive the 3rd servo (index 2) for each
        GDB board from the 2nd servo (index 1) by copying enable/acceleration and
        flipping the sign of the target location.
        """
        # Update timestamp
        self.last_robot_cmd_time = self.get_clock().now()
        
        # Create GDBCmd for gdb0 (motors are on gdb0)
        gdb0_cmd = GDBCmd()
        gdb0_cmd.driver_pwm = msg.driver_pwm  # 2 motors on gdb0
        
        # gdb0 gets servos from RobotLowCmd indices 6-14 (9 servos)
        # We need to create 10 servos for GDB, deriving index 2 from index 1
        gdb0_servo_list = []
        robotlowcmd_index = 6  # Start from 6th servo in the 15-servo array
        
        for gdb_servo_index in range(NUM_SERVOS_GDB0):
            if gdb_servo_index == 2:  # Derive the 3rd servo from the 2nd
                # Get the 2nd servo command (index 1) which is at robotlowcmd_index - 1
                servo_cmd_source = msg.servo_cmd[robotlowcmd_index - 1]
                # Create derived servo command with flipped target location
                derived_servo = self.derive_servo_cmd(servo_cmd_source, gdb_servo_index, flip_sign=True)
                gdb0_servo_list.append(derived_servo)
            else:
                servo_cmd = self.convert_servo_cmd_to_gdb(msg.servo_cmd[robotlowcmd_index], robotlowcmd_index)
                gdb0_servo_list.append(servo_cmd)
                robotlowcmd_index += 1
        
        # GDBCmd expects exactly 10 servos
        gdb0_cmd.servo_cmd = gdb0_servo_list
        
        # Create GDBCmd for gdb1 (eye_lights are on gdb1)
        gdb1_cmd = GDBCmd()
        gdb1_cmd.driver_pwm = msg.eye_pwm  # 2 eye lights on gdb1
        
        # gdb1 gets servos from RobotLowCmd indices 0-5 (6 servos)
        # We need to create 7 servos for GDB, deriving index 2 from index 1
        gdb1_servo_list = []
        robotlowcmd_index = 0
        
        for gdb_servo_index in range(NUM_SERVOS_GDB1):
            if gdb_servo_index == 2:  # Derive the 3rd servo from the 2nd
                # Get the 2nd servo command (index 1) which is at robotlowcmd_index - 1
                servo_cmd_source = msg.servo_cmd[robotlowcmd_index - 1]
                # Create derived servo command with flipped target location
                derived_servo = self.derive_servo_cmd(servo_cmd_source, gdb_servo_index, flip_sign=True)
                gdb1_servo_list.append(derived_servo)
            else:
                servo_cmd = self.convert_servo_cmd_to_gdb(msg.servo_cmd[robotlowcmd_index], robotlowcmd_index)
                gdb1_servo_list.append(servo_cmd)
                robotlowcmd_index += 1
        
        # Pad with 3 default GDBServoCmd for the remaining servos to make 10 total
        for _ in range(3):
            gdb1_servo_list.append(GDBServoCmd())
        gdb1_cmd.servo_cmd = gdb1_servo_list
        
        # Store the commands (they will be published by the timer)
        self.gdb0_cmd = gdb0_cmd
        self.gdb1_cmd = gdb1_cmd
        self.get_logger().debug('Received robot command')
    
    # ========================================================================
    # Publishing Methods
    # ========================================================================
    
    def publish_gdb_commands(self) -> None:
        """Publish gdb commands at 100Hz with failsafe timeout"""
        # Check if we have commands and they're recent enough
        if not self._is_command_fresh():
            return
        
        # Publish the commands
        self.gdb0_cmd_pub.publish(self.gdb0_cmd)
        self.gdb1_cmd_pub.publish(self.gdb1_cmd)
        self.get_logger().debug('Published gdb commands')
    
    # ========================================================================
    # Helper Methods
    # ========================================================================
    
    def convert_servo_cmd_to_gdb(self, servo_cmd: ServoCmd, servo_index: int) -> GDBServoCmd:
        """Convert ServoCmd (standard units) to GDBServoCmd (servo units)
        
        Args:
            servo_cmd: ServoCmd message with radians and rad/s² units
            servo_index: Index (0-14) of the servo in the RobotLowCmd servo array
            
        Returns:
            GDBServoCmd with servo-specific units
        """
        gdb_servo = GDBServoCmd()
        
        # Get the polarity for this servo (map from RobotLowCmd index to GDB index)
        # For servos 0-5, use polarity indices 0,1,3,4,5,6 (skipping 2)
        # For servos 6-14, use polarity indices 7,8,10,11,12,13,14,15,16 (skipping 9)
        if servo_index < 6:
            # gdb1 servos: map 0-5 -> 0,1,3,4,5,6
            polarity_index = servo_index if servo_index < 2 else servo_index + 1
        else:
            # gdb0 servos: map 6-14 -> 7,8,10,11,12,13,14,15,16
            gdb0_offset = servo_index - 6
            polarity_index = 7 + (gdb0_offset if gdb0_offset < 2 else gdb0_offset + 1)
        
        polarity = self.servo_polarity[polarity_index]
        
        # Torque switch (enabled state)
        gdb_servo.torqueswitch = 1 if servo_cmd.enabled else 0
        
        # Acceleration: convert from rad/s² to servo units
        # Each unit = 100 steps/s², and 4096 steps = 2π radians
        # accel_units = accel_rad_s2 / (100 steps/s² * 2π/4096 rad/step)
        accel_conversion = SERVO_ACCEL_UNITS_TO_STEPS_PER_SEC2 * (2.0 * np.pi / SERVO_STEPS_PER_REVOLUTION)
        accel_servo_units = abs(servo_cmd.acceleration / accel_conversion)
        # Clamp to 0-254 range and convert to uint8
        gdb_servo.acceleration = int(np.clip(accel_servo_units, 0, 254))
        
        # Target location: convert from radians (-π to +π) to steps (0-4096)
        # Apply polarity inversion
        target_radians = polarity * servo_cmd.target_location
        gdb_servo.targetlocation = self.convert_radians_to_servo_position(target_radians)
        
        return gdb_servo
    
    def derive_servo_cmd(self, servo_cmd: ServoCmd, gdb_servo_index: int, flip_sign: bool = True) -> GDBServoCmd:
        """Derive a GDBServoCmd from another ServoCmd by flipping the target location sign
        
        This is used to create the 3rd servo command (index 2) from the 2nd servo (index 1)
        for each GDB board. We copy the enable and acceleration, but flip the sign of the
        target location before conversion.
        
        Args:
            servo_cmd: Source ServoCmd message with radians and rad/s² units
            gdb_servo_index: Index (0-9 for gdb0, 0-6 for gdb1) in the GDB servo array
            flip_sign: Whether to flip the sign of the target location (default True)
            
        Returns:
            GDBServoCmd with servo-specific units and flipped target location
        """
        gdb_servo = GDBServoCmd()
        
        # Get the polarity for this servo (should be index 2 or 9 in polarity array)
        polarity = self.servo_polarity[gdb_servo_index]
        
        # Torque switch (enabled state) - copied from source
        gdb_servo.torqueswitch = 1 if servo_cmd.enabled else 0
        
        # Acceleration - copied from source
        accel_conversion = SERVO_ACCEL_UNITS_TO_STEPS_PER_SEC2 * (2.0 * np.pi / SERVO_STEPS_PER_REVOLUTION)
        accel_servo_units = abs(servo_cmd.acceleration / accel_conversion)
        gdb_servo.acceleration = int(np.clip(accel_servo_units, 0, 254))
        
        # Target location: flip the sign before conversion
        target_radians = servo_cmd.target_location
        if flip_sign:
            target_radians = -target_radians
        
        # Apply polarity inversion
        target_radians = polarity * target_radians
        gdb_servo.targetlocation = self.convert_radians_to_servo_position(target_radians)
        
        return gdb_servo
    
    def convert_radians_to_servo_position(self, radians: float) -> int:
        """Convert radians (-π to +π) to servo position (0-4096)
        
        Args:
            radians: Angle in radians from -π to +π
            
        Returns:
            Servo position as integer from 0 to 4096
        """
        # Normalize to 0-2π range
        # -π to +π becomes 0 to 2π by adding π
        normalized_radians = radians + np.pi
        
        # Convert to steps (0-4096)
        steps = normalized_radians * RADIANS_TO_STEPS
        
        # Clamp to valid range and convert to int16
        return int(np.clip(steps, 0, 4095))
    
    def _is_command_fresh(self) -> bool:
        """Check if robot command is recent enough to be published
        
        Returns:
            True if command is fresh and should be published, False otherwise
        """
        # Check if we have commands
        if self.gdb0_cmd is None or self.gdb1_cmd is None:
            return False
        
        if self.last_robot_cmd_time is None:
            return False
        
        # Check if the last command is within the timeout period
        time_since_last_cmd = (self.get_clock().now() - self.last_robot_cmd_time).nanoseconds / 1e9
        
        if time_since_last_cmd > CMD_TIMEOUT_SEC:
            # Throttle warning to once per second
            current_time = self.get_clock().now()
            if self.last_warn_time is None or (current_time - self.last_warn_time).nanoseconds / 1e9 >= WARNING_THROTTLE_SEC:
                self.get_logger().warn(
                    f'No robot command received for {time_since_last_cmd:.3f}s - NOT publishing gdb commands')
                self.last_warn_time = current_time
            return False
        
        return True


# ============================================================================
# Main Entry Point
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MasterCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
