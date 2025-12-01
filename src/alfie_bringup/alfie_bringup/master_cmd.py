import time
from typing import Optional
import rclpy
from rclpy.node import Node
from alfie_msgs.msg import RobotLowCmd, GDBCmd, GDBServoCmd, ServoCmd, BackCmd
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from .servo_config import SERVO_POLARITY


# ============================================================================
# Constants and Configuration
# ============================================================================

# Publishing configuration
PUBLISH_RATE_HZ = 100
PUBLISH_PERIOD_SEC = 1.0 / PUBLISH_RATE_HZ  # 0.01 seconds
CMD_TIMEOUT_SEC = 0.1
WARNING_THROTTLE_SEC = 1.0

# Mecanum drive kinematics configuration
WHEELBASE_LENGTH_MM = 170.5  # Distance between front and rear axles (mm)
WHEELBASE_WIDTH_MM = 183.0   # Distance between left and right wheels (mm)
WHEEL_SEPARATION_X = WHEELBASE_LENGTH_MM / 2000.0  # Convert to meters
WHEEL_SEPARATION_Y = WHEELBASE_WIDTH_MM / 2000.0   # Convert to meters

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
        
        # Use shared servo polarity configuration
        self.servo_polarity = SERVO_POLARITY
        
        # Initialize command variables and timestamp
        self.gdb0_cmd: Optional[GDBCmd] = None
        self.gdb1_cmd: Optional[GDBCmd] = None
        self.back_cmd: Optional[BackCmd] = None
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
            'low/gdb0cmd',
            qos_reliable
        )
        
        self.gdb1_cmd_pub = self.create_publisher(
            GDBCmd,
            'low/gdb1cmd',
            qos_reliable
        )
        
        # Publisher for back command (use RELIABLE)
        self.back_cmd_pub = self.create_publisher(
            BackCmd,
            'low/backcmd',
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
        
        RobotLowCmd has 15 servo commands and a Twist message for velocity control.
        We derive the 3rd servo (index 2) for each GDB board from the 2nd servo (index 1)
        by copying enable/acceleration and flipping the sign of the target location.
        
        The Twist cmd_vel is decomposed into individual wheel velocities using mecanum
        drive inverse kinematics.
        """
        # Update timestamp
        self.last_robot_cmd_time = self.get_clock().now()
        
        # Decompose Twist message into wheel velocities using mecanum kinematics
        wheel_velocities = self.mecanum_drive_kinematics(
            msg.cmd_vel.linear.x,
            msg.cmd_vel.linear.y,
            msg.cmd_vel.angular.z
        )
        
        # Create GDBCmd for gdb0 
        # gdb0 gets rear wheels: RL (wheel_velocities[2]) and RR (wheel_velocities[3])
        gdb0_cmd = GDBCmd()
        gdb0_cmd.velocities = [float(wheel_velocities[2]), float(wheel_velocities[3])]  # Rear Left, Rear Right
        
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
        
        # Create GDBCmd for gdb1
        # gdb1 gets front wheels: FL (wheel_velocities[0]) and FR (wheel_velocities[1])
        gdb1_cmd = GDBCmd()
        gdb1_cmd.velocities = [float(wheel_velocities[0]), float(wheel_velocities[1])]  # Front Left, Front Right
        
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
        self.back_cmd = msg.back_cmd
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
        if self.back_cmd is not None:
            self.back_cmd_pub.publish(self.back_cmd)
        self.get_logger().debug('Published gdb commands')
    
    # ========================================================================
    # Helper Methods
    # ========================================================================
    
    def mecanum_drive_kinematics(self, linear_x: float, linear_y: float, 
                                  angular_z: float) -> list:
        """Calculate wheel velocities from robot velocities using mecanum kinematics
        
        Uses inverse kinematics to convert desired robot motion (linear_x, linear_y, 
        angular_z) into individual wheel velocities for a mecanum drive system.
        
        Args:
            linear_x: Forward/backward velocity in m/s (positive = forward)
            linear_y: Left/right strafe velocity in m/s (positive = left)
            angular_z: Rotational velocity in rad/s (positive = counter-clockwise)
            
        Returns:
            List of 4 wheel velocities [FL, FR, RL, RR] in m/s
        """
        # Mecanum wheel kinematics equations
        # Based on robot geometry and wheel arrangement
        wheel_separation_sum = WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y
        
        # Calculate wheel velocities
        # FL = Front Left, FR = Front Right, RL = Rear Left, RR = Rear Right
        # Note: Right wheels have inverted linear_y due to physical motor mounting/wiring
        fl = linear_x - linear_y - angular_z * wheel_separation_sum  # Front Left
        fr = linear_x - linear_y + angular_z * wheel_separation_sum  # Front Right (inverted strafe)
        rl = linear_x + linear_y - angular_z * wheel_separation_sum  # Rear Left
        rr = linear_x + linear_y + angular_z * wheel_separation_sum  # Rear Right (inverted strafe)
        
        return [fl, fr, rl, rr]
    
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
        gdb_servo.torque_switch = 1 if servo_cmd.enabled else 0
        
        # Target location: convert from radians (-π to +π) to steps (-2048 to +2048)
        # Apply polarity inversion
        target_radians = polarity * servo_cmd.target_location
        gdb_servo.target_location = self.convert_radians_to_servo_position(target_radians)
        
        # Target speed: convert from rad/s to steps/s
        # Speed is 0-3400 steps/s in GDB units
        # Convert rad/s to steps/s: speed_rad_s * (4096 steps / 2π rad)
        speed_steps_per_sec = abs(servo_cmd.target_speed) * RADIANS_TO_STEPS
        gdb_servo.target_speed = int(np.clip(speed_steps_per_sec, 0, 3400))
        
        # Acceleration: convert from rad/s² to servo units
        # Each unit = 100 steps/s², and 4096 steps = 2π radians
        # accel_units = accel_rad_s2 / (100 steps/s² * 2π/4096 rad/step)
        accel_conversion = SERVO_ACCEL_UNITS_TO_STEPS_PER_SEC2 * (2.0 * np.pi / SERVO_STEPS_PER_REVOLUTION)
        accel_servo_units = abs(servo_cmd.target_acceleration / accel_conversion)
        # Clamp to 0-254 range and convert to uint8
        gdb_servo.target_acceleration = int(np.clip(accel_servo_units, 0, 254))
        
        # Target torque: 0-1000 range (0-100% of locked-rotor torque)
        # Input is 0.0-1.0, output is 0-1000
        torque_value = servo_cmd.target_torque * 1000.0
        gdb_servo.target_torque = int(np.clip(torque_value, 0, 1000))
        
        return gdb_servo
    
    def derive_servo_cmd(self, servo_cmd: ServoCmd, gdb_servo_index: int, flip_sign: bool = True) -> GDBServoCmd:
        """Derive a GDBServoCmd from another ServoCmd by flipping the target location sign
        
        This is used to create the 3rd servo command (index 2) from the 2nd servo (index 1)
        for each GDB board. We copy the enable, speed, acceleration, and torque, but flip 
        the sign of the target location before conversion.
        
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
        gdb_servo.torque_switch = 1 if servo_cmd.enabled else 0
        
        # Target location: flip the sign before conversion
        target_radians = servo_cmd.target_location
        if flip_sign:
            target_radians = -target_radians
        
        # Apply polarity inversion
        target_radians = polarity * target_radians
        gdb_servo.target_location = self.convert_radians_to_servo_position(target_radians)
        
        # Target speed - copied from source
        speed_steps_per_sec = abs(servo_cmd.target_speed) * RADIANS_TO_STEPS
        gdb_servo.target_speed = int(np.clip(speed_steps_per_sec, 0, 3400))
        
        # Acceleration - copied from source
        accel_conversion = SERVO_ACCEL_UNITS_TO_STEPS_PER_SEC2 * (2.0 * np.pi / SERVO_STEPS_PER_REVOLUTION)
        accel_servo_units = abs(servo_cmd.target_acceleration / accel_conversion)
        gdb_servo.target_acceleration = int(np.clip(accel_servo_units, 0, 254))
        
        # Target torque - copied from source
        torque_value = servo_cmd.target_torque * 1000.0
        gdb_servo.target_torque = int(np.clip(torque_value, 0, 1000))
        
        return gdb_servo
    
    def convert_radians_to_servo_position(self, radians: float) -> int:
        """Convert radians (-π to +π) to servo position (-2048 to +2048)
        
        Args:
            radians: Angle in radians from -π to +π
            
        Returns:
            Servo position as integer from -2048 to +2048
        """
        # Convert radians to steps
        # -π to +π maps to -2048 to +2048
        steps = radians * RADIANS_TO_STEPS
        
        # Clamp to valid range and convert to int16
        return int(np.clip(steps, -2048, 2048))
    
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
