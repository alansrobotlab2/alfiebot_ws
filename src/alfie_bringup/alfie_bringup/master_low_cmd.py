from typing import Optional
import rclpy
from rclpy.node import Node
from alfie_msgs.msg import RobotLowCmd, ArmCmd, HeadCmd, BackCmd
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


# ============================================================================
# Constants and Configuration
# ============================================================================

# Publishing configuration. The gen2 firmware modules run their comms at ~50 Hz,
# so there is nothing to gain from re-forwarding commands faster than that.
PUBLISH_RATE_HZ = 50
PUBLISH_PERIOD_SEC = 1.0 / PUBLISH_RATE_HZ

# Failsafe: stop forwarding commands if none have arrived recently, so the module
# firmware watchdogs fall back to their safe/hold behavior.
CMD_TIMEOUT_SEC = 0.1
WARNING_THROTTLE_SEC = 1.0

# Servo layout in RobotLowCmd.servo_cmd (0-5 left arm, 6-11 right arm, 12-14 head)
NUM_ARM_SERVOS = 6
NUM_HEAD_SERVOS = 3
LEFT_ARM_SLICE = slice(0, NUM_ARM_SERVOS)                 # 0-5
RIGHT_ARM_SLICE = slice(NUM_ARM_SERVOS, 2 * NUM_ARM_SERVOS)  # 6-11
HEAD_SLICE = slice(2 * NUM_ARM_SERVOS, 2 * NUM_ARM_SERVOS + NUM_HEAD_SERVOS)  # 12-14


# ============================================================================
# MasterLowCmdNode Class
# ============================================================================

class MasterLowCmdNode(Node):
    """Splits a single RobotLowCmd into the per-module firmware command topics.

    Gen2 hardware drives actuation through independent Pico boards, each
    subscribing to its own command:
      - left arm   -> low/left_arm/armcmd   (ArmCmd, ServoCmd[6], SI)
      - right arm  -> low/right_arm/armcmd  (ArmCmd, ServoCmd[6], SI)
      - head       -> low/headcmd           (HeadCmd, ServoCmd[3] + eye PWM)
      - back/spine -> low/backcmd           (BackCmd, linear actuator setpoint)
      - drive base -> low/mecanumdrive      (geometry_msgs/Twist)

    The module firmware handles SI->count conversion, per-servo polarity, the
    derived shoulder-pitch servo expansion, and mecanum inverse kinematics, so
    this node only slices and forwards RobotLowCmd - no unit conversion here.
    """

    def __init__(self):
        super().__init__('master_low_cmd_node')
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Latest decomposed commands (published by the timer)
        self.left_arm_cmd: Optional[ArmCmd] = None
        self.right_arm_cmd: Optional[ArmCmd] = None
        self.head_cmd: Optional[HeadCmd] = None
        self.back_cmd: Optional[BackCmd] = None
        self.drive_cmd: Optional[Twist] = None
        self.last_robot_cmd_time = None
        self.last_warn_time = None

        # ---- Subscription ---------------------------------------------------
        self.robot_cmd_sub = self.create_subscription(
            RobotLowCmd, 'robotlowcmd', self.robot_cmd_callback, qos_best_effort)

        # ---- Publishers -----------------------------------------------------
        self.left_arm_pub = self.create_publisher(ArmCmd, 'low/left_arm/armcmd', qos_best_effort)
        self.right_arm_pub = self.create_publisher(ArmCmd, 'low/right_arm/armcmd', qos_best_effort)
        self.head_pub = self.create_publisher(HeadCmd, 'low/headcmd', qos_best_effort)
        self.back_pub = self.create_publisher(BackCmd, 'low/backcmd', qos_best_effort)
        self.drive_pub = self.create_publisher(Twist, 'low/mecanumdrive', qos_best_effort)

        # ---- Timer ----------------------------------------------------------
        self.cmd_timer = self.create_timer(PUBLISH_PERIOD_SEC, self.publish_module_commands)

        self.get_logger().info(f'Master Low Cmd Node started - publishing at {PUBLISH_RATE_HZ}Hz')

    # ========================================================================
    # Callback Methods
    # ========================================================================

    def robot_cmd_callback(self, msg: RobotLowCmd) -> None:
        """Decompose RobotLowCmd into the per-module command messages.

        RobotLowCmd carries 15 servo commands (SI units), eye PWM, the back
        actuator setpoint, and a base Twist. Servos map directly onto the arm
        and head modules; no conversion or polarity is applied here.
        """
        self.last_robot_cmd_time = self.get_clock().now()

        # Arms: 6 logical joints each, straight through (firmware expands the
        # derived shoulder-pitch servo and applies polarity).
        left_arm = ArmCmd()
        left_arm.joint_cmd = list(msg.servo_cmd[LEFT_ARM_SLICE])
        self.left_arm_cmd = left_arm

        right_arm = ArmCmd()
        right_arm.joint_cmd = list(msg.servo_cmd[RIGHT_ARM_SLICE])
        self.right_arm_cmd = right_arm

        # Head: 3 servos + eye LED PWM
        head = HeadCmd()
        head.servos = list(msg.servo_cmd[HEAD_SLICE])
        # Both RobotLowCmd.eye_pwm and HeadCmd.eye_pwm are uint16[2] (duty 0..4095)
        head.eye_pwm = list(msg.eye_pwm)
        self.head_cmd = head

        # Back/spine actuator setpoint (position/velocity/acceleration).
        self.back_cmd = msg.back_cmd

        # Base velocity: forward the Twist; the mecanum board does the kinematics.
        self.drive_cmd = msg.cmd_vel

        self.get_logger().debug('Received robot command')

    # ========================================================================
    # Publishing Methods
    # ========================================================================

    def publish_module_commands(self) -> None:
        """Forward the latest module commands, with a freshness failsafe."""
        if not self._is_command_fresh():
            return

        self.left_arm_pub.publish(self.left_arm_cmd)
        self.right_arm_pub.publish(self.right_arm_cmd)
        self.head_pub.publish(self.head_cmd)
        self.back_pub.publish(self.back_cmd)
        self.drive_pub.publish(self.drive_cmd)
        self.get_logger().debug('Published module commands')

    # ========================================================================
    # Helper Methods
    # ========================================================================

    def _is_command_fresh(self) -> bool:
        """Return True if a recent RobotLowCmd is available to forward."""
        if self.left_arm_cmd is None or self.right_arm_cmd is None or self.head_cmd is None:
            return False
        if self.last_robot_cmd_time is None:
            return False

        time_since_last_cmd = (self.get_clock().now() - self.last_robot_cmd_time).nanoseconds / 1e9
        if time_since_last_cmd > CMD_TIMEOUT_SEC:
            current_time = self.get_clock().now()
            if self.last_warn_time is None or \
                    (current_time - self.last_warn_time).nanoseconds / 1e9 >= WARNING_THROTTLE_SEC:
                self.get_logger().warn(
                    f'No robot command received for {time_since_last_cmd:.3f}s - '
                    f'NOT forwarding module commands')
                self.last_warn_time = current_time
            return False

        return True


# ============================================================================
# Main Entry Point
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MasterLowCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
