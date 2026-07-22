from typing import Dict, List, Optional
import rclpy
from rclpy.node import Node
from alfie_msgs.msg import RobotLowState, ArmState, HeadState, BackState, JetsonState
from alfie_msgs.msg import ServoState
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from .watchdog_checks import (create_health_checks, HealthCheck,
                              SERVO_STATUS_OVERLOAD, SERVO_STATUS_TEMPERATURE)


# Auto-estop: trip a protective stop when the actuator firmware reports a servo
# fault. Critical temperature (deg C) beyond which we also trip, independent of
# the firmware's own temperature status bit.
AUTO_ESTOP_CRITICAL_TEMP_C = 65.0


# ============================================================================
# Constants and Configuration
# ============================================================================

# Publishing configuration. The gen2 firmware modules stream their state at
# ~50 Hz, so there is nothing to gain from republishing the consolidated state
# faster than that.
PUBLISH_RATE_HZ = 50
PUBLISH_PERIOD_SEC = 1.0 / PUBLISH_RATE_HZ

# Watchdog check rate (slower than publish rate)
WATCHDOG_RATE_HZ = 1.0  # Check once per second
WATCHDOG_PERIOD_SEC = 1.0 / WATCHDOG_RATE_HZ

# Servo layout in RobotLowState.servo_state (0-5 left arm, 6-11 right arm, 12-14 head)
NUM_ARM_SERVOS = 6   # per arm, derived shoulder-pitch servo already merged by firmware
NUM_HEAD_SERVOS = 3
TOTAL_SERVOS = 2 * NUM_ARM_SERVOS + NUM_HEAD_SERVOS  # 15

# Frame id for the republished BNO085 IMU (matches imu_bridge / URDF imu_link)
IMU_FRAME_ID = 'imu_link'

# ----------------------------------------------------------------------------
# Joint names (must match the URDF). Servo order matches ArmCmd / HeadCmd, which
# is the same order the module firmware reports feedback in.
# ----------------------------------------------------------------------------
LEFT_ARM_JOINT_NAMES = [
    'left_shoulder_yaw_joint',    # 0
    'left_shoulder_pitch_joint',  # 1
    'left_elbow_pitch_joint',     # 2
    'left_wrist_pitch_joint',     # 3
    'left_wrist_roll_joint',      # 4
    'left_gripper_active_joint',  # 5
]
RIGHT_ARM_JOINT_NAMES = [
    'right_shoulder_yaw_joint',    # 0
    'right_shoulder_pitch_joint',  # 1
    'right_elbow_pitch_joint',     # 2
    'right_wrist_pitch_joint',     # 3
    'right_wrist_roll_joint',      # 4
    'right_gripper_active_joint',  # 5
]
HEAD_JOINT_NAMES = [
    'head_yaw_joint',    # 0 (pan, bus 1)
    'head_pitch_joint',  # 1 (tilt, bus 2)
    'head_roll_joint',   # 2 (bus 3)
]

# Full servo-joint list in RobotLowState.servo_state order
SERVO_JOINT_NAMES = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES + HEAD_JOINT_NAMES


# ============================================================================
# MasterLowStatusNode Class
# ============================================================================

class MasterLowStatusNode(Node):
    """Consolidates the per-module firmware state topics into a single
    RobotLowState (and a JointState for TF/visualization).

    Gen2 hardware splits actuation across independent Pico driver boards, each
    publishing its own state:
      - left arm   -> low/left_arm/armstate   (ArmState, SI + polarity applied)
      - right arm  -> low/right_arm/armstate  (ArmState, SI + polarity applied)
      - head       -> low/headstate           (HeadState, SI + polarity applied)
      - back/spine -> low/backstate           (BackState, actuator + BNO085 IMU)
      - drive base -> low/odom                (nav_msgs/Odometry, fused wheel odom)
      - jetson SBC -> low/jetsonstate         (JetsonState)

    This node subscribes to all of them and republishes the aggregate. The
    module firmware already converts servo feedback to SI units and applies
    per-servo polarity, so no unit conversion happens here.
    """

    def __init__(self):
        super().__init__('master_low_status_node')
        # Use BEST_EFFORT QoS for all communication (matches the micro-ROS publishers)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Latest state from each firmware module
        self.left_arm_state: Optional[ArmState] = None
        self.right_arm_state: Optional[ArmState] = None
        self.head_state: Optional[HeadState] = None
        self.back_state: Optional[BackState] = None
        self.odom: Optional[Odometry] = None
        self.jetson_state: Optional[JetsonState] = None

        # Initialize watchdog health checks
        self.health_checks: Dict[str, HealthCheck] = create_health_checks()

        # ---- Auto-estop -----------------------------------------------------
        # Publish `estop` (latched) to the command_mux when the actuator firmware
        # reports a servo fault (overload / overtemperature). Rising-edge only;
        # the mux latch persists until a deliberate estop_reset.
        self.declare_parameter('auto_estop_enabled', True)
        self.declare_parameter('auto_estop_critical_temp_c', AUTO_ESTOP_CRITICAL_TEMP_C)
        self.auto_estop_enabled = self.get_parameter('auto_estop_enabled').value
        self.auto_estop_critical_temp = self.get_parameter('auto_estop_critical_temp_c').value
        self._servo_fault_active = False       # aggregate rising-edge latch
        self._module_faults = {'left_arm': False, 'right_arm': False, 'head': False}
        qos_latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.estop_pub = self.create_publisher(Empty, 'estop', qos_latched)

        # ---- Subscriptions --------------------------------------------------
        self.left_arm_sub = self.create_subscription(
            ArmState, 'low/left_arm/armstate', self.left_arm_callback, qos_best_effort)
        self.right_arm_sub = self.create_subscription(
            ArmState, 'low/right_arm/armstate', self.right_arm_callback, qos_best_effort)
        self.head_sub = self.create_subscription(
            HeadState, 'low/headstate', self.head_callback, qos_best_effort)
        self.back_sub = self.create_subscription(
            BackState, 'low/backstate', self.back_callback, qos_best_effort)
        self.odom_sub = self.create_subscription(
            Odometry, 'low/odom', self.odom_callback, qos_best_effort)
        self.jetson_sub = self.create_subscription(
            JetsonState, 'low/jetsonstate', self.jetson_callback, qos_best_effort)

        # ---- Publishers -----------------------------------------------------
        self.robot_state_pub = self.create_publisher(
            RobotLowState, 'robotlowstate', qos_best_effort)
        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', qos_best_effort)
        # Neck-servo-0 (head pan) power heartbeat. Published once per 50 Hz cycle
        # ONLY while the servo has torque enabled, so the back board's BNO085 can
        # decouple its compass while the nearby motor current corrupts the mag.
        # Resolves to /alfie/low/neck_power to match the Pico subscription.
        self.neck_power_pub = self.create_publisher(
            Empty, 'low/neck_power', qos_best_effort)

        # ---- Timers ---------------------------------------------------------
        self.state_timer = self.create_timer(PUBLISH_PERIOD_SEC, self.publish_robot_state)
        self.watchdog_timer = self.create_timer(WATCHDOG_PERIOD_SEC, self.run_health_checks)

        self.get_logger().info(
            f'Master Low Status Node started - publishing at {PUBLISH_RATE_HZ}Hz, '
            f'watchdog at {WATCHDOG_RATE_HZ}Hz')

    # ========================================================================
    # Callback Methods
    # ========================================================================

    def left_arm_callback(self, msg: ArmState) -> None:
        self.left_arm_state = msg
        self.health_checks['left_arm_rate'].update()
        self.health_checks['left_arm_servos'].update(msg.joint_state)
        self.health_checks['left_arm_voltage'].update(msg.joint_state)
        self._check_servo_faults('left_arm', msg.joint_state)

    def right_arm_callback(self, msg: ArmState) -> None:
        self.right_arm_state = msg
        self.health_checks['right_arm_rate'].update()
        self.health_checks['right_arm_servos'].update(msg.joint_state)
        self.health_checks['right_arm_voltage'].update(msg.joint_state)
        self._check_servo_faults('right_arm', msg.joint_state)

    def head_callback(self, msg: HeadState) -> None:
        self.head_state = msg
        self.health_checks['head_rate'].update()
        self.health_checks['head_servos'].update(msg.servos)
        self.health_checks['head_voltage'].update(msg.servos)
        self._check_servo_faults('head', msg.servos)

    def back_callback(self, msg: BackState) -> None:
        self.back_state = msg
        self.health_checks['back_rate'].update()
        self.health_checks['back_board_temp'].update(float(msg.board_temp))

    def odom_callback(self, msg: Odometry) -> None:
        self.odom = msg

    def jetson_callback(self, msg: JetsonState) -> None:
        self.jetson_state = msg
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
            ssid=msg.wifi_ssid)

    # ========================================================================
    # Publishing Methods
    # ========================================================================

    def publish_robot_state(self) -> None:
        """Combine the per-module states and publish the consolidated state."""
        # Neck-servo-0 power heartbeat. Emitted independently of arm availability
        # so the compass-decouple signal keeps flowing whenever the head reports
        # torque on servo 0 (pan). Absence for >100 ms means powered off.
        if self.head_state is not None and self.head_state.servos[0].enabled:
            self.neck_power_pub.publish(Empty())

        # The servo state (15 joints) needs all three actuator modules; without
        # them there is nothing meaningful to publish.
        if self.left_arm_state is None or self.right_arm_state is None or self.head_state is None:
            self.get_logger().debug('Waiting for both arms and head...')
            return

        robot_state = RobotLowState()
        robot_state.header.stamp = self.get_clock().now().to_msg()
        robot_state.header.frame_id = 'base_link'

        # IMU (BNO085 on the back board), converted to standard sensor_msgs/Imu
        robot_state.imu = self._build_imu()

        # Back state (linear actuator + IMU), pass-through
        robot_state.back_state = self.back_state if self.back_state is not None else BackState()

        # Eye LED PWM duty from the head module (0..4095). head_state.eye_state is
        # a numpy uint16 array; convert to Python ints so the RobotLowState setter's
        # isinstance(v, int) check passes (list() alone leaves numpy.uint16 scalars).
        robot_state.eye_state = [int(v) for v in self.head_state.eye_state]

        # Measured base velocity from fused wheel odometry
        robot_state.cmd_vel = self.odom.twist.twist if self.odom is not None else Twist()

        # 15 servos: left arm (0-5), right arm (6-11), head (12-14)
        robot_state.servo_state = self._build_servo_states()

        self.robot_state_pub.publish(robot_state)

        # JointState for robot_state_publisher (TF) and visualization
        self.joint_state_pub.publish(self._build_joint_state(robot_state))

    # ========================================================================
    # Helper Methods for Building State Messages
    # ========================================================================

    def _build_imu(self) -> Imu:
        """Convert the back board's BNO085 telemetry into a sensor_msgs/Imu.

        Returns a zeroed placeholder (with a valid stamp/frame) if the back
        state has not arrived yet.
        """
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = IMU_FRAME_ID

        if self.back_state is None:
            return imu

        gdb_imu = self.back_state.imu
        imu.orientation.x = gdb_imu.orientation_x
        imu.orientation.y = gdb_imu.orientation_y
        imu.orientation.z = gdb_imu.orientation_z
        imu.orientation.w = gdb_imu.orientation_w
        imu.angular_velocity.x = gdb_imu.angular_velocity_x
        imu.angular_velocity.y = gdb_imu.angular_velocity_y
        imu.angular_velocity.z = gdb_imu.angular_velocity_z
        imu.linear_acceleration.x = gdb_imu.linear_acceleration_x
        imu.linear_acceleration.y = gdb_imu.linear_acceleration_y
        imu.linear_acceleration.z = gdb_imu.linear_acceleration_z
        # Unknown covariance
        imu.orientation_covariance = [0.0] * 9
        imu.angular_velocity_covariance = [0.0] * 9
        imu.linear_acceleration_covariance = [0.0] * 9
        # Compass decoupled by the back board (see imu_bridge): the quaternion is
        # the compass-free game rotation vector, so mark orientation unavailable
        # to fusion via the robot_localization convention (covariance[0] = -1.0).
        if not gdb_imu.orientation_reliable:
            imu.orientation_covariance[0] = -1.0
        return imu

    def _build_servo_states(self) -> List[ServoState]:
        """Assemble the 15-servo array from the arm and head modules.

        The module firmware already reports SI units with per-servo polarity
        applied, so the ServoState messages are copied straight through.
        """
        return (
            list(self.left_arm_state.joint_state) +
            list(self.right_arm_state.joint_state) +
            list(self.head_state.servos)
        )

    def _build_joint_state(self, robot_state: RobotLowState) -> JointState:
        """Build a JointState from the consolidated state for TF/visualization.

        Publishes the 15 driven servo joints, the two synthesized passive
        gripper joints, and the prismatic back joint. Mecanum wheel joints are
        not published: gen2 exposes only fused odometry, not per-wheel encoders.
        """
        joint_state = JointState()
        joint_state.header.stamp = robot_state.header.stamp
        joint_state.header.frame_id = ''

        names: List[str] = []
        positions: List[float] = []

        left_gripper_active_pos = None
        right_gripper_active_pos = None

        for joint_name, servo in zip(SERVO_JOINT_NAMES, robot_state.servo_state):
            names.append(joint_name)
            positions.append(servo.current_location)
            if joint_name == 'left_gripper_active_joint':
                left_gripper_active_pos = servo.current_location
            elif joint_name == 'right_gripper_active_joint':
                right_gripper_active_pos = servo.current_location

        # Synthesized passive gripper joints (negated from active)
        if left_gripper_active_pos is not None:
            names.append('left_gripper_passive_joint')
            positions.append(-left_gripper_active_pos)
        if right_gripper_active_pos is not None:
            names.append('right_gripper_passive_joint')
            positions.append(-right_gripper_active_pos)

        # Back joint (prismatic), current_position is in meters
        if self.back_state is not None:
            names.append('back_joint')
            positions.append(robot_state.back_state.current_position)

        joint_state.name = names
        joint_state.position = positions
        return joint_state

    # ========================================================================
    # Watchdog Health Check Runner
    # ========================================================================

    def run_health_checks(self) -> None:
        """Run all registered health checks and log errors."""
        for check_name, check in self.health_checks.items():
            error_msg = check.check()
            if error_msg:
                self.get_logger().error(error_msg)

    # ========================================================================
    # Auto-estop on servo faults
    # ========================================================================

    def _check_servo_faults(self, module: str, servos: List[ServoState]) -> None:
        """Trip a protective stop when the firmware reports a servo fault.

        Fires `estop` to the command_mux on the rising edge of any overload /
        overtemperature condition. Checked on every state message (~50 Hz) for
        fast response; the mux latch then holds until a deliberate estop_reset.
        """
        if not self.auto_estop_enabled:
            return

        faults = []
        for i, servo in enumerate(servos):
            status = int(servo.servo_status)
            if status & SERVO_STATUS_OVERLOAD:
                faults.append(f'{module}[{i}] overload')
            if status & SERVO_STATUS_TEMPERATURE:
                faults.append(f'{module}[{i}] firmware-overtemp')
            if float(servo.current_temperature) >= self.auto_estop_critical_temp:
                faults.append(f'{module}[{i}] temp={servo.current_temperature:.0f}C')

        # Track this module's fault state; edge-detect on the aggregate across all
        # modules so a healthy module's callback doesn't clear another's fault
        # (which would otherwise re-fire estop every message).
        self._module_faults[module] = bool(faults)
        any_fault = any(self._module_faults.values())

        if any_fault and not self._servo_fault_active:
            self._servo_fault_active = True
            self.estop_pub.publish(Empty())
            self.get_logger().error(
                f'AUTO E-STOP: servo fault(s) -> {", ".join(faults)}')
        elif not any_fault:
            # All modules healthy again: re-arm the trigger. The mux stays latched
            # regardless until estop_reset; this only re-arms our rising edge.
            self._servo_fault_active = False


# ============================================================================
# Main Entry Point
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MasterLowStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
