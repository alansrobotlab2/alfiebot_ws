import math
from typing import Dict, List, Optional
import rclpy
from rclpy.node import Node
from alfie_msgs.msg import (RobotLowState, ArmState, HeadState, BackState, JetsonState,
                            MecanumState)
from alfie_msgs.msg import ServoState, SpeechRequest
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from .watchdog_checks import (create_health_checks, HealthCheck,
                              SERVO_STATUS_OVERLOAD, SERVO_STATUS_TEMPERATURE)


# Auto-estop: trip a protective stop when the actuator firmware reports a servo
# fault. Critical temperature (deg C) beyond which we also trip, independent of
# the firmware's own temperature status bit, and the lower temperature that
# logs a warning first.
AUTO_ESTOP_CRITICAL_TEMP_C = 65.0
AUTO_ESTOP_WARN_TEMP_C = 60.0

# Glitch rejection for the auto-estop criteria (see _check_servo_faults).
# Readings arrive at ~50 Hz, so 10 samples is ~0.2 s of history before the
# temperature trigger arms, and 3 strikes is ~60 ms of a persistent status bit.
AUTO_ESTOP_MIN_TEMP_SAMPLES = 10
AUTO_ESTOP_STATUS_STRIKES = 3
# Overload logs a warning after AUTO_ESTOP_OVERLOAD_WARN_SEC and trips
# only after AUTO_ESTOP_OVERLOAD_SEC. A shoulder lifting under VR can flag
# overload for a moment alongside a supply sag (2026-10-18: R shoulder pitch at
# 10.6 V), and a loaded elbow tripped a 0.5 s limit during teleop (2026-10-25);
# the warning gives the operator time to back off before a genuine stall or
# jam trips the stop.
AUTO_ESTOP_OVERLOAD_WARN_SEC = 0.5
AUTO_ESTOP_OVERLOAD_SEC = 3.0
# Grippers only warn on overload, never trip: a gripper closed on an object is
# a stalled servo by design (2026-10-23: R gripper at 3 A / 9.2 V while
# squeezing tripped twice), so a held grasp would otherwise e-stop. The
# temperature trip still protects a gripper left squeezing too long.
GRIPPER_SERVO_INDEX = 5  # in each arm module's servo list
# The filtered temperature must stay at/above warn this long (s) before
# warning, and at/above critical this long before tripping. Servo temperature
# feedback is transiently unreliable for reasons never pinned down: bursts of
# bogus readings long enough to beat the trimmed mean (2026-10-23: R shoulder
# yaw read 87 C at a real 36 C; 2026-08-02: 77-105 C bursts beat a 2 s hold).
# Thermal mass means a genuine overtemp lasts minutes, so this costs nothing
# protective.
AUTO_ESTOP_TEMP_WARN_SEC = 2.0
AUTO_ESTOP_TEMP_SEC = 10.0


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

MODULE_JOINT_NAMES = {
    'left_arm': LEFT_ARM_JOINT_NAMES,
    'right_arm': RIGHT_ARM_JOINT_NAMES,
    'head': HEAD_JOINT_NAMES,
}


def _spoken_joint(module: str, index: int) -> str:
    """'right_shoulder_pitch_joint' -> 'right shoulder pitch' for TTS."""
    names = MODULE_JOINT_NAMES.get(module, [])
    if index >= len(names):
        return f'{module.replace("_", " ")} servo {index}'
    name = names[index].removesuffix('_joint').replace('_active', '')
    return name.replace('_', ' ')


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
      - drive base -> low/mecanumstate        (MecanumState, fused wheel odom + board temp)
      - jetson SBC -> low/jetsonstate         (JetsonState)

    This node subscribes to all of them and republishes the aggregate. The
    module firmware already converts servo feedback to SI units and applies
    per-servo polarity, so no unit conversion happens here.

    The drive board's compact MecanumState is also republished as
    nav_msgs/Odometry on low/odom (zero covariance; frame odom -> base_link) for
    odom_tf_broadcaster / odom_covariance_relay.
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
        self.current_cmd_vel: Optional[Twist] = None
        self.command_cmd_vel: Optional[Twist] = None
        self.jetson_state: Optional[JetsonState] = None

        # Initialize watchdog health checks
        self.health_checks: Dict[str, HealthCheck] = create_health_checks()

        # ---- Auto-estop -----------------------------------------------------
        # Publish `estop` (latched) to the command_mux when the actuator firmware
        # reports a servo fault (overload / overtemperature). Rising-edge only;
        # the mux latch persists until a deliberate estop_reset.
        self.declare_parameter('auto_estop_enabled', True)
        self.declare_parameter('auto_estop_critical_temp_c', AUTO_ESTOP_CRITICAL_TEMP_C)
        self.declare_parameter('auto_estop_warn_temp_c', AUTO_ESTOP_WARN_TEMP_C)
        self.declare_parameter('auto_estop_overload_warn_sec', AUTO_ESTOP_OVERLOAD_WARN_SEC)
        self.declare_parameter('auto_estop_overload_sec', AUTO_ESTOP_OVERLOAD_SEC)
        self.declare_parameter('auto_estop_temp_warn_sec', AUTO_ESTOP_TEMP_WARN_SEC)
        self.declare_parameter('auto_estop_temp_sec', AUTO_ESTOP_TEMP_SEC)
        self.auto_estop_enabled = self.get_parameter('auto_estop_enabled').value
        self.auto_estop_critical_temp = self.get_parameter('auto_estop_critical_temp_c').value
        self.auto_estop_warn_temp = self.get_parameter('auto_estop_warn_temp_c').value
        self.auto_estop_overload_warn_sec = float(
            self.get_parameter('auto_estop_overload_warn_sec').value)
        self.auto_estop_overload_sec = float(self.get_parameter('auto_estop_overload_sec').value)
        self.auto_estop_temp_warn_sec = float(self.get_parameter('auto_estop_temp_warn_sec').value)
        self.auto_estop_temp_sec = float(self.get_parameter('auto_estop_temp_sec').value)
        self._servo_fault_active = False       # aggregate rising-edge latch
        self._module_faults = {'left_arm': False, 'right_arm': False, 'head': False}
        # Per-module {servo_index: consecutive messages with a fault status bit}
        self._status_strikes: Dict[str, Dict[int, int]] = {
            'left_arm': {}, 'right_arm': {}, 'head': {}}
        # Per-module {servo_index: time the overload bit was first seen (s)}
        self._overload_since: Dict[str, Dict[int, float]] = {
            'left_arm': {}, 'right_arm': {}, 'head': {}}
        # Per-module servo indices already warned for their current overload
        self._overload_warned: Dict[str, set] = {
            'left_arm': set(), 'right_arm': set(), 'head': set()}
        # Per-module {servo_index: time the filtered temp first reached warn (s)}
        self._warm_since: Dict[str, Dict[int, float]] = {
            'left_arm': {}, 'right_arm': {}, 'head': {}}
        # Per-module servo indices already warned for their current warm spell
        self._temp_warned: Dict[str, set] = {
            'left_arm': set(), 'right_arm': set(), 'head': set()}
        # Per-module {servo_index: time the filtered temp first reached critical (s)}
        self._hot_since: Dict[str, Dict[int, float]] = {
            'left_arm': {}, 'right_arm': {}, 'head': {}}
        qos_latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.estop_pub = self.create_publisher(Empty, 'estop', qos_latched)

        # ---- E-stop announcements -------------------------------------------
        # Speak every engage/clear edge of the mux latch (manual or automatic),
        # with the fault reason when this node is the one that tripped it.
        self._estopped: Optional[bool] = None  # None until the first state arrives
        self._estop_reason: Optional[str] = None
        self.speech_pub = self.create_publisher(
            SpeechRequest, 'speechrequest',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Bool, 'estop_state', self.estop_state_callback, qos_latched)

        # ---- Subscriptions --------------------------------------------------
        self.left_arm_sub = self.create_subscription(
            ArmState, 'low/left_arm/armstate', self.left_arm_callback, qos_best_effort)
        self.right_arm_sub = self.create_subscription(
            ArmState, 'low/right_arm/armstate', self.right_arm_callback, qos_best_effort)
        self.head_sub = self.create_subscription(
            HeadState, 'low/headstate', self.head_callback, qos_best_effort)
        self.back_sub = self.create_subscription(
            BackState, 'low/backstate', self.back_callback, qos_best_effort)
        self.mecanum_sub = self.create_subscription(
            MecanumState, 'low/mecanumstate', self.mecanum_callback, qos_best_effort)
        # Base velocity command, raw and rate-limited, from command_mux
        self.current_cmd_vel_sub = self.create_subscription(
            Twist, 'low/current_cmd_vel', self.current_cmd_vel_callback, qos_best_effort)
        self.command_cmd_vel_sub = self.create_subscription(
            Twist, 'low/command_cmd_vel', self.command_cmd_vel_callback, qos_best_effort)
        self.jetson_sub = self.create_subscription(
            JetsonState, 'low/jetsonstate', self.jetson_callback, qos_best_effort)

        # ---- Publishers -----------------------------------------------------
        self.robot_state_pub = self.create_publisher(
            RobotLowState, 'robotlowstate', qos_best_effort)
        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', qos_best_effort)
        # RELIABLE, like the firmware's former direct odom publisher, so both
        # reliable and best-effort subscribers match.
        self.odom_pub = self.create_publisher(
            Odometry, 'low/odom', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
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

    def estop_state_callback(self, msg: Bool) -> None:
        engaged = bool(msg.data)
        first = self._estopped is None
        changed = engaged != self._estopped
        self._estopped = engaged
        if first or not changed:
            return  # don't announce the latched state we start up into
        if engaged:
            reason = self._estop_reason
            self._say(f'E-stop engaged. {reason}.' if reason else 'E-stop engaged.')
        else:
            self._say('E-stop cleared.')
        self._estop_reason = None

    def _say(self, text: str) -> None:
        msg = SpeechRequest()
        msg.text = text
        self.speech_pub.publish(msg)

    def left_arm_callback(self, msg: ArmState) -> None:
        self.left_arm_state = msg
        self.health_checks['left_arm_rate'].update()
        self.health_checks['left_arm_servos'].update(msg.joint_state)
        self.health_checks['left_arm_voltage'].update(msg.joint_state)
        self.health_checks['left_arm_board_temp'].update(float(msg.board_temp))
        self._check_servo_faults('left_arm', msg.joint_state)

    def right_arm_callback(self, msg: ArmState) -> None:
        self.right_arm_state = msg
        self.health_checks['right_arm_rate'].update()
        self.health_checks['right_arm_servos'].update(msg.joint_state)
        self.health_checks['right_arm_voltage'].update(msg.joint_state)
        self.health_checks['right_arm_board_temp'].update(float(msg.board_temp))
        self._check_servo_faults('right_arm', msg.joint_state)

    def head_callback(self, msg: HeadState) -> None:
        self.head_state = msg
        self.health_checks['head_rate'].update()
        self.health_checks['head_servos'].update(msg.servos)
        self.health_checks['head_voltage'].update(msg.servos)
        self.health_checks['head_board_temp'].update(float(msg.board_temp))
        self._check_servo_faults('head', msg.servos)

    def back_callback(self, msg: BackState) -> None:
        self.back_state = msg
        self.health_checks['back_rate'].update()
        self.health_checks['back_board_temp'].update(float(msg.board_temp))

    def mecanum_callback(self, msg: MecanumState) -> None:
        self.health_checks['mecanum_board_temp'].update(float(msg.board_temp))

        odom = Odometry()
        # The board stamps with its own uptime; restamp with host time so TF
        # consumers get a usable stamp.
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = msg.header.frame_id or 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(msg.x)
        odom.pose.pose.position.y = float(msg.y)
        odom.pose.pose.orientation.z = math.sin(msg.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(msg.yaw * 0.5)
        odom.twist.twist.linear.x = float(msg.vx)
        odom.twist.twist.linear.y = float(msg.vy)
        odom.twist.twist.angular.z = float(msg.wz)
        self.odom = odom
        self.odom_pub.publish(odom)

    def current_cmd_vel_callback(self, msg: Twist) -> None:
        self.current_cmd_vel = msg

    def command_cmd_vel_callback(self, msg: Twist) -> None:
        self.command_cmd_vel = msg

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

        # Base velocity command from command_mux: rate-limited (sent) and raw
        robot_state.current_cmd_vel = self.current_cmd_vel if self.current_cmd_vel is not None else Twist()
        robot_state.command_cmd_vel = self.command_cmd_vel if self.command_cmd_vel is not None else Twist()

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

        Both criteria are filtered rather than read off a single sample. The
        servo bus is a noisy half-duplex line and a corrupted payload byte
        looks exactly like a real fault for one message: temperature uses the
        ServoMonitor's trimmed mean over its rolling window, and the status
        overtemp bit must persist for AUTO_ESTOP_STATUS_STRIKES consecutive
        messages, the overload bit for auto_estop_overload_sec (with a
        logged warning at auto_estop_overload_warn_sec; grippers only warn),
        and the filtered temperature at/above critical for auto_estop_temp_sec
        (with a logged warning once at/above warn for auto_estop_temp_warn_sec).
        Servo thermal mass means a genuine overtemp lasts many seconds, so the
        sub-second delay this adds costs nothing protective.
        """
        if not self.auto_estop_enabled:
            return

        monitor = self.health_checks.get(f'{module}_servos')
        strikes = self._status_strikes[module]
        overload_since = self._overload_since[module]
        overload_warned = self._overload_warned[module]
        warm_since = self._warm_since[module]
        temp_warned = self._temp_warned[module]
        hot_since = self._hot_since[module]
        now = self.get_clock().now().nanoseconds / 1e9

        faults = []
        spoken = []
        for i, servo in enumerate(servos):
            status = int(servo.servo_status)
            bits = []
            overloaded = False
            if status & SERVO_STATUS_OVERLOAD:
                since = overload_since.setdefault(i, now)
                gripper = module != 'head' and i == GRIPPER_SERVO_INDEX
                if not gripper and now - since >= self.auto_estop_overload_sec:
                    bits.append(f'overload ({now - since:.2f}s)')
                    overloaded = True
                elif (now - since >= self.auto_estop_overload_warn_sec
                        and i not in overload_warned and not self._estopped):
                    overload_warned.add(i)
                    limit = ('gripper, no e-stop' if gripper else
                             f'e-stop at {self.auto_estop_overload_sec:.1f}s')
                    self.get_logger().warn(
                        f'Servo overload warning: {module}[{i}] overloaded for '
                        f'{now - since:.2f}s ({limit})')
            else:
                overload_since.pop(i, None)
                overload_warned.discard(i)

            overtemp = bool(status & SERVO_STATUS_TEMPERATURE)
            strikes[i] = strikes.get(i, 0) + 1 if overtemp else 0
            if overtemp and strikes[i] >= AUTO_ESTOP_STATUS_STRIKES:
                bits.append('firmware-overtemp')

            if bits:
                faults.append(f'{module}[{i}] {"+".join(bits)}')
                spoken.append(f'{_spoken_joint(module, i)} {"overload" if overloaded else "overheating"}')

            temp = (monitor.filtered_temp(i, AUTO_ESTOP_MIN_TEMP_SAMPLES)
                    if monitor is not None else None)
            if temp is not None and temp >= self.auto_estop_warn_temp:
                since = warm_since.setdefault(i, now)
                if (now - since >= self.auto_estop_temp_warn_sec
                        and i not in temp_warned and not self._estopped):
                    temp_warned.add(i)
                    self.get_logger().warn(
                        f'Servo temperature warning: {module}[{i}] at {temp:.0f}C '
                        f'(e-stop at {self.auto_estop_critical_temp:.0f}C for '
                        f'{self.auto_estop_temp_sec:.0f}s)')
            else:
                warm_since.pop(i, None)
                temp_warned.discard(i)
            if temp is not None and temp >= self.auto_estop_critical_temp:
                since = hot_since.setdefault(i, now)
                if now - since >= self.auto_estop_temp_sec:
                    faults.append(f'{module}[{i}] temp={temp:.0f}C ({now - since:.2f}s)')
                    spoken.append(f'{_spoken_joint(module, i)} at {temp:.0f} degrees')
            else:
                hot_since.pop(i, None)

        # Track this module's fault state; edge-detect on the aggregate across all
        # modules so a healthy module's callback doesn't clear another's fault
        # (which would otherwise re-fire estop every message).
        self._module_faults[module] = bool(faults)
        any_fault = any(self._module_faults.values())

        if any_fault and not self._servo_fault_active:
            self._servo_fault_active = True
            if not self._estopped:  # reason is spoken on the engage edge
                self._estop_reason = ', '.join(dict.fromkeys(spoken))
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
