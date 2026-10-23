#!/usr/bin/env python3
"""
Alfie VR teleop - ROS2 node that turns VR headset/controller input into
per-subsystem command_mux inputs (cmd/<subsystem>/vr).

VR only claims a subsystem while it is actually being driven, so the mux can
hand control to other sources (GR00T, idle, joy) the rest of the time:
  head      claimed while the headset link is fresh; released on link loss
  base      claimed while a controller link is fresh; on link loss sends zero
            velocity for base_stop_sec, then releases
  left/right arm
            claimed on first grip (or Y reset), seeded from the measured pose;
            held at the last target on link loss (hold_timeout)
  back      claimed on first A/B press, seeded from the measured height; each
            press steps to the next BACK_PRESETS height; held on link loss
            (hold_timeout)
Published joint targets are rate-limited (max_joint_speed), so no claim, reset
or IK jump can snap a joint.
"""

import asyncio
import math
import os
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from alfie_msgs.msg import RobotLowState, ServoCmd, BackCmd, ArmCmd, HeadCmd
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R

from alfie_vr.vr_monitor import VRMonitor
from alfie_vr.kinematics import AlfieArmKinematics, SimpleTeleopArm, ARM_JOINTS
from alfie_vr.arm_debug_visualizer import ArmDebugVisualizer, PYGAME_AVAILABLE

NUM_SERVOS = 15          # 0-5 left arm, 6-11 right arm, 12-14 head
ARM_OFFSET = {'left': 0, 'right': 6}
HEAD_YAW, HEAD_PITCH, HEAD_ROLL = 12, 13, 14

BACK_MIN = 0.000
BACK_MAX = 0.350         # picobackdriver ACTUATOR_MAX_POSITION (measured stroke)
BACK_PRESETS = (0.0, 0.15, 0.25, 0.35)   # heights (m) A/B step through
BACK_PRESET_TOL = 0.01   # within this of a preset counts as at it

# Headset yaw beyond this (deg) is ambiguous (operator turned around); hold yaw.
HEAD_YAW_WRAP_GUARD_DEG = 150.0


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _next_back_preset(current, up):
    """Next BACK_PRESETS height above (up) or below current; stays put at the ends."""
    if up:
        above = [p for p in BACK_PRESETS if p > current + BACK_PRESET_TOL]
        return min(above) if above else current
    below = [p for p in BACK_PRESETS if p < current - BACK_PRESET_TOL]
    return max(below) if below else current


def _button(buttons, name):
    return bool(buttons.get(name, False) or buttons.get(name.upper(), False))


class AlfieTeleopVRNode(Node):
    """ROS2 node that drives Alfie from a VR headset and controllers."""

    def __init__(self):
        super().__init__('alfiebot_teleop_vr_node')

        def param(name, default, desc):
            self.declare_parameter(name, default, ParameterDescriptor(description=desc))
            return self.get_parameter(name).value

        self.link_timeout = param(
            'link_timeout', 0.3, 'Seconds without data before a VR device counts as lost')
        self.hold_timeout = param(
            'hold_timeout', -1.0,
            'Seconds to keep holding arms/back after link loss before releasing them '
            '(arms go limp when released). < 0 = hold until a new VR session takes over')
        self.base_stop_sec = param(
            'base_stop_sec', 0.3, 'Seconds of explicit zero velocity sent after link loss')
        self.max_joint_speed = param(
            'max_joint_speed', 4.0, 'Rate limit on published arm/head joint targets (rad/s)')
        self.head_yaw_limit = param('head_yaw_limit', math.pi, 'Head yaw command limit (rad)')
        self.head_pitch_limit = param('head_pitch_limit', 1.0472, 'Head pitch command limit (rad)')
        self.head_roll_limit = param('head_roll_limit', 0.35, 'Head roll command limit (rad)')
        # Head tracking multipliers (headset rotation -> robot head rotation)
        self.head_yaw_multiplier = param('head_yaw_multiplier', 2.0, 'Headset yaw gain')
        self.head_pitch_multiplier = param('head_pitch_multiplier', 2.0, 'Headset pitch gain')
        self.head_roll_multiplier = param('head_roll_multiplier', 2.0, 'Headset roll gain')
        # WebXR is +yaw left, +pitch up, +roll left ear down. Measured live
        # 2026-10-18: servo +yaw turns right, +pitch looks down, roll also mirrored.
        self.head_yaw_sign = param('head_yaw_sign', -1.0, '+1: headset yaw passes through unchanged')
        self.head_pitch_sign = param('head_pitch_sign', -1.0, '+1: headset pitch passes through unchanged')
        self.head_roll_sign = param('head_roll_sign', -1.0, '+1: headset roll passes through unchanged')
        # Joystick velocity scaling (acceleration limiting is done by command_mux)
        self.max_linear_vel = param('max_linear_vel', 0.15, 'Max joystick linear speed (m/s)')
        self.max_angular_vel = param('max_angular_vel', 0.5, 'Max joystick rotation speed (rad/s)')
        self.tunable_params = {
            'link_timeout', 'hold_timeout', 'base_stop_sec', 'max_joint_speed',
            'head_yaw_limit', 'head_pitch_limit', 'head_roll_limit',
            'head_yaw_multiplier', 'head_pitch_multiplier', 'head_roll_multiplier',
            'head_yaw_sign', 'head_pitch_sign', 'head_roll_sign',
            'max_linear_vel', 'max_angular_vel'}
        self.declare_parameter(
            'back_height', 0.350,
            ParameterDescriptor(description='Set to command a back height in meters (0.0 to 0.350)'))
        self.add_on_set_parameters_callback(self._on_parameter_change)

        self.state_callback_group = ReentrantCallbackGroup()
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Per-subsystem command_mux inputs ("vr" source). Eyes are not a VR channel.
        self.left_arm_publisher = self.create_publisher(ArmCmd, '/alfie/cmd/left_arm/vr', qos_best_effort)
        self.right_arm_publisher = self.create_publisher(ArmCmd, '/alfie/cmd/right_arm/vr', qos_best_effort)
        self.head_publisher = self.create_publisher(HeadCmd, '/alfie/cmd/head/vr', qos_best_effort)
        self.back_publisher = self.create_publisher(BackCmd, '/alfie/cmd/back/vr', qos_best_effort)
        self.base_publisher = self.create_publisher(Twist, '/alfie/cmd/base/vr', qos_best_effort)

        self.state_subscriber = self.create_subscription(
            RobotLowState, '/alfie/robotlowstate', self.robot_state_callback,
            qos_best_effort, callback_group=self.state_callback_group)

        # E-stop state (latched, mirrors the mux's estop_state publisher).
        qos_latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.estop_active = False
        self.estop_state_sub = self.create_subscription(
            Bool, '/alfie/estop_state', self.estop_state_callback, qos_latched,
            callback_group=self.state_callback_group)

        self.robot_state = None
        self.robot_state_lock = threading.Lock()
        # Guards everything below that both the control timer and the
        # e-stop/parameter callbacks touch.
        self.cmd_lock = threading.Lock()

        # Desired joint targets (from VR/IK) and the rate-limited targets actually published.
        self.desired = [0.0] * NUM_SERVOS
        self.published = [0.0] * NUM_SERVOS
        self.back_target = None
        self.cmd_vel = Twist()

        # Which subsystems VR currently owns.
        self.claimed = {'left_arm': False, 'right_arm': False, 'head': False, 'back': False, 'base': False}

        self.arms = {'left': None, 'right': None}
        self.kinematics = {'left': None, 'right': None}
        self.grip_active = {'left': False, 'right': False}

        self.headset_connected = False
        self.controllers_connected = False
        self.link_lost_time = None      # monotonic time all VR input went stale
        self.base_stop_until = 0.0
        self.last_release_count = 0
        self.last_tick = None

        # Edge detection
        self.passthrough_mode_active = False
        self.left_x_button_pressed = False
        self.left_y_button_pressed = False
        self.is_recording = False
        self.right_thumbstick_pressed = False
        self.right_a_button_pressed = False
        self.right_b_button_pressed = False

        qos_recording = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.recording_start_pub = self.create_publisher(Bool, '/alfie/recording/start', qos_recording)
        self.recording_stop_pub = self.create_publisher(Bool, '/alfie/recording/stop', qos_recording)

        self.debug_logs = False
        self.debug_visualizer = None
        self.enable_debug_viz = False
        self.latest_goals = {'left': None, 'right': None}

        self.vr_monitor = VRMonitor(debug_logs=self.debug_logs)
        self.vr_thread = None
        self.vr_loop = None

        # Timers start in main() once the first robot state has arrived.
        self.control_timer = None
        self.viz_timer = None

        self.get_logger().info('Alfie Teleop VR Node initialized')
        self.start_vr_monitor()

    def debug_log(self, msg: str):
        """Log a debug message only if debug_logs is enabled."""
        if self.debug_logs:
            self.get_logger().info(msg)

    def _on_parameter_change(self, params):
        """Runtime parameter updates."""
        for param in params:
            if param.name == 'back_height':
                state = self.get_robot_state()
                if state is None:
                    return SetParametersResult(successful=False, reason='no robot state yet')
                with self.cmd_lock:
                    self._claim_back(state)
                    self.back_target = _clamp(float(param.value), BACK_MIN, BACK_MAX)
                self.get_logger().info(f'back_height set to {self.back_target:.3f}')
            elif param.name in self.tunable_params:
                setattr(self, param.name, param.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def start_vr_monitor(self):
        """Run the VR web/websocket servers in a background asyncio thread."""
        def run_vr_loop():
            self.vr_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.vr_loop)
            try:
                self.vr_loop.run_until_complete(self.vr_monitor.start_monitoring())
            except Exception as e:
                # Without the servers there is no VR input: exit so launch respawns us.
                self.get_logger().fatal(f'VR monitor error: {e}; shutting down')
                os.kill(os.getpid(), signal.SIGINT)

        self.vr_thread = threading.Thread(target=run_vr_loop, daemon=True)
        self.vr_thread.start()
        self.get_logger().info('VR monitor started in background thread')

    def robot_state_callback(self, msg: RobotLowState):
        with self.robot_state_lock:
            self.robot_state = msg

    def get_robot_state(self):
        with self.robot_state_lock:
            return self.robot_state

    def initialize_controllers(self):
        """Create arm controllers and start control once robot state is available."""
        robot_state = self.get_robot_state()
        if robot_state is None:
            self.get_logger().error('Cannot initialize arm controllers: no robot state')
            return False

        for side in ('left', 'right'):
            self.kinematics[side] = AlfieArmKinematics()
            self.arms[side] = SimpleTeleopArm(
                robotlowstate=robot_state, kinematics=self.kinematics[side],
                prefix=side, debug_logs=self.debug_logs)
        self._seed_from_state(robot_state, range(NUM_SERVOS))

        self._check_back_calibrated(robot_state)

        if self.enable_debug_viz and PYGAME_AVAILABLE:
            self.debug_visualizer = ArmDebugVisualizer(
                self.kinematics['left'], self.kinematics['right'], prefix="left")
            if self.debug_visualizer.start():
                self.get_logger().info('Debug visualizer started (press L/R/Tab to switch arms)')
            else:
                self.get_logger().warn('Debug visualizer failed to start')
                self.debug_visualizer = None

        self.control_timer = self.create_timer(0.01, self.control_tick)   # 100 Hz
        self.viz_timer = self.create_timer(0.033, self.update_visualization)
        self.get_logger().info('Arm and head controllers initialized')
        return True

    def _check_back_calibrated(self, robot_state):
        # Never calibrate from here: homing drives the back onto its limit switch,
        # only safe with the arms tucked. robot_init does that at bringup.
        if not robot_state.back_state.is_calibrated:
            self.get_logger().warn(
                'Back not calibrated (robot_init homes it at bringup); back commands '
                'will be ignored by the firmware until it is')

    # ------------------------------------------------------------------
    # Claims / seeding (call with cmd_lock held)
    # ------------------------------------------------------------------

    def _seed_from_state(self, state, indices):
        """Set desired and published targets for servo indices to the measured pose."""
        n = len(state.servo_state)
        for i in indices:
            if i < n:
                self.desired[i] = self.published[i] = state.servo_state[i].current_location

    def _claim_arm(self, side, state):
        key = f'{side}_arm'
        if self.claimed[key]:
            return
        off = ARM_OFFSET[side]
        self.arms[side].reseed(state)
        self._seed_from_state(state, range(off, off + 6))
        self.claimed[key] = True
        self.get_logger().info(f'VR claimed {key}')

    def _claim_head(self, state):
        if self.claimed['head']:
            return
        self._seed_from_state(state, (HEAD_YAW, HEAD_PITCH, HEAD_ROLL))
        self.claimed['head'] = True
        self.get_logger().info('VR claimed head')

    def _claim_back(self, state):
        if self.claimed['back']:
            return
        self.back_target = _clamp(state.back_state.current_position, BACK_MIN, BACK_MAX)
        self.claimed['back'] = True
        self.get_logger().info('VR claimed back')

    def _release(self, *keys):
        for key in keys:
            if self.claimed[key]:
                self.claimed[key] = False
                self.get_logger().info(f'VR released {key}')

    def _hold_current_pose(self, state):
        """Re-seed every claimed subsystem to the measured pose (no motion)."""
        for side in ('left', 'right'):
            if self.claimed[f'{side}_arm']:
                off = ARM_OFFSET[side]
                self.arms[side].reseed(state)
                self._seed_from_state(state, range(off, off + 6))
        if self.claimed['head']:
            self._seed_from_state(state, (HEAD_YAW, HEAD_PITCH, HEAD_ROLL))
        if self.claimed['back']:
            self.back_target = _clamp(state.back_state.current_position, BACK_MIN, BACK_MAX)

    def estop_state_callback(self, msg: Bool):
        """React to the command_mux e-stop latch.

        While latched, VR input is ignored and every claimed subsystem holds the
        measured pose, so after reset VR resumes from where the robot actually is
        (the mux's resume fence then admits our fresh hold commands).
        """
        engaged = bool(msg.data)
        with self.cmd_lock:
            if engaged and not self.estop_active:
                state = self.get_robot_state()
                if state is not None:
                    self._hold_current_pose(state)
                self.cmd_vel = Twist()
                self.get_logger().warn('E-stop latched: VR holding current pose')
            elif not engaged and self.estop_active:
                for arm in self.arms.values():
                    if arm is not None:
                        arm.reset_tracking()
                self.get_logger().info('E-stop reset: VR input re-enabled')
            self.estop_active = engaged

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def control_tick(self):
        """100 Hz: read fresh VR input, update targets, publish claimed subsystems."""
        now = time.monotonic()
        dt = 0.01 if self.last_tick is None else _clamp(now - self.last_tick, 0.0, 0.1)
        self.last_tick = now

        goals = self.vr_monitor.get_latest_goals(max_age=self.link_timeout)
        state = self.get_robot_state()

        with self.cmd_lock:
            self._update_link_state(goals, now)
            if state is not None and not self.estop_active:
                self._apply_vr_input(goals, state, dt)
            self._publish(now, dt)

    def _update_link_state(self, goals, now):
        release_count, reason = self.vr_monitor.get_release_state()
        if release_count != self.last_release_count:
            self.last_release_count = release_count
            self.get_logger().info(f'VR client released control ({reason})')

        headset = goals['headset'] is not None
        controllers = goals['left'] is not None or goals['right'] is not None

        if headset != self.headset_connected:
            self.headset_connected = headset
            self.get_logger().info(f'Headset {"connected" if headset else "lost"}')
            if not headset and not self.passthrough_mode_active:
                self._release('head')

        # Passthrough (X) stops headset telemetry but keeps the head locked in
        # place; release it once the controllers are gone too.
        if not headset and not controllers:
            self._release('head')

        if controllers != self.controllers_connected:
            self.controllers_connected = controllers
            self.get_logger().info(f'Controllers {"connected" if controllers else "lost"}')
            if not controllers and self.claimed['base']:
                self._release('base')
                self.base_stop_until = now + self.base_stop_sec

        for side in ('left', 'right'):
            if goals[side] is None and self.grip_active[side]:
                self.grip_active[side] = False
                self.debug_log(f'{side} controller lost - arm tracking stopped')

        if headset or controllers:
            self.link_lost_time = None
        elif self.link_lost_time is None:
            self.link_lost_time = now
        elif self.hold_timeout >= 0 and now - self.link_lost_time > self.hold_timeout:
            self._release('left_arm', 'right_arm', 'back')

    def _apply_vr_input(self, goals, state, dt):
        if goals['headset'] is not None:
            self._claim_head(state)
            self._update_head(goals['headset'])

        left, right = goals['left'], goals['right']
        self.latest_goals = {'left': left, 'right': right}
        lx = ly = az = 0.0

        if left is not None:
            meta = left.metadata or {}
            stick = meta.get('thumbstick') or {}
            lx = -float(stick.get('y', 0.0)) * self.max_linear_vel  # forward/back (negated)
            ly = -float(stick.get('x', 0.0)) * self.max_linear_vel  # strafe (ROS +y = left)
            buttons = meta.get('buttons') or {}

            button_x = _button(buttons, 'x')
            if button_x and not self.left_x_button_pressed:
                self.passthrough_mode_active = not self.passthrough_mode_active
                self.get_logger().info(f'Passthrough mode: {"ON" if self.passthrough_mode_active else "OFF"}')
                self.vr_monitor.set_passthrough_mode(self.passthrough_mode_active)
            self.left_x_button_pressed = button_x

            button_y = _button(buttons, 'y')
            if button_y and not self.left_y_button_pressed:
                self.get_logger().info('Y pressed - moving both arms to zero position')
                for side in ('left', 'right'):
                    self._claim_arm(side, state)
                    self.arms[side].move_to_zero_position()
                    self._copy_arm_targets(side)
            self.left_y_button_pressed = button_y

        if right is not None:
            meta = right.metadata or {}
            stick = meta.get('thumbstick') or {}
            az = -float(stick.get('x', 0.0)) * self.max_angular_vel  # ROS +z = CCW (left)
            buttons = meta.get('buttons') or {}

            # Each B press raises, each A press lowers the back one preset
            button_a, button_b = _button(buttons, 'a'), _button(buttons, 'b')
            press_a = button_a and not self.right_a_button_pressed
            press_b = button_b and not self.right_b_button_pressed
            if press_a != press_b:
                self._claim_back(state)
                self.back_target = _next_back_preset(self.back_target, up=press_b)
                self.get_logger().info(f'Back height: {self.back_target:.3f}')
            self.right_a_button_pressed = button_a
            self.right_b_button_pressed = button_b

            thumbstick_button = bool(buttons.get('thumbstick', False))
            if thumbstick_button and not self.right_thumbstick_pressed:
                self._toggle_recording()
            self.right_thumbstick_pressed = thumbstick_button

        if left is not None or right is not None:
            self.claimed['base'] = True
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.angular.z = lx, ly, az

        for side, goal in (('left', left), ('right', right)):
            self._update_arm(side, goal, state, dt)

    def _update_head(self, headset_goal):
        """Headset orientation (WebXR local space, Y up) -> head yaw/pitch/roll targets."""
        meta = headset_goal.metadata or {}
        q = meta.get('quaternion') or {}
        if all(k in q for k in ('x', 'y', 'z', 'w')):
            yaw, pitch, roll = R.from_quat([q['x'], q['y'], q['z'], q['w']]).as_euler('YXZ')
        else:
            rot = meta.get('rotation') or {}
            if not rot:
                return
            yaw, pitch, roll = (math.radians(float(rot.get(k, 0.0))) for k in ('y', 'x', 'z'))

        if abs(math.degrees(yaw)) < HEAD_YAW_WRAP_GUARD_DEG:
            self.desired[HEAD_YAW] = _clamp(
                yaw * self.head_yaw_sign * self.head_yaw_multiplier, -self.head_yaw_limit, self.head_yaw_limit)
        self.desired[HEAD_PITCH] = _clamp(
            pitch * self.head_pitch_sign * self.head_pitch_multiplier, -self.head_pitch_limit, self.head_pitch_limit)
        self.desired[HEAD_ROLL] = _clamp(
            roll * self.head_roll_sign * self.head_roll_multiplier, -self.head_roll_limit, self.head_roll_limit)

    def _update_arm(self, side, goal, state, dt):
        """Grip-held delta tracking for one arm."""
        arm = self.arms[side]
        grip = bool(goal is not None and (goal.metadata or {}).get('grip_active', False))

        if grip and not self.grip_active[side]:
            self.debug_log(f'{side} grip pressed - starting arm tracking')
            self._claim_arm(side, state)   # seeds from the measured pose on first claim
            arm.reset_tracking()
        elif not grip and self.grip_active[side]:
            self.debug_log(f'{side} grip released - stopping arm tracking')
        self.grip_active[side] = grip

        if grip:
            try:
                arm.handle_vr_input(goal, dt)
                self._copy_arm_targets(side)
            except Exception as e:
                self.get_logger().warn(f'{side} arm VR input error: {e}')

    def _copy_arm_targets(self, side):
        off = ARM_OFFSET[side]
        targets = self.arms[side].target_positions
        for i, joint in enumerate(ARM_JOINTS):
            self.desired[off + i] = targets[joint]

    def _toggle_recording(self):
        msg = Bool()
        msg.data = True
        if not self.is_recording:
            self.recording_start_pub.publish(msg)
            self.get_logger().info('Recording: STARTED (right thumbstick pressed)')
        else:
            self.recording_stop_pub.publish(msg)
            self.get_logger().info('Recording: STOPPED (right thumbstick pressed)')
        self.is_recording = not self.is_recording

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _rate_limit(self, indices, dt):
        step = self.max_joint_speed * dt
        for i in indices:
            self.published[i] += _clamp(self.desired[i] - self.published[i], -step, step)

    def _servo_cmds(self, indices):
        cmds = []
        for i in indices:
            s = ServoCmd()
            s.enabled = True
            s.target_location = float(self.published[i])
            s.target_speed = 0.0          # motion is rate-limited here
            s.target_acceleration = 0.0
            s.target_torque = 0.0
            cmds.append(s)
        return cmds

    def _publish(self, now, dt):
        """Publish only the subsystems VR has claimed (the mux arbitrates per subsystem)."""
        for side, pub in (('left', self.left_arm_publisher), ('right', self.right_arm_publisher)):
            if self.claimed[f'{side}_arm']:
                off = ARM_OFFSET[side]
                self._rate_limit(range(off, off + 6), dt)
                msg = ArmCmd()
                msg.joint_cmd = self._servo_cmds(range(off, off + 6))
                pub.publish(msg)

        if self.claimed['head']:
            idx = (HEAD_YAW, HEAD_PITCH, HEAD_ROLL)
            self._rate_limit(idx, dt)
            msg = HeadCmd()
            msg.servos = self._servo_cmds(idx)
            msg.eye_pwm = [0, 0]  # eyes come from the eyes channel
            self.head_publisher.publish(msg)

        if self.claimed['back']:
            msg = BackCmd()
            msg.position = float(self.back_target)
            msg.velocity = 0.2
            msg.acceleration = 0.1
            self.back_publisher.publish(msg)

        if self.claimed['base']:
            self.base_publisher.publish(Twist() if self.estop_active else self.cmd_vel)
        elif now < self.base_stop_until:
            # Explicit stop so the mux doesn't forward our last velocity for its timeout.
            self.base_publisher.publish(Twist())

    def update_visualization(self):
        """Push data to the debug visualizer thread (30 Hz)."""
        if self.debug_visualizer is None:
            return
        try:
            side = self.debug_visualizer.get_current_prefix()
            arm = self.arms.get(side)
            if arm is None:
                return
            if not self.debug_visualizer.update_data(arm, self.latest_goals.get(side)):
                self.get_logger().info('Debug visualizer closed by user')
                self.debug_visualizer.stop()
                self.debug_visualizer = None
        except Exception as e:
            self.get_logger().warn(f'Visualization update error: {e}')

    def shutdown(self):
        """Stop the visualizer and VR servers."""
        self.get_logger().info('Shutting down VR teleop...')
        if self.debug_visualizer:
            self.debug_visualizer.stop()
            self.debug_visualizer = None
        if self.vr_loop and self.vr_loop.is_running():
            self.vr_monitor.is_running = False  # monitor loop exits and stops the servers
        if self.vr_thread:
            self.vr_thread.join(timeout=3.0)


def main():
    """Main function"""
    rclpy.init()
    node = AlfieTeleopVRNode()

    node.get_logger().info('Waiting for first robot state message...')
    while rclpy.ok() and node.get_robot_state() is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    ok = rclpy.ok() and node.initialize_controllers()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        if ok:
            executor.spin()
        else:
            node.get_logger().error('Failed to initialize arm controllers')
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
