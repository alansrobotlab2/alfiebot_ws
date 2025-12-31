#!/usr/bin/env python3
"""
VR Head Tracker - ROS2 node that publishes head tracking data from VR headset
Uses the VRMonitor class for VR communication.
"""

# Standard library imports
import asyncio
import math
import threading
import time

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from alfie_msgs.msg import RobotLowCmd, RobotLowState, ServoCmd, BackCmd
from alfie_msgs.srv import BackRequestCalibration
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# Local imports
from alfie_vr.vr_monitor import VRMonitor
from alfie_vr.kinematics import AlfieArmKinematics
from alfie_vr.kinematics import SimpleTeleopArm
from alfie_vr.arm_debug_visualizer import ArmDebugVisualizer, PYGAME_AVAILABLE


# Joint mapping configurations
LEFT_ARM_JOINT_MAP = {
    "shoulder_yaw": "left_arm_shoulder_yaw",
    "shoulder_pitch": "left_arm_shoulder_pitch",
    "elbow_pitch": "left_arm_elbow_pitch",
    "wrist_pitch": "left_arm_wrist_pitch",
    "wrist_roll": "left_arm_wrist_roll",
    "gripper": "left_arm_gripper",
}

RIGHT_ARM_JOINT_MAP = {
    "shoulder_yaw": "right_arm_shoulder_yaw",
    "shoulder_pitch": "right_arm_shoulder_pitch",
    "elbow_pitch": "right_arm_elbow_pitch",
    "wrist_pitch": "right_arm_wrist_pitch",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}

HEAD_JOINT_MAP = {
    "head_yaw": "head_yaw",
    "head_pitch": "head_pitch",
    "head_roll": "head_roll",
}


class AlfieTeleopVRNode(Node):
    """ROS2 node that publishes head tracking data from VR headset"""

    def __init__(self):
        super().__init__('alfiebot_teleop_vr_node')

        # Declare parameters
        self.declare_parameter(
            'back_height',
            0.390,
            ParameterDescriptor(description='Target back height in meters (0.0 to 0.390)')
        )

        # Get initial parameter value
        self.back_height = self.get_parameter('back_height').value

        # Register callback for parameter updates
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # Head tracking multipliers (headset rotation -> robot head rotation)
        self.head_pitch_multiplier = 2.0
        self.head_roll_multiplier = 2.0
        self.head_yaw_multiplier = 2.0
        
        # Create callback group for state subscription
        self.state_callback_group = ReentrantCallbackGroup()
        
        # QoS profile for best effort communication
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Create publisher for robot low-level commands
        self.cmd_publisher = self.create_publisher(
            RobotLowCmd,
            '/alfie/robotlowcmd',
            qos_best_effort
        )
        
        # Create subscriber for robot low-level state
        self.state_subscriber = self.create_subscription(
            RobotLowState,
            '/alfie/robotlowstate',
            self.robot_state_callback,
            qos_best_effort,
            callback_group=self.state_callback_group
        )
        
        # Storage for latest robot state with thread lock
        self.robot_state = None
        self.robot_state_lock = threading.Lock()

        # Initialize servo positions dictionary (radians)
        self.init_positions = {
            "left_shoulder_yaw":    0.0000,
            "left_shoulder_pitch":  0.1089,
            "left_elbow_pitch":    -1.4864,
            "left_wrist_pitch":    -0.1442,
            "left_wrist_roll":      0.0000,
            "left_gripper":         0.0000,
            "right_shoulder_yaw":   0.0000,
            "right_shoulder_pitch": 0.1089,
            "right_elbow_pitch":   -1.4864,
            "right_wrist_pitch":   -0.1442,
            "right_wrist_roll":     0.0000,
            "right_gripper":        0.0000,
            "head_yaw":             0.0000,
            "head_pitch":           0.0000,
            "head_roll":            0.0000,
        }
        
        # Servo index mapping
        self.servo_indices = {
            "left_shoulder_yaw":    0,
            "left_shoulder_pitch":  1,
            "left_elbow_pitch":     2,
            "left_wrist_pitch":     3,
            "left_wrist_roll":      4,
            "left_gripper":         5,
            "right_shoulder_yaw":   6,
            "right_shoulder_pitch": 7,
            "right_elbow_pitch":    8,
            "right_wrist_pitch":    9,
            "right_wrist_roll":     10,
            "right_gripper":        11,
            "head_yaw":             12,
            "head_pitch":           13,
            "head_roll":            14,
        }
        
        # Initialize RobotLowCmd state with default positions and all servos activated
        self.robot_cmd_state = RobotLowCmd()
        self.robot_cmd_state.servo_cmd = [ServoCmd() for _ in range(15)]
        
        # Set initial positions and activate all servos
        for servo_name, position in self.init_positions.items():
            servo_idx = self.servo_indices[servo_name]
            self.robot_cmd_state.servo_cmd[servo_idx].enabled = True
            self.robot_cmd_state.servo_cmd[servo_idx].target_location = position
            self.robot_cmd_state.servo_cmd[servo_idx].target_speed = 0.0
            self.robot_cmd_state.servo_cmd[servo_idx].target_acceleration = 0.0
            self.robot_cmd_state.servo_cmd[servo_idx].target_torque = 0.0
        
        # Initialize other command fields
        self.robot_cmd_state.eye_pwm = [0, 0]
        self.robot_cmd_state.shoulder_height = 0.0
        self.robot_cmd_state.back_cmd = BackCmd()
        self.robot_cmd_state.back_cmd.position = self.back_height
        self.robot_cmd_state.back_cmd.velocity = 0.2
        self.robot_cmd_state.back_cmd.acceleration = 0.1
        self.robot_cmd_state.cmd_vel = Twist()
        
        # Debug logging flag - set to True to enable verbose debug output
        self.debug_logs = False
        
        # Create timer for 100Hz publishing
        self.timer = self.create_timer(0.01, self.publish_robotlowcmd)  # 100Hz = 0.01s period
        
        # Create VR monitor (pass debug_logs flag)
        self.vr_monitor = VRMonitor(debug_logs=self.debug_logs)
        
        # Thread for running VR monitor async loop
        self.vr_thread = None
        self.vr_loop = None
        
        # Track headset connection status
        self.headset_connected = False
        
        # Performance tracking
        self.last_publish_time = None
        self.publish_count = 0
        self.publish_time_sum = 0.0
        self.max_publish_time = 0.0
        
        # Create separate timer for async VR data processing at 90Hz
        self.vr_update_timer = self.create_timer(0.011, self.update_from_vr_data)  # 90Hz
        
        # Arm and head controllers (initialized after first robot state)
        self.left_arm = None
        self.right_arm = None
        self.head = None
        self.kinematics_left = None
        self.kinematics_right = None
        
        # Grip button state tracking for delta control
        self.left_grip_active = False
        self.right_grip_active = False

        # Velocity control with acceleration limits
        # Velocity limits (easier to adjust here)
        self.max_linear_vel = 0.15  # m/s - max linear speed
        self.max_angular_vel = 0.5  # rad/s - max rotation speed

        # Acceleration limits (smoother motion control)
        self.max_linear_accel = 1.0  # m/s^2 - reach max speed in 0.25s
        self.max_angular_accel = 3.0  # rad/s^2 - reach max angular speed in 0.25s

        # Current velocity state
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        self.last_vel_update_time = time.time()
        
        # Passthrough mode toggle state (triggered by X button on left controller)
        self.passthrough_mode_active = False
        self.left_x_button_pressed = False  # Track X button state for edge detection

        # Recording toggle state (triggered by right thumbstick button)
        self.is_recording = False
        self.right_thumbstick_pressed = False  # Track thumbstick button state for edge detection

        # Recording control publishers (use same QoS as data_recorder subscribers)
        qos_recording = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.recording_start_pub = self.create_publisher(Bool, '/alfie/recording/start', qos_recording)
        self.recording_stop_pub = self.create_publisher(Bool, '/alfie/recording/stop', qos_recording)

        # Debug visualizer (initialized after arm controllers)
        self.debug_visualizer = None
        self.enable_debug_viz = False  # Set to False to disable
        self.latest_left_controller_goal = None  # Store for visualization
        
        # Create timer for visualization data update at 30Hz (just pushes data to viz thread)
        self.viz_timer = self.create_timer(0.033, self.update_visualization)  # ~30Hz
        
        self.get_logger().info('Alfie Teleop VR Node initialized')
        
        # Start VR monitor in separate thread
        self.start_vr_monitor()
    
    def debug_log(self, msg: str):
        """Log a debug message only if debug_logs is enabled."""
        if self.debug_logs:
            self.get_logger().info(msg)

    def _on_parameter_change(self, params):
        """Callback for parameter updates at runtime."""
        for param in params:
            if param.name == 'back_height':
                new_value = param.value
                # Clamp to valid range
                new_value = max(0.0, min(0.390, new_value))
                self.back_height = new_value
                self.robot_cmd_state.back_cmd.position = new_value
                self.get_logger().info(f'back_height parameter updated to {new_value:.3f}')
        return SetParametersResult(successful=True)

    def start_vr_monitor(self):
        """Start VR monitor in a separate thread"""
        # Initialize VR monitor
        self.get_logger().info('Initializing VR monitor...')
        if not self.vr_monitor.initialize():
            self.get_logger().error('VR monitor initialization failed')
            return
        
        def run_vr_loop():
            self.vr_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.vr_loop)
            try:
                self.vr_loop.run_until_complete(self.vr_monitor.start_monitoring())
            except Exception as e:
                self.get_logger().error(f'VR monitor error: {e}')
        
        self.vr_thread = threading.Thread(target=run_vr_loop, daemon=True)
        self.vr_thread.start()
        self.get_logger().info('VR monitor started in background thread')
    
    def robot_state_callback(self, msg: RobotLowState):
        """Callback for robot state messages - stores latest state"""
        with self.robot_state_lock:
            self.robot_state = msg
    
    def get_robot_state(self):
        """Get the latest robot state (thread-safe)"""
        with self.robot_state_lock:
            return self.robot_state
    
    def initialize_arm_controllers(self):
        """Initialize arm and head controllers after robot state is available"""
        robot_state = self.get_robot_state()
        if robot_state is None:
            self.get_logger().error('Cannot initialize arm controllers: no robot state')
            return False
        
        self.kinematics_left = AlfieArmKinematics()
        self.kinematics_right = AlfieArmKinematics()
        
        self.left_arm = SimpleTeleopArm(
            joint_map=LEFT_ARM_JOINT_MAP,
            robotlowstate=robot_state,
            kinematics=self.kinematics_left,
            prefix='left',
            kp=1,
            debug_logs=self.debug_logs
        )
        self.right_arm = SimpleTeleopArm(
            joint_map=RIGHT_ARM_JOINT_MAP,
            robotlowstate=robot_state,
            kinematics=self.kinematics_right,
            prefix='right',
            kp=1,
            debug_logs=self.debug_logs
        )

        # Call back calibration service if not already calibrated
        if not robot_state.back_state.is_calibrated:
            self.get_logger().info('Back not calibrated, calling calibration service...')
            calibrate_client = self.create_client(BackRequestCalibration, '/alfie/low/calibrate_back')
            if calibrate_client.wait_for_service(timeout_sec=5.0):
                request = BackRequestCalibration.Request()
                future = calibrate_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                if future.result() is not None:
                    if future.result().success:
                        self.get_logger().info('Back calibration successful')
                    else:
                        self.get_logger().warn('Back calibration returned failure')
                else:
                    self.get_logger().warn('Back calibration service call failed')
            else:
                self.get_logger().warn('Back calibration service not available')
        else:
            self.get_logger().info('Back already calibrated, skipping calibration')
        
        self.get_logger().info('Arm and head controllers initialized')
        
        # Start debug visualizer (can switch between left/right arms)
        if self.enable_debug_viz and PYGAME_AVAILABLE:
            self.debug_visualizer = ArmDebugVisualizer(
                self.kinematics_left, 
                self.kinematics_right, 
                prefix="left"
            )
            if self.debug_visualizer.start():
                self.get_logger().info('Debug visualizer started successfully (press L/R/Tab to switch arms)')
            else:
                self.get_logger().warn('Debug visualizer failed to start')
                self.debug_visualizer = None
        
        # Store latest controller goals for visualization
        self.latest_right_controller_goal = None
        
        return True
    
    def _apply_arm_targets(self, arm, prefix):
        """Apply arm target positions to robot command state"""
        offset = 0 if prefix == "left" else 6
        
        # Map SimpleTeleopArm joint names to servo indices
        # SimpleTeleopArm uses: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
        # Our servos are: shoulder_yaw(0), shoulder_pitch(1), elbow_pitch(2), wrist_pitch(3), wrist_roll(4), gripper(5)
        joint_to_local_idx = {
            "shoulder_pan": 0,    # maps to shoulder_yaw
            "shoulder_lift": 1,   # maps to shoulder_pitch  
            "elbow_flex": 2,      # maps to elbow_pitch
            "wrist_flex": 3,      # maps to wrist_pitch
            "wrist_roll": 4,      # maps to wrist_roll
            "gripper": 5,         # maps to gripper
        }
        
        for joint, local_idx in joint_to_local_idx.items():
            if joint in arm.target_positions:
                # SimpleTeleopArm now uses radians directly - no conversion needed
                target_rad = arm.target_positions[joint]
                self.robot_cmd_state.servo_cmd[offset + local_idx].target_location = target_rad
    
    def update_from_vr_data(self):
        """Update robot command state from VR data asynchronously (runs at 90Hz)"""
        # Get headset and controller goals from VR monitor
        dual_goals = self.vr_monitor.get_latest_goal_nowait()
        headset_goal = dual_goals.get("headset") if dual_goals else None
        left_controller_goal = dual_goals.get("left") if dual_goals else None
        right_controller_goal = dual_goals.get("right") if dual_goals else None
        
        # Track connection status changes
        if headset_goal is None:
            if self.headset_connected:
                self.headset_connected = False
                self.get_logger().info('Headset disconnected')
            return
        
        # Log when headset connects
        if not self.headset_connected:
            self.headset_connected = True
            self.get_logger().info('Headset connected')

        # Extract rotation angles from headset goal metadata
        # The VR system sends rotation as {'x': pitch, 'y': yaw, 'z': roll}
        if hasattr(headset_goal, 'metadata') and headset_goal.metadata:
            rotation = headset_goal.metadata.get('rotation', {})
            
            if rotation:
                # Servo 12: Head Pan (yaw - rotation around Z-axis)
                # VR sends this as 'y' key
                if 'y' in rotation:
                    yaw_deg = float(rotation['y']) * self.head_yaw_multiplier
                    yaw_rad = math.radians(yaw_deg)
                    self.robot_cmd_state.servo_cmd[12].target_location = yaw_rad
                
                # Servo 13: Head Tilt (pitch - rotation around Y-axis)
                # VR sends this as 'x' key
                if 'x' in rotation:
                    pitch_deg = float(rotation['x']) * self.head_pitch_multiplier
                    pitch_rad = math.radians(pitch_deg)
                    self.robot_cmd_state.servo_cmd[13].target_location = pitch_rad
                
                # Servo 14: Head Roll (roll - rotation around X-axis)
                # VR sends this as 'z' key
                if 'z' in rotation:
                    roll_deg = float(rotation['z']) * self.head_roll_multiplier
                    roll_rad = math.radians(roll_deg)
                    self.robot_cmd_state.servo_cmd[14].target_location = roll_rad
        
        # Process joystick input for cmd_vel
        # Calculate time delta for acceleration limiting
        current_time = time.time()
        dt = current_time - self.last_vel_update_time
        self.last_vel_update_time = current_time
        dt = max(0.001, min(dt, 0.1))  # Clamp dt to reasonable range (1ms to 100ms)
        
        # Target velocities from joystick input
        target_linear_x = 0.0
        target_linear_y = 0.0
        target_angular_z = 0.0

        # Left joystick controls linear velocity (forward/back and strafe left/right)
        if left_controller_goal and hasattr(left_controller_goal, 'metadata') and left_controller_goal.metadata:
            thumbstick = left_controller_goal.metadata.get('thumbstick', {})
            if thumbstick:
                # X-axis: strafe (positive = right, negative = left)
                # Y-axis: forward/back (positive = forward, negative = back)
                joy_x = float(thumbstick.get('x', 0.0))
                joy_y = float(thumbstick.get('y', 0.0))

                # Map joystick values (-1 to 1) to target velocity limits
                target_linear_x = -joy_y * self.max_linear_vel  # forward/back (negated)
                target_linear_y = joy_x * self.max_linear_vel  # strafe left/right
            
            # Y button: reset both arms to zero positions
            buttons = left_controller_goal.metadata.get('buttons', {})
            button_y = buttons.get('y', False) or buttons.get('Y', False)
            
            # X button: toggle passthrough mode
            button_x = buttons.get('x', False) or buttons.get('X', False)
            
            # Detect X button press (rising edge)
            if button_x and not self.left_x_button_pressed:
                # Toggle passthrough mode
                self.passthrough_mode_active = not self.passthrough_mode_active
                self.get_logger().info(f'Passthrough mode: {"ON" if self.passthrough_mode_active else "OFF"}')
                
                # Send passthrough mode command to VR headset
                self.vr_monitor.set_passthrough_mode(self.passthrough_mode_active)
            
            self.left_x_button_pressed = button_x
            
            if button_y:
                self.get_logger().info('Y button pressed - resetting both arms to zero positions')
                
                if self.left_arm is not None:
                    self.left_arm.move_to_zero_position()
                    self._apply_arm_targets(self.left_arm, "left")
                
                if self.right_arm is not None:
                    self.right_arm.move_to_zero_position()
                    self._apply_arm_targets(self.right_arm, "right")
        
        # Right joystick X-axis controls angular velocity (rotate left/right)
        # A/B buttons control back height
        if right_controller_goal and hasattr(right_controller_goal, 'metadata') and right_controller_goal.metadata:
            thumbstick = right_controller_goal.metadata.get('thumbstick', {})
            if thumbstick:
                # X-axis: rotation (positive = rotate right, negative = rotate left)
                joy_x = float(thumbstick.get('x', 0.0))

                # Map joystick value (-1 to 1) to target angular velocity
                target_angular_z = joy_x * self.max_angular_vel  # Rotation
            
            # A/B buttons: back height control (range 0.000 to 0.390)
            # B button increases height, A button decreases height
            # Max rate: 0.1 m/s at 100Hz = 0.001 m per tick
            buttons = right_controller_goal.metadata.get('buttons', {})
            
            # Debug: log buttons dictionary when it has content
            # if buttons:
            #     self.get_logger().info(f'Buttons received: {buttons}')
            
            button_a = buttons.get('a', False) or buttons.get('A', False)
            button_b = buttons.get('b', False) or buttons.get('B', False)
            
            BACK_MAX_RATE = 0.02  # m/s
            BACK_MIN = 0.000
            BACK_MAX = 0.390
            dt = 0.01  # 100Hz update rate
            
            # Calculate delta based on button presses
            back_delta = 0.0
            if button_b:
                back_delta = BACK_MAX_RATE * dt  # B increases height
            elif button_a:
                back_delta = -BACK_MAX_RATE * dt  # A decreases height
            
            if back_delta != 0.0:
                new_back_pos = self.robot_cmd_state.back_cmd.position + back_delta
                # Clamp to valid range
                new_back_pos = max(BACK_MIN, min(BACK_MAX, new_back_pos))
                self.robot_cmd_state.back_cmd.position = new_back_pos

                # Debug log
                self.debug_log(f'Back height: {new_back_pos:.3f} ({"B" if button_b else "A"} pressed)')

            # Thumbstick button: toggle recording (edge detection)
            thumbstick_button = buttons.get('thumbstick', False)

            if thumbstick_button and not self.right_thumbstick_pressed:
                # Rising edge - toggle recording state
                msg = Bool()
                msg.data = True

                if not self.is_recording:
                    # Start recording
                    self.recording_start_pub.publish(msg)
                    self.is_recording = True
                    self.get_logger().info('Recording: STARTED (right thumbstick pressed)')
                else:
                    # Stop recording
                    self.recording_stop_pub.publish(msg)
                    self.is_recording = False
                    self.get_logger().info('Recording: STOPPED (right thumbstick pressed)')

            self.right_thumbstick_pressed = thumbstick_button

        # Apply acceleration limiting to smooth velocity changes
        def apply_accel_limit(current, target, max_accel, dt):
            """Apply acceleration limiting to velocity changes"""
            delta = target - current
            max_delta = max_accel * dt

            if abs(delta) <= max_delta:
                return target
            else:
                return current + math.copysign(max_delta, delta)

        # Smoothly ramp velocities with acceleration limits
        self.current_linear_x = apply_accel_limit(self.current_linear_x, target_linear_x, self.max_linear_accel, dt)
        self.current_linear_y = apply_accel_limit(self.current_linear_y, target_linear_y, self.max_linear_accel, dt)
        self.current_angular_z = apply_accel_limit(self.current_angular_z, target_angular_z, self.max_angular_accel, dt)

        # Apply smoothed velocities to command
        self.robot_cmd_state.cmd_vel.linear.x = self.current_linear_x
        self.robot_cmd_state.cmd_vel.linear.y = self.current_linear_y
        self.robot_cmd_state.cmd_vel.angular.z = self.current_angular_z

        # Debug log (only when commanded velocity is non-zero)
        if abs(self.current_linear_x) > 0.01 or abs(self.current_linear_y) > 0.01 or abs(self.current_angular_z) > 0.01:
            self.debug_log(f'Cmd_vel: linear=({self.current_linear_x:.2f}, {self.current_linear_y:.2f}), angular={self.current_angular_z:.2f}')

        # Process arm control using SimpleTeleopArm with VR controller input
        # Only process when grip button is held (delta control relative to grip press)
        if self.left_arm is not None and left_controller_goal is not None:
            try:
                # Check grip button state from metadata
                left_grip_active = False
                if hasattr(left_controller_goal, 'metadata') and left_controller_goal.metadata:
                    left_grip_active = left_controller_goal.metadata.get('grip_active', False)
                    # Also check for 'gripActive' key (alternative naming)
                    if not left_grip_active:
                        left_grip_active = left_controller_goal.metadata.get('gripActive', False)
                
                # Detect grip press (transition from not pressed to pressed)
                if left_grip_active and not self.left_grip_active:
                    self.debug_log('Left grip pressed - starting arm tracking')
                    # Reset delta tracking by clearing previous position
                    if hasattr(self.left_arm, 'prev_vr_pos'):
                        delattr(self.left_arm, 'prev_vr_pos')
                    if hasattr(self.left_arm, 'prev_wrist_flex'):
                        delattr(self.left_arm, 'prev_wrist_flex')
                    if hasattr(self.left_arm, 'prev_wrist_roll'):
                        delattr(self.left_arm, 'prev_wrist_roll')
                
                # Detect grip release
                if not left_grip_active and self.left_grip_active:
                    self.debug_log('Left grip released - stopping arm tracking')
                
                self.left_grip_active = left_grip_active
                
                # Only process arm movement if grip is held
                if left_grip_active:
                    gripper_state = None
                    self.left_arm.handle_vr_input(left_controller_goal, gripper_state)
                    self._apply_arm_targets(self.left_arm, "left")
                
                # Store VR goal for visualization
                self.latest_left_controller_goal = left_controller_goal
            except Exception as e:
                self.get_logger().warn(f'Left arm VR input error: {e}')
        else:
            # Still store goal even if arm not active (for visualization)
            self.latest_left_controller_goal = left_controller_goal
        
        if self.right_arm is not None and right_controller_goal is not None:
            try:
                # Check grip button state from metadata
                right_grip_active = False
                if hasattr(right_controller_goal, 'metadata') and right_controller_goal.metadata:
                    right_grip_active = right_controller_goal.metadata.get('grip_active', False)
                    if not right_grip_active:
                        right_grip_active = right_controller_goal.metadata.get('gripActive', False)
                
                # Detect grip press
                if right_grip_active and not self.right_grip_active:
                    self.debug_log('Right grip pressed - starting arm tracking')
                    if hasattr(self.right_arm, 'prev_vr_pos'):
                        delattr(self.right_arm, 'prev_vr_pos')
                    if hasattr(self.right_arm, 'prev_wrist_flex'):
                        delattr(self.right_arm, 'prev_wrist_flex')
                    if hasattr(self.right_arm, 'prev_wrist_roll'):
                        delattr(self.right_arm, 'prev_wrist_roll')
                
                # Detect grip release
                if not right_grip_active and self.right_grip_active:
                    self.debug_log('Right grip released - stopping arm tracking')
                
                self.right_grip_active = right_grip_active
                
                # Only process arm movement if grip is held
                if right_grip_active:
                    gripper_state = None
                    self.right_arm.handle_vr_input(right_controller_goal, gripper_state)
                    self._apply_arm_targets(self.right_arm, "right")
                
                # Store VR goal for visualization
                self.latest_right_controller_goal = right_controller_goal
            except Exception as e:
                self.get_logger().warn(f'Right arm VR input error: {e}')
        else:
            # Still store goal even if arm not active (for visualization)
            self.latest_right_controller_goal = right_controller_goal
    
    def update_visualization(self):
        """Update debug visualization data at 30Hz - just pushes data to viz thread"""
        if self.debug_visualizer is None:
            return
        
        try:
            # Determine which arm to visualize based on visualizer's current target
            current_prefix = self.debug_visualizer.get_current_prefix()
            
            if current_prefix == "left" and self.left_arm is not None:
                arm = self.left_arm
                vr_goal = self.latest_left_controller_goal
            elif current_prefix == "right" and self.right_arm is not None:
                arm = self.right_arm
                vr_goal = self.latest_right_controller_goal
            else:
                return
            
            if not self.debug_visualizer.update_data(arm, vr_goal):
                # User explicitly closed visualizer window (pressed Q or closed window)
                self.get_logger().info('Debug visualizer closed by user')
                self.debug_visualizer.stop()
                self.debug_visualizer = None
        except Exception as e:
            self.get_logger().warn(f'Visualization update error: {e}')
    
    def publish_robotlowcmd(self):
        """Publish robot command at 100Hz - just publishes current state"""
        start_time = time.time()
        
        # Track actual callback rate
        # if self.last_publish_time is not None:
        #     actual_dt = start_time - self.last_publish_time
        #     actual_hz = 1.0 / actual_dt if actual_dt > 0 else 0
        #     if self.publish_count % 100 == 0:
        #         self.get_logger().info(f'Actual timer rate: {actual_hz:.1f}Hz (dt={actual_dt*1000:.2f}ms)')
        # self.last_publish_time = start_time
        
        # Simply publish whatever is in robot_cmd_state
        self.cmd_publisher.publish(self.robot_cmd_state)
        
        # Performance tracking
        elapsed = time.time() - start_time
        self.publish_count += 1
        self.publish_time_sum += elapsed
        self.max_publish_time = max(self.max_publish_time, elapsed)
        
        # Log performance stats every 100 publishes (~1 second at 100Hz)
        # if self.publish_count % 100 == 0:
        #     avg_time = self.publish_time_sum / 100
        #     self.get_logger().info(
        #         f'Publish stats: avg={avg_time*1000:.2f}ms, max={self.max_publish_time*1000:.2f}ms, '
        #         f'theoretical_hz={1.0/avg_time:.1f}Hz'
        #     )
        #     # Reset counters
        #     self.publish_time_sum = 0.0
        #     self.max_publish_time = 0.0
    
    def shutdown(self):
        """Shutdown the node and VR monitor"""
        self.get_logger().info('Shutting down head tracker...')
        
        # Stop debug visualizer (it handles its own pygame cleanup in its thread)
        if self.debug_visualizer:
            self.debug_visualizer.stop()
            self.debug_visualizer = None
            self.get_logger().info('Debug visualizer stopped')
        
        if self.vr_loop:
            self.vr_loop.call_soon_threadsafe(self.vr_loop.stop)
        if self.vr_thread:
            self.vr_thread.join(timeout=2.0)


def main():
    """Main function"""
    print("🎮 Alfie Teleop- VR Headset to Robot Control")
    print("=" * 60)
    
    # Initialize ROS2
    rclpy.init()
    
    # Create node
    node = AlfieTeleopVRNode()

    # wait for first robot state message
    print("⏳ Waiting for first robot state message...")
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.get_robot_state() is not None:
            print("✅ Received first robot state message")
            break   

    # Initialize arm and head controllers now that we have robot state
    if node.initialize_arm_controllers():
        print("✅ Arm and head controllers initialized")
    else:
        print("❌ Failed to initialize arm controllers")
    
    # Create multi-threaded executor for state subscription
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    
    try:
        # Spin the node with multi-threaded executor
        executor.spin()
    except KeyboardInterrupt:
        print("\n👋 Teleop stopped by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()