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
from alfie_msgs.msg import RobotLowCmd, RobotLowState, ServoCmd, BackCmd
from alfie_msgs.srv import BackRequestCalibration
from geometry_msgs.msg import Twist

# Local imports
from alfie_vr.vr_monitor import VRMonitor
from alfie_vr.kinematics import AlfieArmKinematics
from alfie_vr.kinematics import SimpleTeleopArm
from alfie_vr.kinematics import SimpleHeadControl

# Optional pygame import for debug visualization
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False


class ArmDebugVisualizer:
    """
    Pygame-based debug visualizer for arm kinematics.
    Runs in a separate thread to avoid interfering with robotlowcmd timing.
    """
    
    # Colors
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (0, 100, 255)
    YELLOW = (255, 255, 0)
    GRAY = (128, 128, 128)
    LIGHT_GRAY = (200, 200, 200)
    DARK_GRAY = (50, 50, 50)
    ORANGE = (255, 165, 0)
    CYAN = (0, 255, 255)
    MAGENTA = (255, 0, 255)
    
    def __init__(self, kinematics_left, kinematics_right, prefix="left"):
        self.kinematics_left = kinematics_left
        self.kinematics_right = kinematics_right
        self.kinematics = kinematics_left if prefix == "left" else kinematics_right
        self.prefix = prefix
        self.running = False
        self.screen = None
        self.clock = None
        self.font = None
        self.small_font = None
        
        # Window settings
        self.window_width = 900
        self.window_height = 700
        self.scale = 800  # pixels per meter
        self.origin_x = self.window_width // 2
        self.origin_y = 150  # Near top since arm hangs down
        
        # Latest data (thread-safe with lock)
        self.latest_data = None
        self.data_lock = threading.Lock()
        
        # Thread management
        self.viz_thread = None
        self.stop_event = threading.Event()
        
        # Arm switching (thread-safe)
        self.target_prefix = prefix
        self.prefix_lock = threading.Lock()
        
    def start(self):
        """Start pygame visualization in a separate thread."""
        if not PYGAME_AVAILABLE:
            print("[DEBUG VIS] pygame not available, skipping visualization")
            return False
        
        self.stop_event.clear()
        self.running = True
        self.viz_thread = threading.Thread(target=self._viz_thread_loop, daemon=True)
        self.viz_thread.start()
        print(f"[DEBUG VIS] Started {self.prefix} arm visualizer thread")
        return True
    
    def _viz_thread_loop(self):
        """Main visualization loop running in separate thread."""
        try:
            # Initialize pygame in this thread
            pygame.init()
            
            # Check if display module initialized correctly
            if not pygame.display.get_init():
                print("[DEBUG VIS] pygame display failed to initialize")
                self.running = False
                return
            
            self.screen = pygame.display.set_mode((self.window_width, self.window_height))
            pygame.display.set_caption(f"Alfie {self.prefix.title()} Arm Debug Visualizer")
            self.clock = pygame.time.Clock()
            self.font = pygame.font.Font(None, 22)
            self.small_font = pygame.font.Font(None, 18)
            
            print(f"[DEBUG VIS] Pygame initialized in thread for {self.prefix} arm")
            
            # Main loop
            while self.running and not self.stop_event.is_set():
                try:
                    # Handle pygame events
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            print("[DEBUG VIS] Window close requested")
                            self.running = False
                            break
                        if event.type == pygame.KEYDOWN:
                            if event.key in (pygame.K_q, pygame.K_ESCAPE):
                                print("[DEBUG VIS] Quit key pressed")
                                self.running = False
                                break
                            # Arm switching keys
                            elif event.key == pygame.K_l:
                                self._switch_arm("left")
                            elif event.key == pygame.K_r:
                                self._switch_arm("right")
                            elif event.key == pygame.K_TAB:
                                # Toggle between left and right
                                new_prefix = "right" if self.prefix == "left" else "left"
                                self._switch_arm(new_prefix)
                    
                    if not self.running:
                        break
                    
                    # Get latest data with lock
                    with self.data_lock:
                        data = self.latest_data
                    
                    if data is not None:
                        # Clear screen
                        self.screen.fill(self.BLACK)
                        
                        # Draw visualization
                        self._draw_grid(self.screen, self.small_font)
                        self._draw_workspace(self.screen)
                        self._draw_arm(self.screen, data)
                        self._draw_info_panel(self.screen, self.font, self.small_font, data)
                        self._draw_vr_input_panel(self.screen, self.font, self.small_font, data)
                        self._draw_controls_panel(self.screen, self.small_font)
                        
                        # Update display
                        pygame.display.flip()
                    
                    # Limit to ~30 FPS to reduce CPU usage
                    self.clock.tick(30)
                    
                except pygame.error as e:
                    print(f"[DEBUG VIS] Pygame error: {e}")
                    # Don't stop on pygame errors, just skip this frame
                    pass
                except Exception as e:
                    print(f"[DEBUG VIS] Update error: {e}")
                    # Don't stop on errors, just skip this frame
                    pass
            
        except Exception as e:
            print(f"[DEBUG VIS] Thread initialization error: {e}")
        finally:
            # Cleanup pygame in this thread
            try:
                pygame.display.quit()
                pygame.quit()
            except Exception:
                pass
            self.screen = None
            print(f"[DEBUG VIS] {self.prefix.title()} arm visualizer thread stopped")
        
    def stop(self):
        """Stop the visualizer thread."""
        self.running = False
        self.stop_event.set()
        if self.viz_thread is not None:
            self.viz_thread.join(timeout=2.0)
            self.viz_thread = None
        print(f"[DEBUG VIS] {self.prefix.title()} arm visualizer stopped")
    
    def _switch_arm(self, new_prefix):
        """Switch which arm is being visualized."""
        if new_prefix == self.prefix:
            return
        
        with self.prefix_lock:
            self.target_prefix = new_prefix
        
        self.prefix = new_prefix
        self.kinematics = self.kinematics_left if new_prefix == "left" else self.kinematics_right
        
        # Update window title
        if self.screen:
            pygame.display.set_caption(f"Alfie {self.prefix.title()} Arm Debug Visualizer")
        
        print(f"[DEBUG VIS] Switched to {new_prefix.upper()} arm")
    
    def get_current_prefix(self):
        """Get the currently targeted arm prefix (thread-safe)."""
        with self.prefix_lock:
            return self.target_prefix
            
    def update_data(self, arm_controller, vr_goal=None):
        """
        Update visualization data - call this from main loop.
        Thread-safe data update for the visualization thread.
        """
        if not self.running:
            return self.running
        
        try:
            # Prepare data snapshot
            data = {
                'current_x': getattr(arm_controller, 'current_x', 0.0),
                'current_y': getattr(arm_controller, 'current_y', 0.0),
                'target_positions': arm_controller.target_positions.copy(),
                'pitch': getattr(arm_controller, 'pitch', 0.0),
                'prev_vr_pos': getattr(arm_controller, 'prev_vr_pos', None),
                'vr_goal': None,
                'timestamp': time.time(),
            }
            
            # Extract VR goal data if available
            if vr_goal is not None:
                target_pos = None
                if hasattr(vr_goal, 'target_position') and vr_goal.target_position is not None:
                    try:
                        target_pos = list(vr_goal.target_position)
                    except (TypeError, ValueError):
                        target_pos = None
                
                meta = {}
                if hasattr(vr_goal, 'metadata') and vr_goal.metadata is not None:
                    try:
                        meta = dict(vr_goal.metadata)
                    except (TypeError, ValueError):
                        meta = {}
                
                data['vr_goal'] = {
                    'target_position': target_pos,
                    'wrist_flex_deg': getattr(vr_goal, 'wrist_flex_deg', None),
                    'wrist_roll_deg': getattr(vr_goal, 'wrist_roll_deg', None),
                    'metadata': meta,
                }
            
            # Update data with lock
            with self.data_lock:
                self.latest_data = data
                
        except Exception as e:
            print(f"[DEBUG VIS] Data update error: {e}")
        
        return self.running
            
    def _world_to_screen(self, x, y):
        """Convert world coordinates (meters) to screen coordinates (pixels)."""
        screen_x = self.origin_x + x * self.scale
        screen_y = self.origin_y - y * self.scale  # Flip Y for screen
        return int(screen_x), int(screen_y)
    
    def _draw_grid(self, screen, font):
        """Draw reference grid."""
        grid_spacing = 0.05 * self.scale  # 5cm grid
        
        # Vertical lines
        for i in range(-10, 11):
            x = self.origin_x + i * grid_spacing
            color = self.GRAY if i != 0 else self.WHITE
            pygame.draw.line(screen, color, (x, 0), (x, self.window_height), 1 if i != 0 else 2)
        
        # Horizontal lines
        for i in range(-12, 5):
            y = self.origin_y - i * grid_spacing
            color = self.GRAY if i != 0 else self.WHITE
            pygame.draw.line(screen, color, (0, y), (self.window_width, y), 1 if i != 0 else 2)
        
        # Axis labels
        label_x = font.render("+X (forward)", True, self.WHITE)
        label_y = font.render("+Y (up)", True, self.WHITE)
        screen.blit(label_x, (self.window_width - 80, self.origin_y + 5))
        screen.blit(label_y, (self.origin_x + 5, 10))
    
    def _draw_workspace(self, screen):
        """Draw reachable workspace boundary."""
        l1, l2 = self.kinematics.l1, self.kinematics.l2
        r_max = (l1 + l2) * self.scale
        r_min = abs(l1 - l2) * self.scale
        
        # Max reach circle
        pygame.draw.circle(screen, self.DARK_GRAY, (self.origin_x, self.origin_y), int(r_max), 2)
        # Min reach circle  
        pygame.draw.circle(screen, self.DARK_GRAY, (self.origin_x, self.origin_y), int(r_min), 1)
    
    def _draw_arm(self, screen, data):
        """Draw the 2-link arm."""
        if data is None:
            return
        
        targets = data['target_positions']
        shoulder_lift = targets.get('shoulder_lift', 0.0)
        elbow_flex = targets.get('elbow_flex', 0.0)
        
        # Get positions from kinematics
        x_elbow, y_elbow = self.kinematics.get_elbow_position(shoulder_lift)
        x_end, y_end = self.kinematics.forward_kinematics(shoulder_lift, elbow_flex)
        
        # Also show the target XY position
        target_x = data['current_x']
        target_y = data['current_y']
        
        # Convert to screen coordinates
        origin_screen = (self.origin_x, self.origin_y)
        elbow_screen = self._world_to_screen(x_elbow, y_elbow)
        end_screen = self._world_to_screen(x_end, y_end)
        target_screen = self._world_to_screen(target_x, target_y)
        
        # Draw target position (yellow crosshair)
        pygame.draw.circle(screen, self.YELLOW, target_screen, 10, 2)
        pygame.draw.line(screen, self.YELLOW, 
                        (target_screen[0] - 15, target_screen[1]),
                        (target_screen[0] + 15, target_screen[1]), 2)
        pygame.draw.line(screen, self.YELLOW,
                        (target_screen[0], target_screen[1] - 15),
                        (target_screen[0], target_screen[1] + 15), 2)
        
        # Draw links
        pygame.draw.line(screen, self.BLUE, origin_screen, elbow_screen, 8)  # L1
        pygame.draw.line(screen, self.CYAN, elbow_screen, end_screen, 6)     # L2
        
        # Draw joints
        pygame.draw.circle(screen, self.WHITE, origin_screen, 12)
        pygame.draw.circle(screen, self.RED, origin_screen, 8)      # Shoulder
        pygame.draw.circle(screen, self.WHITE, elbow_screen, 10)
        pygame.draw.circle(screen, self.ORANGE, elbow_screen, 6)    # Elbow
        pygame.draw.circle(screen, self.WHITE, end_screen, 10)
        pygame.draw.circle(screen, self.GREEN, end_screen, 6)       # End effector
        
        # Draw wrist orientation indicator
        wrist_flex = targets.get('wrist_flex', 0.0)
        pitch = data.get('pitch', 0.0)
        wrist_angle = shoulder_lift + elbow_flex + wrist_flex
        wrist_len = 0.05 * self.scale
        wrist_end_x = end_screen[0] + wrist_len * math.cos(-wrist_angle + math.pi/2)
        wrist_end_y = end_screen[1] + wrist_len * math.sin(-wrist_angle + math.pi/2)
        pygame.draw.line(screen, self.MAGENTA, end_screen, (int(wrist_end_x), int(wrist_end_y)), 4)
    
    def _draw_info_panel(self, screen, font, small_font, data):
        """Draw joint angles and position info."""
        if data is None:
            return
        
        panel_x, panel_y = 10, 10
        line_height = 20
        
        # Background
        panel_rect = pygame.Rect(5, 5, 340, 300)
        pygame.draw.rect(screen, (0, 0, 0, 200), panel_rect)
        pygame.draw.rect(screen, self.WHITE, panel_rect, 1)
        
        targets = data['target_positions']
        current_x = data['current_x']
        current_y = data['current_y']
        pitch = data.get('pitch', 0.0)
        
        # Compute FK position for comparison
        shoulder_lift = targets.get('shoulder_lift', 0.0)
        elbow_flex = targets.get('elbow_flex', 0.0)
        fk_x, fk_y = self.kinematics.forward_kinematics(shoulder_lift, elbow_flex)
        
        # Compute elbow angle (interior angle at elbow joint)
        # From IK: cos_angle = (l1^2 + l2^2 - r^2) / (2*l1*l2)
        l1, l2 = self.kinematics.l1, self.kinematics.l2
        r = math.sqrt(fk_x**2 + fk_y**2)
        if r > 0:
            cos_elbow = (l1**2 + l2**2 - r**2) / (2 * l1 * l2)
            cos_elbow = max(-1.0, min(1.0, cos_elbow))
            elbow_interior_angle = math.acos(cos_elbow)
        else:
            elbow_interior_angle = math.pi / 2
        
        lines = [
            (f"=== {self.prefix.upper()} ARM KINEMATICS ===", self.YELLOW),
            ("", self.WHITE),
            ("Target XY (IK input):", self.CYAN),
            (f"  X: {current_x:+.4f} m  ({current_x*100:+.2f} cm)", self.WHITE),
            (f"  Y: {current_y:+.4f} m  ({current_y*100:+.2f} cm)", self.WHITE),
            ("", self.WHITE),
            ("FK Result (from joint angles):", self.GREEN),
            (f"  X: {fk_x:+.4f} m  ({fk_x*100:+.2f} cm)", self.WHITE),
            (f"  Y: {fk_y:+.4f} m  ({fk_y*100:+.2f} cm)", self.WHITE),
            (f"  Reach: {r*100:.2f} cm  Elbow: {math.degrees(elbow_interior_angle):.1f}°", self.CYAN),
            (f"  Error: {math.sqrt((fk_x-current_x)**2 + (fk_y-current_y)**2)*100:.3f} cm", self.RED if abs(fk_x-current_x) > 0.01 or abs(fk_y-current_y) > 0.01 else self.GREEN),
            ("", self.WHITE),
            ("Joint Angles (radians / degrees):", self.ORANGE),
            (f"  shoulder_pan:  {targets.get('shoulder_pan', 0):+.4f} / {math.degrees(targets.get('shoulder_pan', 0)):+.1f}°", self.WHITE),
            (f"  shoulder_lift: {shoulder_lift:+.4f} / {math.degrees(shoulder_lift):+.1f}°", self.WHITE),
            (f"  elbow_flex:    {elbow_flex:+.4f} / {math.degrees(elbow_flex):+.1f}°", self.WHITE),
            (f"  wrist_flex:    {targets.get('wrist_flex', 0):+.4f} / {math.degrees(targets.get('wrist_flex', 0)):+.1f}°", self.WHITE),
            (f"  wrist_roll:    {targets.get('wrist_roll', 0):+.4f} / {math.degrees(targets.get('wrist_roll', 0)):+.1f}°", self.WHITE),
            (f"  gripper:       {targets.get('gripper', 0):+.4f} / {math.degrees(targets.get('gripper', 0)):+.1f}°", self.WHITE),
            ("", self.WHITE),
            (f"  pitch (wrist target): {pitch:+.4f} / {math.degrees(pitch):+.1f}°", self.MAGENTA),
        ]
        
        for i, (text, color) in enumerate(lines):
            surface = small_font.render(text, True, color)
            screen.blit(surface, (panel_x, panel_y + i * line_height))
    
    def _draw_vr_input_panel(self, screen, font, small_font, data):
        """Draw VR controller input info."""
        panel_x = 10
        panel_y = 320
        line_height = 18
        
        # Background
        panel_rect = pygame.Rect(5, 315, 340, 200)
        pygame.draw.rect(screen, (0, 0, 0, 200), panel_rect)
        pygame.draw.rect(screen, self.WHITE, panel_rect, 1)
        
        lines = [("=== VR CONTROLLER INPUT ===", self.YELLOW)]
        
        if data and data.get('vr_goal'):
            vr = data['vr_goal']
            pos = vr.get('target_position')
            if pos is not None and len(pos) >= 3:
                lines.append((f"VR Position (raw):", self.CYAN))
                lines.append((f"  X: {pos[0]:+.4f} m", self.WHITE))
                lines.append((f"  Y: {pos[1]:+.4f} m", self.WHITE))
                lines.append((f"  Z: {pos[2]:+.4f} m", self.WHITE))
            else:
                lines.append(("VR Position: None", self.GRAY))
            
            lines.append(("", self.WHITE))
            lines.append((f"wrist_flex_deg: {vr.get('wrist_flex_deg', 'N/A')}", self.WHITE))
            lines.append((f"wrist_roll_deg: {vr.get('wrist_roll_deg', 'N/A')}", self.WHITE))
            
            meta = vr.get('metadata', {})
            lines.append(("", self.WHITE))
            lines.append((f"grip_active: {meta.get('grip_active', meta.get('gripActive', False))}", self.GREEN if meta.get('grip_active', meta.get('gripActive', False)) else self.RED))
            lines.append((f"trigger: {meta.get('trigger', 0):.2f}", self.WHITE))
            
            prev = data.get('prev_vr_pos')
            if prev is not None:
                try:
                    lines.append(("", self.WHITE))
                    lines.append((f"prev_vr_pos: [{prev[0]:.3f}, {prev[1]:.3f}, {prev[2]:.3f}]", self.GRAY))
                except (TypeError, IndexError):
                    lines.append((f"prev_vr_pos: {prev}", self.GRAY))
        else:
            lines.append(("No VR data", self.GRAY))
        
        for i, (text, color) in enumerate(lines):
            surface = small_font.render(text, True, color)
            screen.blit(surface, (panel_x, panel_y + i * line_height))
    
    def _draw_controls_panel(self, screen, font):
        """Draw workspace bounds info."""
        panel_x = self.window_width - 280
        panel_y = 10
        line_height = 18
        
        # Background
        panel_rect = pygame.Rect(self.window_width - 285, 5, 280, 230)
        pygame.draw.rect(screen, (0, 0, 0, 200), panel_rect)
        pygame.draw.rect(screen, self.WHITE, panel_rect, 1)
        
        l1, l2 = self.kinematics.l1, self.kinematics.l2
        
        lines = [
            ("=== ARM CONFIGURATION ===", self.YELLOW),
            ("", self.WHITE),
            (f"L1 (upper arm): {l1:.4f} m ({l1*100:.2f} cm)", self.BLUE),
            (f"L2 (forearm):   {l2:.4f} m ({l2*100:.2f} cm)", self.CYAN),
            ("", self.WHITE),
            (f"Max reach: {l1+l2:.4f} m ({(l1+l2)*100:.2f} cm)", self.WHITE),
            (f"Min reach: {abs(l1-l2):.4f} m ({abs(l1-l2)*100:.2f} cm)", self.WHITE),
            ("", self.WHITE),
            ("Legend:", self.WHITE),
            ("  Yellow = Target XY (IK input)", self.YELLOW),
            ("  Green = End effector (FK)", self.GREEN),
            ("  Magenta = Wrist orientation", self.MAGENTA),
            ("", self.WHITE),
            ("Controls:", self.WHITE),
            ("  L = Left arm, R = Right arm", self.CYAN),
            ("  Tab = Toggle arm, Q/Esc = Quit", self.CYAN),
        ]
        
        for i, (text, color) in enumerate(lines):
            surface = font.render(text, True, color)
            screen.blit(surface, (panel_x, panel_y + i * line_height))


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
        
        # Create callback group for state subscription
        self.state_callback_group = ReentrantCallbackGroup()
        
        # Create publisher for robot low-level commands
        self.cmd_publisher = self.create_publisher(
            RobotLowCmd,
            '/alfie/robotlowcmd',
            1
        )
        
        # Create subscriber for robot low-level state
        self.state_subscriber = self.create_subscription(
            RobotLowState,
            '/alfie/robotlowstate',
            self.robot_state_callback,
            1,
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
        self.robot_cmd_state.back_cmd.position = 0.390
        self.robot_cmd_state.back_cmd.velocity = 0.2
        self.robot_cmd_state.back_cmd.acceleration = 0.1
        self.robot_cmd_state.cmd_vel = Twist()
        
        # Create timer for 100Hz publishing
        self.timer = self.create_timer(0.01, self.publish_robotlowcmd)  # 100Hz = 0.01s period
        
        # Create VR monitor
        self.vr_monitor = VRMonitor()
        
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
        
        # Debug visualizer (initialized after arm controllers)
        self.debug_visualizer = None
        self.enable_debug_viz = True  # Set to False to disable
        self.latest_left_controller_goal = None  # Store for visualization
        
        # Create timer for visualization data update at 30Hz (just pushes data to viz thread)
        self.viz_timer = self.create_timer(0.033, self.update_visualization)  # ~30Hz
        
        self.get_logger().info('Alfie Teleop VR Node initialized')
        
        # Start VR monitor in separate thread
        self.start_vr_monitor()
    
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
            kp=1
        )
        self.right_arm = SimpleTeleopArm(
            joint_map=RIGHT_ARM_JOINT_MAP,
            robotlowstate=robot_state,
            kinematics=self.kinematics_right,
            prefix='right',
            kp=1
        )
        self.head = SimpleHeadControl(
            joint_map=HEAD_JOINT_MAP,
            robotlowstate=robot_state,
            kp=1
        )

        # Call back calibration service
        # self.get_logger().info('Calling back calibration service...')
        # calibrate_client = self.create_client(BackRequestCalibration, '/alfie/low/calibrate_back')
        # if calibrate_client.wait_for_service(timeout_sec=5.0):
        #     request = BackRequestCalibration.Request()
        #     future = calibrate_client.call_async(request)
        #     rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        #     if future.result() is not None:
        #         if future.result().success:
        #             self.get_logger().info('Back calibration successful')
        #         else:
        #             self.get_logger().warn('Back calibration returned failure')
        #     else:
        #         self.get_logger().warn('Back calibration service call failed')
        # else:
        #     self.get_logger().warn('Back calibration service not available')
        
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
        
        # Initialize cmd_vel values to zero (will be updated if joystick input exists)
        # Don't create a new Twist object to avoid race condition with publisher
        self.robot_cmd_state.cmd_vel.linear.x = 0.0
        self.robot_cmd_state.cmd_vel.linear.y = 0.0
        self.robot_cmd_state.cmd_vel.linear.z = 0.0
        self.robot_cmd_state.cmd_vel.angular.x = 0.0
        self.robot_cmd_state.cmd_vel.angular.y = 0.0
        self.robot_cmd_state.cmd_vel.angular.z = 0.0
        
        # Extract rotation angles from headset goal metadata
        # The VR system sends rotation as {'x': pitch, 'y': yaw, 'z': roll}
        if hasattr(headset_goal, 'metadata') and headset_goal.metadata:
            rotation = headset_goal.metadata.get('rotation', {})
            
            if rotation:
                # Servo 12: Head Pan (yaw - rotation around Z-axis)
                # VR sends this as 'y' key
                if 'y' in rotation:
                    yaw_deg = float(rotation['y'])
                    yaw_rad = math.radians(yaw_deg)
                    self.robot_cmd_state.servo_cmd[12].target_location = yaw_rad
                
                # Servo 13: Head Tilt (pitch - rotation around Y-axis)
                # VR sends this as 'x' key
                if 'x' in rotation:
                    pitch_deg = float(rotation['x'])
                    pitch_rad = math.radians(pitch_deg)
                    self.robot_cmd_state.servo_cmd[13].target_location = pitch_rad
                
                # Servo 14: Head Roll (roll - rotation around X-axis)
                # VR sends this as 'z' key
                if 'z' in rotation:
                    roll_deg = float(rotation['z'])
                    roll_rad = math.radians(roll_deg)
                    self.robot_cmd_state.servo_cmd[14].target_location = roll_rad
        
        # Process joystick input for cmd_vel
        # Velocity limits
        MAX_LINEAR_VEL = 0.25  # m/s
        MAX_ANGULAR_VEL = 1.0  # rad/s
        
        # Left joystick controls linear velocity (forward/back and strafe left/right)
        if left_controller_goal and hasattr(left_controller_goal, 'metadata') and left_controller_goal.metadata:
            thumbstick = left_controller_goal.metadata.get('thumbstick', {})
            if thumbstick:
                # X-axis: strafe (positive = right, negative = left)
                # Y-axis: forward/back (positive = forward, negative = back)
                joy_x = float(thumbstick.get('x', 0.0))
                joy_y = float(thumbstick.get('y', 0.0))
                
                # Map joystick values (-1 to 1) to velocity limits
                self.robot_cmd_state.cmd_vel.linear.x = -joy_y * MAX_LINEAR_VEL  # forward/back (negated)
                self.robot_cmd_state.cmd_vel.linear.y = joy_x * MAX_LINEAR_VEL  # strafe left/right
                
                # Debug log
                if abs(joy_x) > 0.1 or abs(joy_y) > 0.1:
                    self.get_logger().info(f'Left thumbstick: x={joy_x:.2f}, y={joy_y:.2f} -> linear.x={self.robot_cmd_state.cmd_vel.linear.x:.2f}, linear.y={self.robot_cmd_state.cmd_vel.linear.y:.2f}')
        
        # Right joystick X-axis controls angular velocity (rotate left/right)
        # A/B buttons control back height
        if right_controller_goal and hasattr(right_controller_goal, 'metadata') and right_controller_goal.metadata:
            thumbstick = right_controller_goal.metadata.get('thumbstick', {})
            if thumbstick:
                # X-axis: rotation (positive = rotate right, negative = rotate left)
                joy_x = float(thumbstick.get('x', 0.0))
                
                # Map joystick value (-1 to 1) to angular velocity limit
                self.robot_cmd_state.cmd_vel.angular.z = joy_x * MAX_ANGULAR_VEL  # Rotation (removed negation)
                
                # Debug log
                if abs(joy_x) > 0.1:
                    self.get_logger().info(f'Right thumbstick: x={joy_x:.2f} -> angular.z={self.robot_cmd_state.cmd_vel.angular.z:.2f}')
            
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
                self.get_logger().info(f'Back height: {new_back_pos:.3f} ({"B" if button_b else "A"} pressed)')
        
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
                    self.get_logger().info('Left grip pressed - starting arm tracking')
                    # Reset delta tracking by clearing previous position
                    if hasattr(self.left_arm, 'prev_vr_pos'):
                        delattr(self.left_arm, 'prev_vr_pos')
                    if hasattr(self.left_arm, 'prev_wrist_flex'):
                        delattr(self.left_arm, 'prev_wrist_flex')
                    if hasattr(self.left_arm, 'prev_wrist_roll'):
                        delattr(self.left_arm, 'prev_wrist_roll')
                
                # Detect grip release
                if not left_grip_active and self.left_grip_active:
                    self.get_logger().info('Left grip released - stopping arm tracking')
                
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
                    self.get_logger().info('Right grip pressed - starting arm tracking')
                    if hasattr(self.right_arm, 'prev_vr_pos'):
                        delattr(self.right_arm, 'prev_vr_pos')
                    if hasattr(self.right_arm, 'prev_wrist_flex'):
                        delattr(self.right_arm, 'prev_wrist_flex')
                    if hasattr(self.right_arm, 'prev_wrist_roll'):
                        delattr(self.right_arm, 'prev_wrist_roll')
                
                # Detect grip release
                if not right_grip_active and self.right_grip_active:
                    self.get_logger().info('Right grip released - stopping arm tracking')
                
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