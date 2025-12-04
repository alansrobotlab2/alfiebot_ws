#!/usr/bin/env python3
"""
Arm Debug Visualizer - Pygame-based debug visualization for arm kinematics.
Optional module that runs in a separate thread to avoid interfering with robotlowcmd timing.
"""

import math
import threading
import time

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
