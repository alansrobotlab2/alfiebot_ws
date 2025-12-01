#!/usr/bin/env python3
"""
Alfie Arm Kinematics Test & Visualization Tool
===============================================

This interactive tool helps you test and validate the inverse kinematics (IK) 
and forward kinematics (FK) for your robot arm configuration before deploying
to real hardware.

PURPOSE:
--------
- Visualize how joint angles affect end-effector position
- Test IK solutions by clicking target positions
- Verify your arm's zero position configuration
- Debug kinematics issues without risk to hardware
- Tune L1/L2 link lengths to match your physical arm

ARM CONFIGURATION (Inverted/Hanging Arm):
------------------------------------------
This is configured for an INVERTED arm where:
- The shoulder is at the TOP (origin)
- The arm hangs DOWN from the shoulder
- Zero position: L1 points straight DOWN, L2 points horizontally FORWARD

  Shoulder (origin)
      |
      | L1 (upper arm)
      |
    Elbow
      -------- L2 (forearm) -------> End Effector
      
CONTROLS:
---------
- Left/Right Arrow: Adjust shoulder_lift angle
- Up/Down Arrow: Adjust elbow_flex angle  
- Mouse Click: Use IK to move end-effector to clicked position
- R: Reset to zero position
- +/-: Adjust angle step size
- Q/ESC: Quit

WHAT TO LOOK FOR:
-----------------
1. At zero position (R key), L1 should point straight down, L2 horizontal
2. Clicking anywhere should move the green end-effector to that spot
3. The arm should stay within the workspace circles (min/max reach)
4. Joint angles should change smoothly as you click around

CUSTOMIZATION:
--------------
Edit the AlfieArmKinematics initialization to match your arm:
  - l1: Length of upper arm (shoulder to elbow) in meters
  - l2: Length of forearm (elbow to end-effector) in meters

Example: kinematics = AlfieArmKinematics(l1=0.25, l2=0.20)
"""

import pygame
import math
import sys

# Add parent directory to path for imports
sys.path.insert(0, '/home/alansrobotlab/Projects/alfiebot_ws/src/alfie_vr/alfie_vr')
from kinematics import AlfieArmKinematics


# Constants
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
FPS = 60

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

# Scale: pixels per meter
SCALE = 800  # 1 meter = 800 pixels

# Origin position on screen (upper area since arm hangs down)
ORIGIN_X = WINDOW_WIDTH // 2
ORIGIN_Y = 120  # Near top since arm hangs down


class ArmVisualizer:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Alfie Arm Kinematics Tester (Inverted)")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 20)
        
        # Initialize kinematics
        self.kinematics = AlfieArmKinematics(l1=0.2387, l2=0.2160)
        
        # Current joint angles (in radians)
        self.shoulder_lift = 0.0
        self.elbow_flex = 0.0
        
        # Angle step for keyboard control
        self.angle_step = math.radians(5)  # 5 degrees per key press
        
        # Mouse target for IK
        self.mouse_target = None
        
    def world_to_screen(self, x, y):
        """Convert world coordinates (meters) to screen coordinates (pixels)."""
        # +X goes right, +Y goes up in world, but down on screen
        screen_x = ORIGIN_X + x * SCALE
        screen_y = ORIGIN_Y - y * SCALE  # Flip Y for screen coordinates
        return int(screen_x), int(screen_y)
    
    def screen_to_world(self, screen_x, screen_y):
        """Convert screen coordinates to world coordinates."""
        x = (screen_x - ORIGIN_X) / SCALE
        y = -(screen_y - ORIGIN_Y) / SCALE  # Flip Y back
        return x, y
    
    def draw_grid(self):
        """Draw a reference grid."""
        # Draw grid lines every 0.05 meters (5 cm)
        grid_spacing = 0.05 * SCALE
        
        # Vertical lines
        for i in range(-10, 11):
            x = ORIGIN_X + i * grid_spacing
            color = GRAY if i != 0 else WHITE
            pygame.draw.line(self.screen, color, (x, 0), (x, WINDOW_HEIGHT), 1 if i != 0 else 2)
            
        # Horizontal lines  
        for i in range(-10, 11):
            y = ORIGIN_Y - i * grid_spacing
            color = GRAY if i != 0 else WHITE
            pygame.draw.line(self.screen, color, (0, y), (WINDOW_WIDTH, y), 1 if i != 0 else 2)
            
        # Draw axis labels
        label_x = self.small_font.render("+X", True, WHITE)
        label_y = self.small_font.render("+Y", True, WHITE)
        self.screen.blit(label_x, (WINDOW_WIDTH - 30, ORIGIN_Y + 5))
        self.screen.blit(label_y, (ORIGIN_X + 5, 10))
    
    def draw_workspace(self):
        """Draw the reachable workspace boundary."""
        l1, l2 = self.kinematics.l1, self.kinematics.l2
        r_max = (l1 + l2) * SCALE
        r_min = abs(l1 - l2) * SCALE
        
        # Draw max reach circle
        pygame.draw.circle(self.screen, DARK_GRAY, (ORIGIN_X, ORIGIN_Y), int(r_max), 1)
        # Draw min reach circle
        pygame.draw.circle(self.screen, DARK_GRAY, (ORIGIN_X, ORIGIN_Y), int(r_min), 1)
        
    def draw_arm(self):
        """Draw the 2-link arm based on current joint angles."""
        # Get elbow position
        x_elbow, y_elbow = self.kinematics.get_elbow_position(self.shoulder_lift)
        
        # Get end effector position
        x_end, y_end = self.kinematics.forward_kinematics(self.shoulder_lift, self.elbow_flex)
        
        # Convert to screen coordinates
        origin_screen = (ORIGIN_X, ORIGIN_Y)
        elbow_screen = self.world_to_screen(x_elbow, y_elbow)
        end_screen = self.world_to_screen(x_end, y_end)
        
        # Draw links
        # Link 1 (upper arm) - BLUE
        pygame.draw.line(self.screen, BLUE, origin_screen, elbow_screen, 8)
        # Link 2 (forearm) - CYAN
        pygame.draw.line(self.screen, CYAN, elbow_screen, end_screen, 6)
        
        # Draw joints
        # Shoulder joint - RED
        pygame.draw.circle(self.screen, WHITE, origin_screen, 12)
        pygame.draw.circle(self.screen, RED, origin_screen, 8)
        
        # Elbow joint - ORANGE
        pygame.draw.circle(self.screen, WHITE, elbow_screen, 10)
        pygame.draw.circle(self.screen, ORANGE, elbow_screen, 6)
        
        # End effector - GREEN
        pygame.draw.circle(self.screen, WHITE, end_screen, 10)
        pygame.draw.circle(self.screen, GREEN, end_screen, 6)
        
        return x_end, y_end, x_elbow, y_elbow
    
    def draw_mouse_target(self):
        """Draw the mouse target position."""
        if self.mouse_target:
            target_screen = self.world_to_screen(self.mouse_target[0], self.mouse_target[1])
            pygame.draw.circle(self.screen, YELLOW, target_screen, 8, 2)
            pygame.draw.line(self.screen, YELLOW, 
                           (target_screen[0] - 12, target_screen[1]), 
                           (target_screen[0] + 12, target_screen[1]), 2)
            pygame.draw.line(self.screen, YELLOW, 
                           (target_screen[0], target_screen[1] - 12), 
                           (target_screen[0], target_screen[1] + 12), 2)
    
    def draw_info(self, x_end, y_end):
        """Draw information panel."""
        info_x = 10
        info_y = 10
        line_height = 22
        
        # Background panel
        panel_rect = pygame.Rect(5, 5, 320, 220)
        pygame.draw.rect(self.screen, (0, 0, 0, 180), panel_rect)
        pygame.draw.rect(self.screen, WHITE, panel_rect, 1)
        
        texts = [
            f"Joint Angles:",
            f"  shoulder_lift: {math.degrees(self.shoulder_lift):+7.2f}° ({self.shoulder_lift:+.4f} rad)",
            f"  elbow_flex:    {math.degrees(self.elbow_flex):+7.2f}° ({self.elbow_flex:+.4f} rad)",
            f"",
            f"End Effector Position:",
            f"  X: {x_end:+.4f} m ({x_end*100:+.2f} cm)",
            f"  Y: {y_end:+.4f} m ({y_end*100:+.2f} cm)",
            f"",
            f"Link Lengths: L1={self.kinematics.l1:.4f}m, L2={self.kinematics.l2:.4f}m",
            f"Angle step: {math.degrees(self.angle_step):.1f}°",
        ]
        
        for i, text in enumerate(texts):
            color = YELLOW if i == 0 or i == 4 else WHITE
            surface = self.small_font.render(text, True, color)
            self.screen.blit(surface, (info_x, info_y + i * line_height))
    
    def draw_controls(self):
        """Draw control instructions."""
        controls_x = WINDOW_WIDTH - 280
        controls_y = 10
        line_height = 20
        
        # Background panel
        panel_rect = pygame.Rect(WINDOW_WIDTH - 285, 5, 280, 200)
        pygame.draw.rect(self.screen, (0, 0, 0, 180), panel_rect)
        pygame.draw.rect(self.screen, WHITE, panel_rect, 1)
        
        controls = [
            "Controls:",
            "  Left/Right: shoulder_lift",
            "  Up/Down: elbow_flex",
            "  Mouse Click: IK to position",
            "  R: Reset to zero position",
            "  +/-: Adjust angle step",
            "  Q/ESC: Quit",
            "",
            "Zero Position:",
            "  shoulder=0: L1 points DOWN",
            "  elbow=0: L2 perpendicular (+X)",
        ]
        
        for i, text in enumerate(controls):
            color = YELLOW if i == 0 or i == 8 else LIGHT_GRAY
            surface = self.small_font.render(text, True, color)
            self.screen.blit(surface, (controls_x, controls_y + i * line_height))
    
    def handle_events(self):
        """Handle pygame events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    return False
                
                # Joint control
                if event.key == pygame.K_LEFT:
                    self.shoulder_lift -= self.angle_step
                elif event.key == pygame.K_RIGHT:
                    self.shoulder_lift += self.angle_step
                elif event.key == pygame.K_DOWN:
                    self.elbow_flex -= self.angle_step
                elif event.key == pygame.K_UP:
                    self.elbow_flex += self.angle_step
                
                # Reset
                elif event.key == pygame.K_r:
                    self.shoulder_lift = 0.0
                    self.elbow_flex = 0.0
                    self.mouse_target = None
                
                # Adjust angle step
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                    self.angle_step = min(math.radians(15), self.angle_step + math.radians(1))
                elif event.key == pygame.K_MINUS:
                    self.angle_step = max(math.radians(1), self.angle_step - math.radians(1))
            
            # Mouse click for IK
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    mouse_x, mouse_y = event.pos
                    world_x, world_y = self.screen_to_world(mouse_x, mouse_y)
                    self.mouse_target = (world_x, world_y)
                    
                    # Try IK
                    try:
                        shoulder, elbow = self.kinematics.inverse_kinematics(world_x, world_y)
                        self.shoulder_lift = shoulder
                        self.elbow_flex = elbow
                        print(f"IK: target=({world_x:.4f}, {world_y:.4f}) -> shoulder={math.degrees(shoulder):.2f}°, elbow={math.degrees(elbow):.2f}°")
                    except Exception as e:
                        print(f"IK failed: {e}")
        
        return True
    
    def run(self):
        """Main loop."""
        running = True
        
        print()
        print("=" * 70)
        print("  ALFIE ARM KINEMATICS TEST & VISUALIZATION TOOL")
        print("=" * 70)
        print()
        print("  This tool lets you interactively test your arm's kinematics")
        print("  before deploying to real hardware.")
        print()
        print("  ARM CONFIGURATION (Inverted/Hanging):")
        print("  --------------------------------------")
        print(f"    L1 (upper arm):  {self.kinematics.l1:.4f} m  ({self.kinematics.l1*100:.2f} cm)")
        print(f"    L2 (forearm):    {self.kinematics.l2:.4f} m  ({self.kinematics.l2*100:.2f} cm)")
        print(f"    Max reach:       {self.kinematics.l1 + self.kinematics.l2:.4f} m")
        print(f"    Min reach:       {abs(self.kinematics.l1 - self.kinematics.l2):.4f} m")
        print()
        print("  ZERO POSITION:")
        print("    - shoulder_lift=0: L1 points straight DOWN")
        print("    - elbow_flex=0:    L2 perpendicular, pointing FORWARD (+X)")
        print()
        print("  CONTROLS:")
        print("    Arrow Keys     - Manually adjust joint angles")
        print("    Mouse Click    - IK: move end-effector to target")
        print("    R              - Reset to zero position")
        print("    +/-            - Change angle step size")
        print("    Q / ESC        - Quit")
        print()
        print("  TIP: Click around the workspace to test IK accuracy!")
        print("       The end-effector (green) should land on your click.")
        print("=" * 70)
        print()
        
        while running:
            running = self.handle_events()
            
            # Clear screen
            self.screen.fill(BLACK)
            
            # Draw elements
            self.draw_grid()
            self.draw_workspace()
            self.draw_mouse_target()
            x_end, y_end, _, _ = self.draw_arm()
            self.draw_info(x_end, y_end)
            self.draw_controls()
            
            # Update display
            pygame.display.flip()
            self.clock.tick(FPS)
        
        pygame.quit()


def main():
    visualizer = ArmVisualizer()
    visualizer.run()


if __name__ == "__main__":
    main()
