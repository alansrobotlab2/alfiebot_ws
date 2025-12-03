import math
import numpy as np
from typing import List, Union, Tuple
from alfie_msgs.msg import RobotLowCmd, RobotLowState, ServoCmd



class SimpleTeleopArm:
    """
    A class for controlling a robot arm using VR input with delta action control.
    
    This class provides inverse kinematics-based arm control with proportional control
    for smooth movement and gripper operations based on VR controller input.
    """
    
    def __init__(self, joint_map, robotlowstate, kinematics, prefix="right", kp=1):
        """
        Initialize the SimpleTeleopArm with joint mapping and initial state.

        Args:
            joint_map (dict): Mapping of generic joint names to specific robot joint names.
            robotlowstate (object): RobotLowState message containing the current state of the robot.
            kinematics (object): Kinematics model for inverse kinematics calculations.
            prefix (str, optional): Prefix for the arm (e.g., "left" or "right"). Defaults to "right".
            kp (float, optional): Proportional gain for the control loop. Defaults to 1.
        """
        self.joint_map = joint_map
        self.prefix = prefix
        self.kp = kp
        self.kinematics = kinematics
        
        # set offset to 0 if prefix is left otherwise 7 for right
        if prefix == 'left':
            offset = 0
        else:
            offset = 6

        # Initial joint positions - adapted for XLerobot observation format
        self.joint_positions = {
            "shoulder_pan": robotlowstate.servo_state[offset + 0].current_location,
            "shoulder_lift": robotlowstate.servo_state[offset + 1].current_location,
            "elbow_flex": robotlowstate.servo_state[offset + 2].current_location,
            "wrist_flex": robotlowstate.servo_state[offset + 3].current_location,
            "wrist_roll": robotlowstate.servo_state[offset + 4].current_location,
            "gripper": robotlowstate.servo_state[offset + 5].current_location,
        }
        
        # Initialize pitch for wrist orientation
        self.pitch = 0.0
        
        # Delta control state variables for VR input
        self.last_vr_time = 0.0
        self.vr_deadzone = 0.001  # Minimum movement threshold
        self.max_delta_per_frame = 0.005  # Maximum position change per frame
        
        # Set step size
        self.degree_step = 2
        self.xy_step = 0.005
        
        # Define zero position first
        self.zero_pos = {
            'shoulder_pan':  0.0000,
            'shoulder_lift': 0.1089,
            'elbow_flex':   -1.4864,
            'wrist_flex':   -0.1442,
            'wrist_roll':    0.0000,
            'gripper':       0.0000
        }
        
        # P control target positions, set to zero position
        self.target_positions = self.zero_pos.copy()
        
        # Derive current_x and current_y from zero_pos using forward kinematics
        self.current_x, self.current_y = self.kinematics.forward_kinematics(
            self.zero_pos["shoulder_lift"],
            self.zero_pos["elbow_flex"]
        )


    def move_to_zero_position(self, robot):
        """
        Move the arm to its defined zero position.

        Resets internal state variables including current Cartesian coordinates,
        pitch, and delta control timers. Sends the zero position action to the robot.

        Args:
            robot (object): The robot instance to send actions to.
        """
        print(f"[{self.prefix}] Moving to Zero Position: {self.zero_pos} ......")
        self.target_positions = self.zero_pos.copy()
        
        # Derive current_x and current_y from zero_pos using forward kinematics
        self.current_x, self.current_y = self.kinematics.forward_kinematics(
            self.zero_pos["shoulder_lift"],
            self.zero_pos["elbow_flex"]
        )
        self.pitch = self.zero_pos["wrist_flex"]
        
        # Reset delta control state
        self.last_vr_time = 0.0
        
        # Explicitly set wrist_flex
        # self.target_positions["wrist_flex"] = 0.0
        
        action = self.p_control_action(robot)
        robot.send_action(action)

    def handle_vr_input(self, vr_goal, gripper_state):
        """
        Handle VR input with delta action control - incremental position updates.
        
        Args:
            vr_goal: VR controller goal data containing target position and orientations
            gripper_state: Current gripper state (not used in current implementation)
        """
        if vr_goal is None:
            return
        
        # VR goal contains: target_position [x, y, z], wrist_roll_deg, wrist_flex_deg, gripper_closed
        if not hasattr(vr_goal, 'target_position') or vr_goal.target_position is None:
            return
            
        # Extract VR position data
        # Get current VR position
        current_vr_pos = vr_goal.target_position  # [x, y, z] in meters
        
        # Initialize previous VR position if not set
        if not hasattr(self, 'prev_vr_pos'):
            self.prev_vr_pos = current_vr_pos
            return  # Skip first frame to establish baseline
        
        print(current_vr_pos)
        
        # Calculate relative change (delta) from previous frame
        # Scale factors for converting VR movement to robot workspace
        vr_x = (current_vr_pos[0] - self.prev_vr_pos[0]) * 80 # * 220  # Scale for the shoulder rotate
        vr_y = (current_vr_pos[1] - self.prev_vr_pos[1]) * 40 # * 70   # Scale for arm reach (Y)
        vr_z = (current_vr_pos[2] - self.prev_vr_pos[2]) * 40 # * 70   # Scale for arm reach (Z->X)

        # Update previous position for next frame
        self.prev_vr_pos = current_vr_pos
        
        # Delta control parameters - adjust these for sensitivity
        pos_scale = 0.05  # Position sensitivity scaling
        angle_scale = 0.07  # Angle sensitivity scaling (radians, ~4 degrees)
        delta_limit = 0.02  # Maximum delta per update (meters)
        angle_limit = 0.14  # Maximum angle delta per update (radians, ~8 degrees)
        
        delta_x = vr_x * pos_scale
        delta_y = vr_y * pos_scale  
        delta_z = vr_z * pos_scale
        
        # Limit delta values to prevent sudden movements
        delta_x = max(-delta_limit, min(delta_limit, delta_x))
        delta_y = max(-delta_limit, min(delta_limit, delta_y))
        delta_z = max(-delta_limit, min(delta_limit, delta_z))
        
        # Update workspace position
        # VR coordinate system (WebXR): X=left(-)/right(+), Y=down(-)/up(+), Z=forward(-)/back(+)
        # Kinematics coordinate system: X=forward reach, Y=vertical (negative=down)
        # Fix: swap X and Y mapping to match physical motion
        self.current_y += delta_z  # VR Z (forward/back) -> robot y (vertical) - SWAPPED
        self.current_x += -delta_y   # VR Y (up/down) -> robot x (reach) - SWAPPED
        
        # Constrain x to be positive (arm can't reach behind shoulder)
        self.current_x = max(0.01, self.current_x)
        
        # Debug: show position changes
        if abs(delta_y) > 0.001 or abs(delta_z) > 0.001:
            print(f"[{self.prefix}] delta_y={delta_y:.4f}, delta_z={delta_z:.4f} -> x={self.current_x:.4f}, y={self.current_y:.4f}")

        # Handle wrist angles with delta control - use relative changes (all in radians)
        if hasattr(vr_goal, 'wrist_flex_deg') and vr_goal.wrist_flex_deg is not None:
            # Initialize previous wrist_flex if not set
            if not hasattr(self, 'prev_wrist_flex'):
                self.prev_wrist_flex = vr_goal.wrist_flex_deg
                return
            
            # Calculate relative change from previous frame (convert VR degrees to radians)
            delta_pitch = math.radians(vr_goal.wrist_flex_deg - self.prev_wrist_flex) * angle_scale / 0.07
            delta_pitch = max(-angle_limit, min(angle_limit, delta_pitch))
            self.pitch += delta_pitch
            self.pitch = max(-math.pi/2, min(math.pi/2, self.pitch))  # Limit pitch range (±90°)
            
            # Update previous value for next frame
            self.prev_wrist_flex = vr_goal.wrist_flex_deg
        
        if hasattr(vr_goal, 'wrist_roll_deg') and vr_goal.wrist_roll_deg is not None:
            # Initialize previous wrist_roll if not set
            if not hasattr(self, 'prev_wrist_roll'):
                self.prev_wrist_roll = vr_goal.wrist_roll_deg
                return
            
            # Convert VR degrees to radians for delta
            delta_roll = math.radians(vr_goal.wrist_roll_deg - self.prev_wrist_roll) * angle_scale / 0.07
            delta_roll = max(-angle_limit, min(angle_limit, delta_roll))
            
            current_roll = self.target_positions.get("wrist_roll", 0.0)
            new_roll = current_roll - delta_roll  # Negated to fix direction
            new_roll = max(-math.pi/2, min(math.pi/2, new_roll))  # Limit roll range (±90°)
            self.target_positions["wrist_roll"] = new_roll
            
            # Update previous value for next frame
            self.prev_wrist_roll = vr_goal.wrist_roll_deg
        
        # VR X axis controls shoulder_pan joint (delta control)
        if abs(delta_x) > 0.001:  # Only update if significant movement
            x_scale = 3.5  # Scale factor for shoulder pan (radians)
            delta_pan = delta_x * x_scale
            delta_pan = max(-angle_limit, min(angle_limit, delta_pan))
            current_pan = self.target_positions.get("shoulder_pan", 0.0)
            new_pan = current_pan - delta_pan  # Negated to fix direction
            new_pan = max(-math.pi/4, min(math.pi/4, new_pan))  # Limit pan range (±45°)
            self.target_positions["shoulder_pan"] = new_pan
        
        try:
            joint2_target, joint3_target = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
            # IK returns radians - use directly
            
            # Debug: print current position and IK results
            print(f"[{self.prefix}] x={self.current_x:.4f}, y={self.current_y:.4f} -> shoulder_lift={math.degrees(joint2_target):.1f}°, elbow_flex={math.degrees(joint3_target):.1f}°")
            
            # Smooth transition to new joint positions, Smoothing factor 0-1, higher = faster response
            alpha = 0.3
            self.target_positions["shoulder_lift"] = (1-alpha) * self.target_positions.get("shoulder_lift", 0.0) + alpha * joint2_target
            self.target_positions["elbow_flex"] = (1-alpha) * self.target_positions.get("elbow_flex", 0.0) + alpha * joint3_target
        except Exception as e:
            print(f"[{self.prefix}] VR IK failed: {e}")
        
        # Calculate wrist_flex to maintain end-effector orientation
        self.target_positions["wrist_flex"] = (-self.target_positions["shoulder_lift"] - 
                                               self.target_positions["elbow_flex"] + self.pitch)
   
        # Handle gripper state directly (in radians)
        if vr_goal.metadata.get('trigger', 0) > 0.5:
            self.target_positions["gripper"] = 0.785  # ~45 degrees in radians
        else:
            self.target_positions["gripper"] = 0.0

    def p_control_action(self, robot):
        """
        Generate proportional control action based on target positions.
        
        Args:
            robot: Robot instance to get current observations
            
        Returns:
            dict: Action dictionary with position commands for each joint
        """
        obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action


class SimpleHeadControl:
    """
    A class for controlling robot head motors using VR thumbstick input.
    
    Provides simple head movement control with proportional control for smooth operation.
    """
    
    def __init__(self, joint_map, robotlowstate, kp=1):
        """
        Initialize the SimpleHeadControl.

        Args:
            robotlowstate (object): RobotLowState instance from the robot.
            kp (float, optional): Proportional gain for the control loop. Defaults to 1.
        """
        self.joint_map = joint_map
        self.kp = kp
        self.degree_step = 2  # Move 2 degrees each time
        # Initialize head motor positions
        self.target_positions = {
            "head_yaw": robotlowstate.servo_state[12].current_location,
            "head_pitch": robotlowstate.servo_state[13].current_location,
            "head_roll": robotlowstate.servo_state[14].current_location,
        }
        self.zero_pos = {"head_yaw": 0.0, "head_pitch": 0.0, "head_roll": 0.0}

    def handle_vr_input(self, vr_goal):
        """
        Process VR input to update head motor target positions.

        Uses the VR controller's thumbstick input to incrementally adjust
        the pan (head_motor_1) and tilt (head_motor_2) of the head.

        Args:
            vr_goal (object): VR controller goal data containing metadata with thumbstick values.
        """
        # Map VR input to head motor targets
        rotation = vr_goal.metadata.get('rotation', {})
        if rotation:
            thumb_x = rotation.get('x', 0)
            thumb_y = rotation.get('y', 0)
            if abs(thumb_x) > 0.1:
                if thumb_x > 0:
                    self.target_positions["head_motor_1"] += self.degree_step
                else:
                    self.target_positions["head_motor_1"] -= self.degree_step
            if abs(thumb_y) > 0.1:
                if thumb_y > 0:
                    self.target_positions["head_motor_2"] += self.degree_step
                else:
                    self.target_positions["head_motor_2"] -= self.degree_step
                    
    def move_to_zero_position(self, robot):
        """
        Move the head motors to their zero positions.

        Resets the target positions to zero and sends the action to the robot.

        Args:
            robot (object): The robot instance to send actions to.
        """
        print(f"[HEAD] Moving to Zero Position: {self.zero_pos} ......")
        self.target_positions = self.zero_pos.copy()
        action = self.p_control_action(robot)
        robot.send_action(action)

    def p_control_action(self, robot):
        """
        Generate proportional control action for head motors.
        
        Args:
            robot: Robot instance to get current observations
            
        Returns:
            dict: Action dictionary with position commands for head motors
        """
        obs = robot.get_observation()
        action = {}
        for motor in self.target_positions:
            current = obs.get(f"{self.joint_map[motor]}.pos", 0.0)
            error = self.target_positions[motor] - current
            control = self.kp * error
            action[f"{self.joint_map[motor]}.pos"] = current + control
        return action


class AlfieArmKinematics:
    """
    A class to represent the kinematics of an inverted 2-link robot arm.
    
    Coordinate system:
    - Origin at shoulder joint
    - +X points forward (horizontal)
    - +Y points up (vertical)
    
    Zero position configuration:
    - shoulder_lift = 0: L1 points straight DOWN (-Y direction)
    - elbow_flex = 0: L2 is perpendicular to L1, pointing forward (+X direction)
    
    All public methods use radians for input/output.
    """

    def __init__(self, l1=0.2387, l2=0.2160):
        self.l1 = l1  # Length of the first link (upper arm)
        self.l2 = l2  # Length of the second link (lower arm)

    def forward_kinematics(self, shoulder_lift, elbow_flex, l1=None, l2=None):
        """
        Calculate forward kinematics for the inverted 2-link arm.
        
        Parameters:
            shoulder_lift: Shoulder joint angle in radians (0 = L1 pointing down)
            elbow_flex: Elbow joint angle in radians (0 = L2 perpendicular to L1)
            l1: Upper arm length (default uses instance value)
            l2: Lower arm length (default uses instance value)
            
        Returns:
            x, y: End effector coordinates in meters
        """
        if l1 is None:
            l1 = self.l1
        if l2 is None:
            l2 = self.l2
        
        # At shoulder_lift=0, L1 points down (-pi/2 from +X axis)
        # Positive shoulder_lift rotates L1 forward (toward +X)
        theta1 = -math.pi/2 + shoulder_lift
        
        # Elbow position
        x_elbow = l1 * math.cos(theta1)
        y_elbow = l1 * math.sin(theta1)
        
        # At elbow_flex=0, L2 is perpendicular to L1 (rotated +90° from L1)
        # This means L2 points in the direction of theta1 + pi/2
        # Positive elbow_flex bends the arm (rotates L2 toward L1)
        theta2 = theta1 + math.pi/2 - elbow_flex
        
        # End effector position
        x = x_elbow + l2 * math.cos(theta2)
        y = y_elbow + l2 * math.sin(theta2)
        
        return x, y
    
    def get_elbow_position(self, shoulder_lift, l1=None):
        """
        Get the elbow joint position for visualization.
        
        Parameters:
            shoulder_lift: Shoulder joint angle in radians
            l1: Upper arm length (default uses instance value)
            
        Returns:
            x_elbow, y_elbow: Elbow coordinates in meters
        """
        if l1 is None:
            l1 = self.l1
        
        theta1 = -math.pi/2 + shoulder_lift
        x_elbow = l1 * math.cos(theta1)
        y_elbow = l1 * math.sin(theta1)
        
        return x_elbow, y_elbow

    def inverse_kinematics(self, x, y, l1=None, l2=None):
        """
        Calculate inverse kinematics for the inverted 2-link arm.
        
        Parameters:
            x: End effector x coordinate (forward)
            y: End effector y coordinate (up, typically negative for inverted arm)
            l1: Upper arm length (default uses instance value)
            l2: Lower arm length (default uses instance value)
            
        Returns:
            shoulder_lift, elbow_flex: Joint angles in radians
        """
        if l1 is None:
            l1 = self.l1
        if l2 is None:
            l2 = self.l2
        
        # Calculate distance from origin to target point
        r = math.sqrt(x**2 + y**2)
        r_max = l1 + l2
        r_min = abs(l1 - l2)
        
        # Clamp to reachable workspace
        if r > r_max:
            scale = r_max / r * 0.999  # Slightly inside boundary
            x *= scale
            y *= scale
            r = r_max * 0.999
        elif r < r_min and r > 0:
            scale = r_min / r * 1.001  # Slightly outside inner boundary
            x *= scale
            y *= scale
            r = r_min * 1.001
        
        # Law of cosines to find the angle at the elbow
        cos_angle_at_elbow = (l1**2 + l2**2 - r**2) / (2 * l1 * l2)
        cos_angle_at_elbow = max(-1.0, min(1.0, cos_angle_at_elbow))
        angle_at_elbow = math.acos(cos_angle_at_elbow)  # Interior angle at elbow
        
        # elbow_flex = 0 means L2 perpendicular to L1 (angle = pi/2)
        # For inverted arm, negate the elbow flex
        elbow_flex = -(math.pi/2 - angle_at_elbow)
        
        # Find theta1 (absolute angle of L1 from +X axis)
        # Using law of cosines to find angle at shoulder
        cos_angle_at_shoulder = (l1**2 + r**2 - l2**2) / (2 * l1 * r)
        cos_angle_at_shoulder = max(-1.0, min(1.0, cos_angle_at_shoulder))
        angle_at_shoulder = math.acos(cos_angle_at_shoulder)
        
        # Angle from +X to target point
        target_angle = math.atan2(y, x)
        
        # For inverted arm, we subtract the shoulder angle to keep the elbow
        # on the correct side (below the line from shoulder to end effector)
        theta1 = target_angle - angle_at_shoulder
        
        # Convert theta1 to shoulder_lift
        # theta1 = -pi/2 + shoulder_lift, so shoulder_lift = theta1 + pi/2
        shoulder_lift = theta1 + math.pi/2
        
        return shoulder_lift, elbow_flex

    
    def generate_sinusoidal_velocity_trajectory(
        self,
        start_point: Union[List[float], np.ndarray],
        end_point: Union[List[float], np.ndarray],
        control_freq: float = 100.0,  # Hz
        total_time: float = 5.0,      # seconds
        velocity_amplitude: float = 1.0,  # m/s
        velocity_period: float = 2.0,     # seconds
        phase_offset: float = 0.0         # radians
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate a straight-line trajectory with sinusoidal velocity profile.
        
        Parameters:
        -----------
        start_point : array-like
            3D coordinates of starting point [x, y, z]
        end_point : array-like  
            3D coordinates of ending point [x, y, z]
        control_freq : float
            Control frequency in Hz
        total_time : float
            Total trajectory time in seconds
        velocity_amplitude : float
            Amplitude of velocity oscillation in m/s
        velocity_period : float
            Period of velocity oscillation in seconds
        phase_offset : float
            Phase offset in radians
            
        Returns:
        --------
        trajectory : np.ndarray
            Array of 3D positions (n_points, 3)
        velocities : np.ndarray
            Array of velocity magnitudes (n_points,)
        time_array : np.ndarray
            Time array (n_points,)
        """
        
        # Convert to numpy arrays
        start = np.array(start_point, dtype=float)
        end = np.array(end_point, dtype=float)
        
        # Calculate direction and distance
        direction_vector = end - start
        total_distance = np.linalg.norm(direction_vector)
        direction_unit = direction_vector / total_distance if total_distance > 0 else np.zeros(3)
        
        # Generate time array
        dt = 1.0 / control_freq
        n_points = int(total_time * control_freq) + 1
        time_array = np.linspace(0, total_time, n_points)
        
        # Calculate angular frequency
        omega = 2 * np.pi / velocity_period
        
        # Generate sinusoidal velocity profile
        base_velocity = total_distance / total_time  # Average velocity needed
        velocities = base_velocity + velocity_amplitude * np.sin(omega * time_array + phase_offset)
        
        # Ensure non-negative velocities (optional - remove if negative velocities are desired)
        velocities = np.maximum(velocities, 0.1 * base_velocity)
        
        # Integrate velocity to get position along the path
        positions_1d = np.zeros(n_points)
        for i in range(1, n_points):
            positions_1d[i] = positions_1d[i-1] + velocities[i-1] * dt
        
        # Scale positions to fit exactly between start and end points
        if positions_1d[-1] > 0:
            positions_1d = positions_1d * (total_distance / positions_1d[-1])
        
        # Convert 1D positions to 3D trajectory
        trajectory = np.zeros((n_points, 3))
        for i in range(n_points):
            progress = positions_1d[i] / total_distance if total_distance > 0 else 0
            trajectory[i] = start + progress * direction_vector
        
        return trajectory, velocities, time_array
    # Example usage
    # if __name__ == "__main__":
    #     # Define start and end points
    #     start = [0, 0, 0]
    #     end = [5, 3, 2]
        
    #     # Generate trajectory
    #     trajectory, velocities, time_array = generate_sinusoidal_velocity_trajectory(
    #         start_point=start,
    #         end_point=end,
    #         control_freq=100.0,
    #         total_time=6.0,
    #         velocity_amplitude=0.8,
    #         velocity_period=1.5,
    #         phase_offset=0
    #     )
        
    #     print(f"Generated {len(trajectory)} trajectory points")
    #     print(f"Total distance: {np.linalg.norm(np.array(end) - np.array(start)):.3f}")
    #     print(f"Time duration: {time_array[-1]:.2f} seconds")
    #     print(f"Average velocity: {velocities.mean():.3f} m/s")
    #     print(f"Velocity range: {velocities.min():.3f} to {velocities.max():.3f} m/s")
        
    #     print("\nFirst few trajectory points:")
    #     for i in range(0, min(10, len(trajectory)), 2):
    #         print(f"t={time_array[i]:.2f}s: pos=[{trajectory[i,0]:.3f}, {trajectory[i,1]:.3f}, {trajectory[i,2]:.3f}], vel={velocities[i]:.3f} m/s")
