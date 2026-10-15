import math


# Servo-order joint names for one arm (servo index offset 0 = left, 6 = right).
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")

# Joint limits (radians) from alfie_urdf (alfiebot.urdf); gripper is the
# trigger-mapped range 0 (open) .. 0.785 (closed).
ARM_JOINT_LIMITS = {
    "shoulder_pan":  (-0.7854, 0.7854),
    "shoulder_lift": (-1.4835, 1.4835),
    "elbow_flex":    (-1.5708, 1.5708),
    "wrist_flex":    (-0.7854, 1.5708),
    "wrist_roll":    (-1.5708, 1.5708),
    "gripper":       (0.0, 0.7854),
}


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SimpleTeleopArm:
    """
    VR delta-control for one arm.

    While the operator holds grip, controller motion since the previous goal is
    added to a Cartesian (reach, height) target solved with 2-link IK; controller
    X drives shoulder pan, and controller pitch/roll drive the wrist. All targets
    are radians in servo order and are clamped to ARM_JOINT_LIMITS.
    """

    # IK smoothing time constant (s). 0.031 s == the former alpha=0.3 at 90 Hz.
    SMOOTHING_TAU = 0.031

    def __init__(self, robotlowstate, kinematics, prefix="right", debug_logs=False):
        """
        Args:
            robotlowstate: RobotLowState used to seed targets from the measured pose.
            kinematics: AlfieArmKinematics for this arm.
            prefix: "left" or "right".
            debug_logs: Enable debug printing.
        """
        self.debug_logs = debug_logs
        self.prefix = prefix
        self.offset = 0 if prefix == 'left' else 6
        self.kinematics = kinematics

        # VR position multipliers (VR controller position -> robot arm movement)
        self.vr_x_multiplier = 40  # Scale for shoulder rotate
        self.vr_y_multiplier = 40  # Scale for arm reach (Y)
        self.vr_z_multiplier = 40  # Scale for arm reach (Z->X)

        # Wrist multipliers (VR controller rotation -> robot wrist rotation)
        self.wrist_pitch_multiplier = 4.0
        self.wrist_roll_multiplier = 4.0

        # Per-goal delta limits (prevent sudden jumps from tracking glitches)
        self.pos_scale = 0.05      # Position sensitivity scaling
        self.delta_limit = 0.02    # Maximum reach/height delta per goal (m)
        self.angle_limit = 0.14    # Maximum angle delta per goal (rad, ~8 degrees)

        # Zero ("ready") position, used by the Y-button reset
        self.zero_pos = {
            'shoulder_pan':  0.0000,
            'shoulder_lift': 0.1089,
            'elbow_flex':   -1.4864,
            'wrist_flex':   -0.1442,
            'wrist_roll':    0.0000,
            'gripper':       0.0000
        }

        self.reseed(robotlowstate)

    def debug_print(self, msg: str):
        """Print debug message only if debug_logs is enabled."""
        if self.debug_logs:
            print(msg)

    def _set_targets(self, joints):
        """Set all targets (clamped) and derive the Cartesian/pitch state from them."""
        self.target_positions = {
            j: _clamp(float(joints[j]), *ARM_JOINT_LIMITS[j]) for j in ARM_JOINTS
        }
        t = self.target_positions
        self.current_x, self.current_y = self.kinematics.forward_kinematics(
            t["shoulder_lift"], t["elbow_flex"])
        # Inverse of wrist_flex = -shoulder_lift - elbow_flex + pitch, so the
        # first tracked frame reproduces the current wrist angle (no jump).
        self.pitch = t["wrist_flex"] + t["shoulder_lift"] + t["elbow_flex"]
        self.reset_tracking()

    def reseed(self, robotlowstate):
        """Set targets to the measured joint positions (no motion on next command)."""
        servos = robotlowstate.servo_state
        self._set_targets({
            j: servos[self.offset + i].current_location for i, j in enumerate(ARM_JOINTS)
        })

    def move_to_zero_position(self):
        """Set targets to the zero position (the node rate-limits the motion)."""
        self.debug_print(f"[{self.prefix}] Moving to zero position: {self.zero_pos}")
        self._set_targets(self.zero_pos)

    def reset_tracking(self):
        """Forget the previous VR sample so the next goal only sets a new baseline."""
        self.prev_vr_pos = None
        self.prev_wrist_flex = None
        self.prev_wrist_roll = None
        self._last_goal_stamp = None

    def handle_vr_input(self, vr_goal, dt):
        """
        Apply one control tick while grip is held.

        Deltas are taken only from goals not seen before (the client sends at
        ~30 Hz while this runs at ~90 Hz); IK smoothing runs every tick using dt.

        Args:
            vr_goal: ControlGoal for this controller.
            dt: Seconds since the previous tick.
        """
        if vr_goal is None:
            return

        if vr_goal.stamp != self._last_goal_stamp:
            self._last_goal_stamp = vr_goal.stamp
            self._apply_goal_deltas(vr_goal)

        self._solve(dt)

        # Gripper follows the analog trigger: 0.0-1.0 -> open..closed
        trigger_value = _clamp(float(vr_goal.metadata.get('trigger', 0) or 0), 0.0, 1.0)
        self.target_positions["gripper"] = _clamp(
            trigger_value * ARM_JOINT_LIMITS["gripper"][1], *ARM_JOINT_LIMITS["gripper"])

    def _apply_goal_deltas(self, vr_goal):
        """Accumulate controller motion since the previous goal into the targets."""
        current_vr_pos = vr_goal.target_position
        if current_vr_pos is None:
            # Tracking gap: re-baseline when tracking returns instead of jumping.
            self.reset_tracking()
            self._last_goal_stamp = vr_goal.stamp
            return

        if self.prev_vr_pos is None:
            # First sample after grip / gap: establish baselines only.
            self.prev_vr_pos = current_vr_pos
            self.prev_wrist_flex = vr_goal.wrist_flex_deg
            self.prev_wrist_roll = vr_goal.wrist_roll_deg
            return

        dl = self.delta_limit
        delta_x = _clamp((current_vr_pos[0] - self.prev_vr_pos[0]) * self.vr_x_multiplier * self.pos_scale, -dl, dl)
        delta_y = _clamp((current_vr_pos[1] - self.prev_vr_pos[1]) * self.vr_y_multiplier * self.pos_scale, -dl, dl)
        delta_z = _clamp((current_vr_pos[2] - self.prev_vr_pos[2]) * self.vr_z_multiplier * self.pos_scale, -dl, dl)
        self.prev_vr_pos = current_vr_pos

        # VR coordinate system (WebXR): X=left(-)/right(+), Y=down(-)/up(+), Z=forward(-)/back(+)
        # Kinematics coordinate system: X=forward reach, Y=vertical (negative=down)
        self.current_x += -delta_z  # VR Z (forward/back) -> robot X (reach)
        self.current_y += delta_y   # VR Y (up/down) -> robot Y (vertical)

        if abs(delta_y) > 0.001 or abs(delta_z) > 0.001:
            self.debug_print(f"[{self.prefix}] delta_y={delta_y:.4f}, delta_z={delta_z:.4f} -> x={self.current_x:.4f}, y={self.current_y:.4f}")

        al = self.angle_limit
        if vr_goal.wrist_flex_deg is not None:
            if self.prev_wrist_flex is not None:
                delta_pitch = math.radians(vr_goal.wrist_flex_deg - self.prev_wrist_flex) * self.wrist_pitch_multiplier
                self.pitch = _clamp(self.pitch + _clamp(delta_pitch, -al, al), -math.pi / 2, math.pi / 2)
            self.prev_wrist_flex = vr_goal.wrist_flex_deg

        if vr_goal.wrist_roll_deg is not None:
            if self.prev_wrist_roll is not None:
                delta_roll = math.radians(vr_goal.wrist_roll_deg - self.prev_wrist_roll) * self.wrist_roll_multiplier
                self.target_positions["wrist_roll"] = _clamp(
                    self.target_positions["wrist_roll"] - _clamp(delta_roll, -al, al),
                    *ARM_JOINT_LIMITS["wrist_roll"])
            self.prev_wrist_roll = vr_goal.wrist_roll_deg

        # VR X axis controls shoulder_pan joint (delta control)
        if abs(delta_x) > 0.001:
            delta_pan = _clamp(delta_x * 3.5, -al, al)
            self.target_positions["shoulder_pan"] = _clamp(
                self.target_positions["shoulder_pan"] - delta_pan,  # Negated to fix direction
                *ARM_JOINT_LIMITS["shoulder_pan"])

    def _solve(self, dt):
        """IK toward (current_x, current_y), smoothed, clamped, without wind-up."""
        lift_lim = ARM_JOINT_LIMITS["shoulder_lift"]
        elbow_lim = ARM_JOINT_LIMITS["elbow_flex"]

        # Arm can't reach behind the shoulder; keep the target reachable.
        self.current_x, self.current_y = self.kinematics.clamp_to_workspace(
            max(0.01, self.current_x), self.current_y)
        lift, elbow = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
        lift, elbow = _clamp(lift, *lift_lim), _clamp(elbow, *elbow_lim)
        # Anti-windup: the target is wherever the clamped solution actually is,
        # so pushing past a limit doesn't have to be "unwound" afterwards.
        self.current_x, self.current_y = self.kinematics.forward_kinematics(lift, elbow)

        alpha = 1.0 - math.exp(-max(dt, 0.0) / self.SMOOTHING_TAU)
        t = self.target_positions
        t["shoulder_lift"] = (1 - alpha) * t["shoulder_lift"] + alpha * lift
        t["elbow_flex"] = (1 - alpha) * t["elbow_flex"] + alpha * elbow

        # Wrist pitch holds end-effector orientation; clamp and back off pitch at the limit.
        wrist = -t["shoulder_lift"] - t["elbow_flex"] + self.pitch
        wrist_c = _clamp(wrist, *ARM_JOINT_LIMITS["wrist_flex"])
        self.pitch += wrist_c - wrist
        t["wrist_flex"] = wrist_c

        self.debug_print(f"[{self.prefix}] x={self.current_x:.4f}, y={self.current_y:.4f} -> shoulder_lift={math.degrees(lift):.1f}°, elbow_flex={math.degrees(elbow):.1f}°")


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
        
        # At shoulder_lift=0, L1 points down (-pi/2 from +X axis).
        # Servo convention: positive shoulder_lift rotates L1 backward (toward -X);
        # this matches inverse_kinematics, which returns servo angles.
        theta1 = -math.pi/2 - shoulder_lift
        
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
        
        theta1 = -math.pi/2 - shoulder_lift  # servo convention, see forward_kinematics
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
        
        # Convert theta1 to shoulder_lift (servo convention)
        # theta1 = -pi/2 - shoulder_lift, so shoulder_lift = -(theta1 + pi/2)
        shoulder_lift = -(theta1 + math.pi/2)
        
        return shoulder_lift, elbow_flex

    def clamp_to_workspace(self, x, y, l1=None, l2=None):
        """Project (x, y) onto the reachable annulus [|l1-l2|, l1+l2] (same margins as IK)."""
        l1 = self.l1 if l1 is None else l1
        l2 = self.l2 if l2 is None else l2
        r = math.hypot(x, y)
        r_max = (l1 + l2) * 0.999
        r_min = abs(l1 - l2) * 1.001
        if r > r_max:
            return x * r_max / r, y * r_max / r
        if 0 < r < r_min:
            return x * r_min / r, y * r_min / r
        return x, y
