"""
head_motion — pure keyframe plans for the agent's head tools (no ROS here).

The head servos ramp to a commanded target at a commanded speed by themselves, so
a "motion" is just a short list of keyframes — (start time, yaw, pitch, roll,
speed) — that body.py republishes on ``cmd/head/agent`` at 20 Hz while the plan
lasts (the mux drops a source that goes quiet for 0.5 s, and the idle behavior
takes the head back). Everything is clamped to the head's travel.

Conventions match idle_behavior.py: radians, HeadCmd order [yaw, pitch, roll],
``yaw_sign = -1`` (positive bearing = to the robot's left = negative servo yaw),
pitch + = down.
"""
import math

YAW_LIMITS = (-3.14, 3.14)
PITCH_LIMITS = (-0.7, 0.4)
ROLL_LIMITS = (-0.35, 0.35)
YAW_SIGN = -1.0

LOOK_SPEED = 1.5        # rad/s, idle_behavior's attend_speed
GESTURE_SPEED = 3.0
DEFAULT_HOLD_S = 4.0

# Named directions: bearing (deg, + = robot's left) and elevation (deg, + = up).
DIRECTIONS = {
    "left": (60.0, None), "right": (-60.0, None),
    "up": (None, 25.0), "down": (None, -20.0),
    "forward": (0.0, 0.0), "ahead": (0.0, 0.0), "center": (0.0, 0.0),
    "straight": (0.0, 0.0), "behind": (170.0, None),
}

GESTURES = ("nod", "shake", "tilt", "look_around")


def _clamp(v, lo_hi):
    return max(lo_hi[0], min(lo_hi[1], v))


def clamp_pose(yaw, pitch, roll=0.0):
    return (_clamp(yaw, YAW_LIMITS), _clamp(pitch, PITCH_LIMITS),
            _clamp(roll, ROLL_LIMITS))


def bearing_to_yaw(bearing_deg, cur_yaw=0.0, relative=True):
    """Servo yaw for a bearing (+ = robot's left), relative to the current head
    yaw or to the body's forward axis."""
    delta = YAW_SIGN * math.radians(bearing_deg)
    return (cur_yaw if relative else 0.0) + delta


def elevation_to_pitch(elev_deg, cur_pitch=0.0, relative=True):
    """Servo pitch for an elevation (+ = up; servo pitch + = down)."""
    return (cur_pitch if relative else 0.0) - math.radians(elev_deg)


def image_offset_to_angles(cx, cy, hfov_deg, aspect=256.0 / 352.0):
    """Bearing/elevation (deg) of a normalized image point (0..1, 0..1).

    Pinhole model with the camera's horizontal FOV, as idle_behavior's face
    centering (``atan(2 (c - 0.5) tan(hfov / 2))``).
    """
    tan_h = math.tan(math.radians(hfov_deg) / 2.0)
    tan_v = tan_h * aspect
    bearing = -math.degrees(math.atan(2.0 * (cx - 0.5) * tan_h))
    elev = -math.degrees(math.atan(2.0 * (cy - 0.5) * tan_v))
    return bearing, elev


def look_plan(cur, target, hold_s=DEFAULT_HOLD_S):
    """Turn to ``target`` (yaw, pitch[, roll]) and hold it ``hold_s`` seconds.

    The hold includes the travel time at LOOK_SPEED.
    """
    yaw, pitch, roll = clamp_pose(*target) if len(target) == 3 else \
        clamp_pose(target[0], target[1], 0.0)
    travel = max(abs(yaw - cur[0]), abs(pitch - cur[1])) / LOOK_SPEED
    return [(0.0, yaw, pitch, roll, LOOK_SPEED)], travel + hold_s


def gesture_plan(name, cur):
    """Keyframes for a named gesture starting from pose ``cur``; ends where it began."""
    y, p, r = cur[0], cur[1], cur[2] if len(cur) > 2 else 0.0
    s = GESTURE_SPEED
    if name == "nod":
        seq = [(0.0, y, p + 0.25, r), (0.35, y, p - 0.05, r),
               (0.7, y, p + 0.25, r), (1.05, y, p, r)]
        end = 1.5
    elif name == "shake":
        seq = [(0.0, y - 0.35, p, r), (0.4, y + 0.35, p, r),
               (0.8, y - 0.35, p, r), (1.2, y, p, r)]
        end = 1.7
    elif name == "tilt":
        seq = [(0.0, y, p, 0.3), (1.6, y, p, r)]
        end = 2.0
        s = 1.0
    elif name == "look_around":
        seq = [(0.0, y + 0.9, p - 0.1, r), (1.6, y - 0.9, p - 0.1, r),
               (3.4, y, p, r)]
        end = 4.2
        s = LOOK_SPEED
    else:
        raise ValueError(f"unknown gesture: {name}")
    frames = [(t, *clamp_pose(fy, fp, fr), s) for t, fy, fp, fr in seq]
    return frames, end


def frame_at(frames, t):
    """The keyframe active at time ``t`` into the plan (the last one started)."""
    active = frames[0]
    for f in frames:
        if f[0] <= t:
            active = f
    return active
