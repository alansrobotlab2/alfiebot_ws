std_msgs/Header header

# Single IMU: the BNO085 on the back/spine board (via picobackdriver), reported
# in the base frame. The same data is available raw inside back_state.imu; it is
# republished here as a standard sensor_msgs/Imu for convenience.
sensor_msgs/Imu imu

# Back / spine state (linear actuator + BNO085), straight from picobackdriver.
BackState back_state

# Eye LED PWM duty from the head board, 0..4095.
# [0] = left eye, [1] = right eye
uint16[2] eye_state

# Measured base velocity from the mecanum drive board's fused wheel odometry
# (/alfie/low/odom). Twist expressed in base_link. There is no per-wheel
# feedback in gen2 hardware, so individual motor states are not reported.
geometry_msgs/Twist cmd_vel

# State for all 15 arm + head servos, in SI units with per-servo polarity already
# applied by the module firmware:
#   0-5   left arm   (shoulder_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll, gripper)
#   6-11  right arm  (same order)
#   12-14 head       (yaw/pan, pitch/tilt, roll)
ServoState[15] servo_state
