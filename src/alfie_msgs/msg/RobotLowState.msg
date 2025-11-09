std_msgs/Header header

# 3 imu total, head, upper gdb board in base, lower gdb board in base
sensor_msgs/Imu[3] imu

# 2 magnetic field sensor total, upper gdb board in base, lower gdb board in base
sensor_msgs/MagneticField[2] magnetic_field

# back state
BackState back_state

# pwm value from 0 to 255, lowest on state is 1
uint8[2] eye_state

geometry_msgs/Twist cmd_vel

# state for each of the 4 driver motors in base
# 0: Front Left  (FL)
# 1: Front Right (FR)
# 2: Rear Left   (RL)
# 3: Rear Right  (RR)
MotorState[4] motor_state

# state for all 15 servos on robot 
# 0-5 left arm, 
# 6-11 right arm, 
# 12-14 head
ServoState[15] servo_state