std_msgs/Header header

# 3 imu total, head, upper gdb board in base, lower gdb board in base
sensor_msgs/Imu[3] imu

# 2 magnetic field sensor total, upper gdb board in base, lower gdb board in base
sensor_msgs/MagneticField[2] magnetic_field

# back state
BackState back_state

# pwm value from 0 to 255, lowest on state is 1
uint8[2] eye_state

# state for each of the driver motors in base
MotorState[2] motor_state

# state for all 15 servos on robot 
# 0-5 left arm, 6-11 right arm, 12-14 head
ServoState[15] servo_state