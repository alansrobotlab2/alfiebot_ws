std_msgs/Header header

# 3 imu total, head, upper gdb board in base, lower gdb board in base
sensor_msgs/Imu[3] imu

# 2 magnetic field sensor total, upper gdb board in base, lower gdb board in base
sensor_msgs/MagneticField[2] magnetic_field

# shoulder limit switch trigger eye_state
# value of true indicates lowest position of shoulders 
# (max height is 400 mm from there)
bool shoulder_limit_state

# pwm value from 0 to 255, lowest on state is 1
uint8[2] eye_state

# state for each of the driver motors in base
MotorState[2] motor_state

# state for all 17 servos on robot 
# 0-6 left arm, 7-13 right arm, 14-17 head
ServoState[17] servo_state