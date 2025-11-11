# Write 0: disable the torque output; 
# Write 1: enable the torque output; 
# Write 128: Arbitrary current position correction to 2048.
uint8 torque_switch 


# -2048 - 2048 for a full rotation at standard
int16 target_location 


# 0 - 3400 
# Number of steps per unit time (per second)
# 50 steps per second = 0.732 RPM (revolutions per minute)
uint16 target_speed  


# 0 - 254
# If set to 10, it corresponds to an acceleration and deceleration rate of 1000 steps per second squared
uint8 target_acceleration


# 0 - 1000
# The initial value of power-on will be assigned by the maximum torque (0x10), which can be modified by the user to control the output of the maximum torque.
uint16 target_torque
   