int16[2] eye_pwm
float32 shoulder_height

BackCmd back_cmd

geometry_msgs/Twist cmd_vel

# state for all 15 servos on robot 
# 0-5 left arm, 
# 6-11 right arm, 
# 12-14 head
ServoCmd[15] servo_cmd