# Combined state for the head module: 3 serial bus servos + 2 eye LEDs.
# Servos use SI units (see ServoState).
# Array order: [0] = pan (bus 1), [1] = tilt (bus 2), [2] = yaw (bus 3).
std_msgs/Header header

ServoState[3] servos

# Last-applied eye LED PWM duty, 0..4095.
# [0] = left eye  (TB6612 channel A)
# [1] = right eye (TB6612 channel B)
uint16[2] eye_state
