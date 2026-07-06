# Combined state for the head module: 3 serial bus servos + 2 eye LEDs.
# Servos use SI units (see ServoState). Bus IDs: pan=1, tilt=2, roll=3.
std_msgs/Header header

ServoState pan
ServoState tilt
ServoState roll

# Last-applied eye LED PWM duty, 0..255.
# [0] = left eye  (TB6612 channel A)
# [1] = right eye (TB6612 channel B)
uint8[2] eye_state
