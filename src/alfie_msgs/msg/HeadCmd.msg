# Combined command for the head module: 3 serial bus servos + 2 eye LEDs.
# Servos use SI units (see ServoCmd).
# Array order: [0] = pan (bus 1), [1] = tilt (bus 2), [2] = yaw (bus 3).
ServoCmd[3] servos

# Eye LED brightness as raw PWM duty, 0..4095 (driven at 100 Hz through a TB6612FNG).
# [0] = left eye  (TB6612 channel A)
# [1] = right eye (TB6612 channel B)
uint16[2] eye_pwm
