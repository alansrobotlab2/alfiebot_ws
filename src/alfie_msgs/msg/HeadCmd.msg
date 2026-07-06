# Combined command for the head module: 3 serial bus servos + 2 eye LEDs.
# Servos use SI units (see ServoCmd). Bus IDs: pan=1, tilt=2, roll=3.
ServoCmd pan
ServoCmd tilt
ServoCmd roll

# Eye LED brightness as raw PWM duty, 0..255 (driven at 100 Hz through a TB6612FNG).
# [0] = left eye  (TB6612 channel A)
# [1] = right eye (TB6612 channel B)
uint8[2] eye_pwm
