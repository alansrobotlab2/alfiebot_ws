# Eye LED command for the head module, exposed as a standalone mux channel so a
# low-priority source (agent expression, idle "life") can animate the eyes without
# owning head-servo motion. The command_mux merges the winning EyeCmd's eye_pwm into
# the outgoing HeadCmd (low/headcmd); the head board owns both servos and eye LEDs.
#
# Raw PWM duty, 0..4095 (driven at 100 Hz through a TB6612FNG).
# [0] = left eye  (TB6612 channel A)
# [1] = right eye (TB6612 channel B)
uint16[2] eye_pwm
