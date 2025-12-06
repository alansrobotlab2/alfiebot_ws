"""
Servo configuration shared across master_status and master_cmd nodes.

This module contains the servo polarity configuration that maps between
physical servo control and actual robot controls.
"""

# Servo polarity array - converts between physical servo control and actual controls
# Positive (1) means no inversion, Negative (-1) means inverted
# Index corresponds to servo number in RobotLowState/RobotLowCmd (0-16)
# Note: Indices 2 and 9 are derived servos (computed from indices 1 and 8 respectively)
SERVO_POLARITY = [
     1,   # Servo 0:  driver1/servo01 - left shoulder yaw 
    -1,   # Servo 1:  driver1/servo02 - left shoulder1 pitch
    -1,   # Servo 2:  driver1/servo03 - left shoulder2 pitch (DERIVED from index 1)
    -1,   # Servo 3:  driver1/servo04 - left elbow pitch
     1,   # Servo 4:  driver1/servo05 - left wrist pitch
    -1,   # Servo 5:  driver1/servo06 - left wrist roll
    -1,   # Servo 6:  driver1/servo07 - left hand
     1,   # Servo 7:  driver0/servo01 - right shoulder yaw
    -1,   # Servo 8:  driver0/servo02 - right shoulder1 pitch
    -1,   # Servo 9:  driver0/servo03 - right shoulder2 pitch (DERIVED from index 8)
    -1,   # Servo 10: driver0/servo04 - right elbow pitch
     1,   # Servo 11: driver0/servo05 - right wrist pitch
    -1,   # Servo 12: driver0/servo06 - right wrist roll
    -1,   # Servo 13: driver0/servo07 - right hand
    -1,  # Servo 14: driver0/servo08 - head yaw
    -1,  # Servo 15: driver0/servo09 - head pitch
    -1,  # Servo 16: driver0/servo10 - head roll
]
