uint8 board_temp
bool limit_switch_triggered

# Linear actuator state feedback
# in meters
float32 command_position

# in meters per second
float32 command_velocity

# in meters per second per second
float32 command_acceleration

bool is_moving
float32 current_position
float32 current_velocity
float32 current_acceleration
int64 pulse_count
uint8 pwm_output

bool is_calibrated

# Calibration state constants
# CALIBRATION_IDLE = 0
# CALIBRATION_LOWERING = 1
# CALIBRATION_RAISING = 2
# CALIBRATION_COMPLETE = 3
# CALIBRATION_ERROR = 4

# Current calibration state (see constants above)
uint8 calibration_status
