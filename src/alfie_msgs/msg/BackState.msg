# State message for shoulder linear actuator feedback
# Position range: 0mm (bottom, switch sensor position) to 400mm (top)

# The calibrated value is set as soon as the calibration routine has been successfully completed
# before calibration the back will ignore all commands
bool calibrated

# Actuator enabled state (motor on/off)
bool enabled

# Movement detection flag
bool is_moving

# Current position in meters (0.0 to 0.4m)
# 0.0m = bottom position (switch sensor triggered)
# 0.4m = top position (maximum travel)
float32 position

# Current velocity in meters per second (m/s)
# Positive = moving up, negative = moving down
float32 velocity

# Current acceleration in meters per second squared (m/s²)
float32 acceleration

# Bottom limit switch state (true when at 0mm position)
bool at_bottom_limit

# Top limit reached (true when at 400mm position)
bool at_top_limit
