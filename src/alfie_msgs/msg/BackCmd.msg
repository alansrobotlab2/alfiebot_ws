# Command message for shoulder linear actuator control
# Position range: 0mm (bottom, switch sensor position) to 400mm (top)

# Enable/disable the actuator motor
bool enabled

# Target position in meters (0.0 to 0.4m)
float32 target_position

# Target velocity in meters per second (m/s)
# Positive values move up, negative values move down
float32 target_velocity
