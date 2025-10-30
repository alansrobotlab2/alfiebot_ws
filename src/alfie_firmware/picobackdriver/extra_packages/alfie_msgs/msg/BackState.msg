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
