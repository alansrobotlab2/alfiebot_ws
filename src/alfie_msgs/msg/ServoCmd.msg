# Servo enabled state (torque on/off)
bool enabled

# Target position in radians (-π to +π)
float32 target_location 


# Servo speed in radians per second
float32 target_speed


# Servo acceleration in radians per second squared (rad/s²)
float32 target_acceleration


# Set the maximum output torque output for this command, where 1000 corresponds to 100% of the locked-rotor torque. 
# This value is assigned from max_torque upon power-up as the torque limit.
float32 target_torque

