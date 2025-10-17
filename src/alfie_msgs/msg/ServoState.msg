# Servo enabled state (torque on/off)
bool enabled

# Servo acceleration in radians per second squared (rad/s²)
float32 acceleration

# Target position in radians (-π to +π)
float32 target_location

# Current position in radians (-π to +π)
float32 current_location

# Current rotational velocity in radians per second (rad/s)
float32 current_speed

# Current load as percentage (%)
float32 current_load

# Current servo temperature in degrees Celsius (°C)
uint8 current_temperature

# Servo status code (vendor-specific)
uint8 servo_status

# Movement detection flag
bool is_moving

# Current voltage in volts (V)
float32 current_voltage

# Current draw in milliamps (mA)
float32 current_current
  