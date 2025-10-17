# Motor state information
bool is_moving             # Movement detection flag
int16 pwm_cmd              # PWM command value (-255 to 255)
float32 velocity           # Linear velocity in meters per second (m/s)
float32 acceleration       # Linear acceleration in meters per second squared (m/s^2)
int64 pulse_count          # Total encoder pulse count (accumulated)

