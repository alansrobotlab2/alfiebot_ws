# MecanumMotorState.msg - State information for a single mecanum wheel motor
# Based on the MotorState_t structure from the RP2040 firmware

# Standard ROS header with timestamp
std_msgs/Header header

# Motor velocities
float32 target_velocity      # Target wheel velocity (m/s)
float32 current_velocity     # Current wheel velocity (m/s)  
float32 current_acceleration # Current wheel acceleration (m/s²)

# Encoder data
int32 encoder_count          # Current encoder count
uint32 last_pulse_time       # Last encoder pulse timestamp (microseconds)

# Motor control
float32 pwm_output           # PWM output value (0-255)

# Status flags
bool fault_detected          # Motor fault flag
bool is_moving              # True if motor is currently moving
bool encoder_a_state        # Current state of encoder A pin
bool encoder_b_state        # Current state of encoder B pin