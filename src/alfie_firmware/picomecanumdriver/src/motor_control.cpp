/**
 * @file motor_control.cpp
 * @brief Motor control and peripheral management implementation
 *
 * Drives a Hiwonder 4-channel encoder motor controller over I2C. The Hiwonder
 * board runs its own per-channel closed-loop velocity PID, so this module only
 * converts body/wheel velocities to the controller's speed units (pulses per
 * 10 ms), pushes them over I2C, reads back accumulated encoder counts, and
 * integrates odometry. No GPIO PWM or encoder interrupts are used.
 *
 * @author Alfie Bot Project
 */

#include "motor_control.h"
#include "driverboard.h"

// Wheel contact circumference in meters (distance travelled per wheel revolution)
static const float WHEEL_CIRCUMFERENCE_M = PI * (WHEEL_DIAMETER_MM / 1000.0f);

// =============================================================================
// DRIVERBOARD CLASS IMPLEMENTATION
// =============================================================================

/**
 * @brief DriverBoard constructor - initializes all state variables
 */
DriverBoard::DriverBoard() {
    // Initialize motor states
    for (int i = 0; i < 4; i++) {
        motors[i].target_velocity = 0.0f;
        motors[i].current_velocity = 0.0f;
        motors[i].current_acceleration = 0.0f;
        motors[i].encoder_count = 0;
        motors[i].speed_cmd = 0;
        motors[i].fault_detected = false;
        motors[i].is_moving = false;
    }

    // Initialize robot status
    robot_status = ERROR_NONE;

    // Initialize ROS state machine variables
    agent_state = WAITING_AGENT;
    last_state_time = 0;
    ros_entities_created = false;

    // Initialize ROS communication state
    micro_ros_initialized = false;
    last_command_time = 0;

    // Initialize inter-core communication
    velocity_cmd.linear_x = 0.0f;
    velocity_cmd.linear_y = 0.0f;
    velocity_cmd.angular_z = 0.0f;
    velocity_cmd.timestamp = 0;

    odometry.position_x = 0.0f;
    odometry.position_y = 0.0f;
    odometry.orientation = 0.0f;
    odometry.linear_velocity_x = 0.0f;
    odometry.linear_velocity_y = 0.0f;
    odometry.angular_velocity = 0.0f;
    odometry.timestamp = 0;

    new_velocity_command = false;
    new_odometry_data = false;
}

// =============================================================================
// MOTOR CONTROL IMPLEMENTATION
// =============================================================================

/**
 * @brief Initialize all hardware peripherals
 * Brings up the Hiwonder controller over I2C and stops the motors.
 * (The status LED is a WS2812 owned by main.cpp Core 0.)
 */
void DriverBoard::initializePeripherals(void) {
    // Bring up the Hiwonder motor controller (I2C bus, motor type, polarity)
    hw.begin();

    // Establish the encoder baseline and stop all motors
    resetEncoders();
    emergencyStop();
}

/**
 * @brief Main peripheral management loop
 * Should be called regularly from Core 0 loop
 */
void DriverBoard::updatePeripherals(void) {
    // Read encoder counts and derive per-wheel velocity
    readEncoders();

    // Convert the current velocity command into Hiwonder closed-loop targets
    updateMotorControl();

    // Calculate odometry from wheel velocities
    updateOdometry();

    // Check for motor faults and safety conditions
    handleMotorSafety();

    // Status LED is handled in main.cpp Core 0 loop
    updateStatusLED();
}

/**
 * @brief Update motor control - push closed-loop speed targets to the controller
 *
 * Runs inverse mecanum kinematics on the current body velocity command, converts
 * each wheel's linear velocity (m/s) into the Hiwonder speed unit (pulses per
 * HIWONDER_CONTROL_PERIOD_MS), and writes all four channels over I2C. The
 * controller closes the velocity loop internally.
 */
void DriverBoard::updateMotorControl(void) {
    // Snapshot the current velocity command
    float linear_x = velocity_cmd.linear_x;
    float linear_y = velocity_cmd.linear_y;
    float angular_z = velocity_cmd.angular_z;

    // Inverse kinematics -> per-wheel linear velocity [FL, FR, RL, RR] (m/s)
    float wheel_velocities[4];
    mecanumDriveKinematics(linear_x, linear_y, angular_z, wheel_velocities);

    // Convert m/s -> pulses per control window and clamp to the int8 register range
    const float window_s = HIWONDER_CONTROL_PERIOD_MS / 1000.0f;
    int8_t speed_pulses[4];
    for (int i = 0; i < 4; i++) {
        motors[i].target_velocity = wheel_velocities[i];

        float rev_per_s = wheel_velocities[i] / WHEEL_CIRCUMFERENCE_M;
        float pulses = rev_per_s * COUNTS_PER_WHEEL_REV * window_s;
        int32_t p = (int32_t)lroundf(pulses);
        if (p > HIWONDER_MAX_SPEED)  p = HIWONDER_MAX_SPEED;
        if (p < -HIWONDER_MAX_SPEED) p = -HIWONDER_MAX_SPEED;

        speed_pulses[i] = (int8_t)p;
        motors[i].speed_cmd = (int16_t)p;
    }

    hw.setSpeeds(speed_pulses);
}

/**
 * @brief Read encoder counts from the controller and derive wheel velocity
 * Updates each wheel's velocity/acceleration from the accumulated count deltas.
 */
void DriverBoard::readEncoders(void) {
    static uint32_t last_encoder_time = 0;
    static int32_t last_counts[4] = {0, 0, 0, 0};
    static bool initialized = false;

    int32_t counts[4];
    hw.readEncoders(counts);

    uint32_t current_time = millis();

    // First pass just captures a baseline so the first delta isn't a huge jump
    if (!initialized) {
        for (int i = 0; i < 4; i++) {
            last_counts[i] = counts[i];
            motors[i].encoder_count = counts[i];
        }
        last_encoder_time = current_time;
        initialized = true;
        return;
    }

    float dt = (current_time - last_encoder_time) / 1000.0f; // seconds
    if (dt <= 0.0f) {
        return;
    }

    for (int i = 0; i < 4; i++) {
        int32_t count_diff = counts[i] - last_counts[i];
        last_counts[i] = counts[i];

        float previous_velocity = motors[i].current_velocity;

        // counts -> wheel revolutions -> meters travelled -> m/s
        float rev = (float)count_diff / COUNTS_PER_WHEEL_REV;
        motors[i].current_velocity = (rev * WHEEL_CIRCUMFERENCE_M) / dt;
        motors[i].current_acceleration = (motors[i].current_velocity - previous_velocity) / dt;

        const float VELOCITY_THRESHOLD = 0.01f; // 1 cm/s
        motors[i].is_moving = (fabs(motors[i].current_velocity) > VELOCITY_THRESHOLD);

        motors[i].encoder_count = counts[i];
    }

    last_encoder_time = current_time;
}

/**
 * @brief Calculate odometry from wheel velocities
 * Updates robot position and velocity estimates via forward kinematics + Euler integration.
 */
void DriverBoard::updateOdometry(void) {
    // Get wheel velocities
    float wheel_velocities[4] = {
        motors[0].current_velocity, // FL
        motors[1].current_velocity, // FR
        motors[2].current_velocity, // RL
        motors[3].current_velocity  // RR
    };

    // Forward kinematics -> body velocities
    float linear_x, linear_y, angular_z;
    mecanumDriveOdometry(wheel_velocities, &linear_x, &linear_y, &angular_z);

    static uint32_t last_odom_time = 0;
    uint32_t current_time = millis();
    float dt = (current_time - last_odom_time) / 1000.0f;

    if (dt >= (CONTROL_LOOP_PERIOD_MS / 1000.0f)) {
        // Update velocities
        odometry.linear_velocity_x = linear_x;
        odometry.linear_velocity_y = linear_y;
        odometry.angular_velocity = angular_z;

        // Integrate to get pose (simple Euler integration in the world frame)
        float cos_theta = cos(odometry.orientation);
        float sin_theta = sin(odometry.orientation);

        odometry.position_x += dt * (linear_x * cos_theta - linear_y * sin_theta);
        odometry.position_y += dt * (linear_x * sin_theta + linear_y * cos_theta);
        odometry.orientation += dt * angular_z;

        odometry.timestamp = current_time;
        new_odometry_data = true;

        last_odom_time = current_time;
    }
}

/**
 * @brief Check for motor faults and safety conditions
 */
void DriverBoard::handleMotorSafety(void) {
    // Placeholder: the Hiwonder controller does not expose per-motor fault flags
    // over this register set. Battery monitoring could be added here via
    // hw.readBatteryMv() and MIN_BATTERY_VOLTAGE if desired.
    for (int i = 0; i < 4; i++) {
        motors[i].fault_detected = false;
    }
    robot_status = ERROR_NONE;
}

/**
 * @brief Update status LED based on robot state
 */
void DriverBoard::updateStatusLED(void) {
    // LED status is handled in main.cpp Core 0 loop (WS2812 / NeoPixel)
}

/**
 * @brief Emergency stop all motors
 */
void DriverBoard::emergencyStop(void) {
    // Command zero closed-loop speed to the controller
    hw.stop();

    for (int i = 0; i < 4; i++) {
        motors[i].target_velocity = 0.0f;
        motors[i].speed_cmd = 0;
    }

    // Reset velocity command (field-by-field for volatile)
    velocity_cmd.linear_x = 0.0f;
    velocity_cmd.linear_y = 0.0f;
    velocity_cmd.angular_z = 0.0f;
    velocity_cmd.timestamp = 0;
}

/**
 * @brief Re-baseline the encoder counters to the current controller values
 */
void DriverBoard::resetEncoders(void) {
    int32_t counts[4];
    hw.readEncoders(counts);
    for (int i = 0; i < 4; i++) {
        motors[i].encoder_count = counts[i];
    }
}

// =============================================================================
// MECANUM DRIVE KINEMATICS
// =============================================================================

/**
 * @brief Convert robot velocities to individual wheel velocities
 *
 * @param linear_x Forward/backward velocity (m/s)
 * @param linear_y Left/right strafe velocity (m/s)
 * @param angular_z Rotational velocity (rad/s)
 * @param wheel_velocities Output array for wheel velocities [FL, FR, RL, RR]
 */
void mecanumDriveKinematics(float linear_x, float linear_y, float angular_z,
                           float wheel_velocities[4]) {
    float wheel_separation_x = ROBOT_CENTER_TO_WHEEL_X / 1000.0f; // mm -> m
    float wheel_separation_y = ROBOT_CENTER_TO_WHEEL_Y / 1000.0f; // mm -> m

    // Standard mecanum inverse kinematics (X-roller config), body frame with
    // +x forward, +y left, +z CCW. Strafe (y) is a DIAGONAL wheel pattern
    // [FL-, FR+, RL+, RR-]; rotation (z) is a SIDE pattern [FL-, FR+, RL-, RR+].
    // (Verified on hardware 2026-07-13: the earlier RL/RR terms had y and w
    //  swapped, which made strafe rotate the robot and rotate strafe it.)
    float k = wheel_separation_x + wheel_separation_y;
    wheel_velocities[0] = linear_x - linear_y - angular_z * k; // FL
    wheel_velocities[1] = linear_x + linear_y + angular_z * k; // FR
    wheel_velocities[2] = linear_x + linear_y - angular_z * k; // RL
    wheel_velocities[3] = linear_x - linear_y + angular_z * k; // RR
}

/**
 * @brief Convert wheel velocities to robot velocities (forward kinematics)
 *
 * @param wheel_velocities Input array of wheel velocities [FL, FR, RL, RR]
 * @param linear_x Output forward/backward velocity (m/s)
 * @param linear_y Output left/right strafe velocity (m/s)
 * @param angular_z Output rotational velocity (rad/s)
 */
void mecanumDriveOdometry(float wheel_velocities[4], float *linear_x,
                         float *linear_y, float *angular_z) {
    float wheel_separation_x = ROBOT_CENTER_TO_WHEEL_X / 1000.0f; // mm -> m
    float wheel_separation_y = ROBOT_CENTER_TO_WHEEL_Y / 1000.0f; // mm -> m

    // Inverse of the corrected mecanum kinematics above (must stay in sync).
    *linear_x = (wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] + wheel_velocities[3]) / 4.0f;
    *linear_y = (-wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] - wheel_velocities[3]) / 4.0f;
    *angular_z = (-wheel_velocities[0] + wheel_velocities[1] - wheel_velocities[2] + wheel_velocities[3]) /
                 (4.0f * (wheel_separation_x + wheel_separation_y));
}
