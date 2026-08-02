/**
 * @file motor_control.cpp
 * @brief Motor control and peripheral management implementation for linear actuator
 * 
 * @author Alfie Bot Project
 * @date 2025-10-27
 */

#include <Arduino.h>
#include <motor_control.h>
#include <../../include/driverboard.h>
#include "hardware/adc.h"

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

/**
 * @brief Read RP2040 internal temperature sensor
 * @return Temperature in degrees Celsius (rounded to uint8_t)
 */
uint8_t readBoardTemperature() {
    // Enable temperature sensor ADC
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4); // ADC4 is the temperature sensor
    
    // Read raw ADC value (12-bit)
    uint16_t raw = adc_read();
    
    // Convert to voltage (3.3V reference, 12-bit ADC)
    const float conversion_factor = 3.3f / (1 << 12);
    float voltage = raw * conversion_factor;
    
    // Convert to temperature using RP2040 formula
    // T = 27 - (ADC_voltage - 0.706) / 0.001721
    float temperature = 27.0f - (voltage - 0.706f) / 0.001721f;
    
    // Round and clamp to uint8_t range (0-255°C)
    if (temperature < 0) temperature = 0;
    if (temperature > 255) temperature = 255;
    
    return (uint8_t)(temperature + 0.5f); // Round to nearest integer
}

// =============================================================================
// DRIVERBOARD CLASS IMPLEMENTATION
// =============================================================================

/**
 * @brief DriverBoard constructor - initializes all state variables
 */
DriverBoard::DriverBoard() 
    : statusLED(WS2812B_PIN, 800000) {  // Initialize WS2812B on pin 16 at 800kHz
    // Initialize motor state
    motor.target_position = 0.0;
    motor.target_velocity = 0.0;
    motor.max_acceleration = MAX_ACTUATOR_ACCELERATION;
    motor.current_position = 0.0;
    motor.current_velocity = 0.0;
    motor.current_acceleration = 0.0;
    motor.encoder_count = 0;
    motor.last_pulse_time = 0;
    motor.pwm_output = 0;
    motor.velocity_error_integral = 0.0;
    motor.velocity_error_previous = 0.0;
    motor.ramped_velocity = 0.0;
    motor.fault_detected = false;
    motor.stall_detected = false;
    motor.motor_blocked = false;
    motor.stall_position = 0.0;
    motor.stall_count = 0;
    motor.fault_latched = false;
    motor.driver_fault = false;
    motor.runaway_detected = false;
    motor.is_moving = false;
    motor.encoder_a_state = false;
    motor.encoder_b_state = false;
    
    // Initialize robot status
    robot_status = ERROR_NONE;
    
    // Initialize encoder data
    encoder.count = 0;
    encoder.last_time = 0;
    encoder.direction = true;
    encoder.pin_a = MOTOR_ENCODER_A;
    encoder.pin_b = MOTOR_ENCODER_B;
    
    // Initialize inter-core communication
    actuator_cmd.position = 0.0;
    actuator_cmd.velocity = 0.0;
    actuator_cmd.acceleration = MAX_ACTUATOR_ACCELERATION;
    actuator_cmd.timestamp = 0;
    
    actuator_state.board_temp = 0;
    actuator_state.limit_switch_triggered = false;
    actuator_state.command_position = 0.0;
    actuator_state.command_velocity = 0.0;
    actuator_state.command_acceleration = 0.0;
    actuator_state.is_moving = false;
    actuator_state.current_position = 0.0;
    actuator_state.current_velocity = 0.0;
    actuator_state.current_acceleration = 0.0;
    actuator_state.pulses = 0;
    actuator_state.is_calibrated = false;
    actuator_state.error_code = ERROR_NONE;
    actuator_state.stall_position = 0.0;
    actuator_state.stall_count = 0;
    actuator_state.fault_latched = false;
    actuator_state.timestamp = 0;
    
    new_actuator_command = false;
    new_actuator_state = false;
    
    // Initialize calibration state
    calibration_in_progress = false;

    // Initialize IMU state
    imu_data.qw = 1.0; imu_data.qx = 0.0; imu_data.qy = 0.0; imu_data.qz = 0.0;
    imu_data.gyro_x = 0.0; imu_data.gyro_y = 0.0; imu_data.gyro_z = 0.0;
    imu_data.accel_x = 0.0; imu_data.accel_y = 0.0; imu_data.accel_z = 0.0;
    imu_data.valid = false;
    new_imu_data = false;
    imu_initialized = false;

}

// =============================================================================
// MOTOR CONTROL IMPLEMENTATION
// =============================================================================

/**
 * @brief Initialize all hardware peripherals
 * Sets up motor, encoder, and status LED
 */
void DriverBoard::initializePeripherals(void) {

    // Initialize WS2812B RGB LED
    if (statusLED.begin()) {
        // Set initial color to dim blue (initializing)
        statusLED.setColor(0, 0, 50);
    }
    
    // Initialize DRV8876 motor driver control pins.
    //
    // ORDER IS LOAD-BEARING. The rising edge on nSLEEP latches PMODE, so PMODE
    // must be driven first; and the bridge must be parked before it goes live.
    // Sequence: PMODE -> EN/PH parked -> nFAULT -> nSLEEP.

    // PMODE LOW selects PH/EN mode. This pin is driven rather than strapped, so
    // if it is ever left floating the part latches into independent half-bridge
    // mode, which inverts the down direction and disables current regulation.
    // Drive it hard and never switch it to INPUT.
    pinMode(MOTOR_PMODE_PIN, OUTPUT);
    digitalWrite(MOTOR_PMODE_PIN, LOW);

    pinMode(MOTOR_EN_PIN, OUTPUT);
    digitalWrite(MOTOR_EN_PIN, LOW);
    pinMode(MOTOR_PH_PIN, OUTPUT);
    digitalWrite(MOTOR_PH_PIN, MOTOR_PH_UP);

    // nFAULT is open-drain on the driver, so pull it up here. LOW = fault.
    pinMode(MOTOR_NFAULT_PIN, INPUT_PULLUP);

    // nSLEEP HIGH keeps the bridge active; LOW puts the outputs in Hi-Z (coast).
    // Datasheet tWAKE is 1 ms from nSLEEP rising to outputs live. This edge is
    // also what samples PMODE and IMODE, hence the ordering above.
    pinMode(MOTOR_NSLEEP_PIN, OUTPUT);
    digitalWrite(MOTOR_NSLEEP_PIN, HIGH);
    delay(2);

    // Make PWM_FREQUENCY real. Until this call existed the define was dead and
    // the bridge ran at whatever the core defaulted analogWrite() to. 4 kHz sits
    // above audible while staying long-period against the DRV8876's 25 us
    // current-regulation off-time - see the rationale in config.h.
    analogWriteFreq(PWM_FREQUENCY);
    analogWriteRange(PWM_MAX_DUTY);
    
    // Encoder pins with pull-up resistors
    pinMode(MOTOR_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_ENCODER_B, INPUT_PULLUP);
    
    // Limit switch pin - active high, goes LOW when triggered (at lower position limit)
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    
    // Setup encoder interrupts
    setupEncoderInterrupts();

    // Stop motor initially
    emergencyStop();

    // Initialize BNO085 IMU on I2C0 (non-fatal: telemetry only)
    imu_initialized = imuInit();
}

/**
 * @brief Main peripheral management loop
 * Should be called regularly from Core 0 loop
 */
void DriverBoard::updatePeripherals(void) {
    // Read encoder value
    readEncoder();
    
    // Update actuator state (position, velocity, acceleration)
    updateActuatorState();
    
    // Update motor control (PID control, PWM output)
    updateMotorControl();
    
    // Check for motor faults and safety conditions
    handleMotorSafety();
    
    // Update RGB LED status
    updateRgbLED();    // Update status LED based on robot state
    //updateStatusLED();

    // Poll BNO085 IMU (non-blocking; drains any pending events)
    if (imu_initialized && imuUpdate(imu_data)) {
        new_imu_data = true;
    }
}

/**
 * @brief Update motor control (PID control, PWM output)
 * Processes actuator commands and applies motor control with velocity PID and acceleration limiting
 * 
 * CONTROL STRATEGY:
 * - Position is the primary control target (actuator_cmd.position)
 * - Velocity is a CONSTRAINT specifying maximum speed to reach position (actuator_cmd.velocity)
 * - Acceleration is a CONSTRAINT specifying maximum acceleration (actuator_cmd.acceleration)
 * - The controller generates velocity setpoints from position error, clamped by velocity/accel limits
 * 
 * NOTE: Motor control is suspended during calibration to prevent interference
 */
void DriverBoard::updateMotorControl(void) {
    // Skip motor control during calibration
    if (calibration_in_progress) {
        return;
    }

    // A latched fault holds the bridge in brake until handleMotorSafety() clears
    // it. Enforced here as well as there because safety runs after this function
    // in updatePeripherals(), so without this the PID would re-command the motor
    // for one 2 ms cycle after every fault.
    if (motor.fault_detected) {
        analogWrite(MOTOR_EN_PIN, 0);
        motor.pwm_output = 0;
        motor.velocity_error_integral = 0.0;
        motor.velocity_error_previous = 0.0;
        motor.ramped_velocity = 0.0;
        return;
    }

    static uint32_t last_control_time = 0;
    uint32_t current_time = millis();
    float dt = (current_time - last_control_time) / 1000.0; // Convert to seconds
    
    // Initialize timing on first call
    if (last_control_time == 0) {
        last_control_time = current_time;
        return;
    }
    
    if (dt >= (CONTROL_LOOP_PERIOD_MS / 1000.0)) {
        last_control_time = current_time;
        
        // Get current actuator command
        motor.target_position = actuator_cmd.position;
        motor.max_acceleration = actuator_cmd.acceleration;
        
        // Clamp target position to limits
        motor.target_position = constrain(motor.target_position,
                                         ACTUATOR_MIN_POSITION,
                                         ACTUATOR_MAX_POSITION);
        
        // Clamp acceleration to maximum
        motor.max_acceleration = constrain(motor.max_acceleration, 
                                          0.0, 
                                          MAX_ACTUATOR_ACCELERATION);
        
        // Position control: Calculate velocity needed to reach target position
        // Velocity and acceleration from command are constraints, not direct control inputs
        float position_error = motor.target_position - motor.current_position;
        
        // Simple proportional position control to generate velocity setpoint
        const float POSITION_KP = 5.0; // Position gain - increased from 3.0 for faster settling
        float desired_velocity = POSITION_KP * position_error;
        
        // Limit velocity based on commanded max velocity (if non-zero) or system max
        // actuator_cmd.velocity specifies the MAXIMUM velocity to use, not a direct velocity command
        float max_vel = (actuator_cmd.velocity != 0.0) ? fabs(actuator_cmd.velocity) : MAX_ACTUATOR_VELOCITY;
        desired_velocity = constrain(desired_velocity, -max_vel, max_vel);
        
        // Stop if within tolerance
        if (fabs(position_error) < POSITION_TOLERANCE_M) {
            desired_velocity = 0.0;
        }
        
        motor.target_velocity = desired_velocity;
        
        // Clamp target velocity to maximum
        motor.target_velocity = constrain(motor.target_velocity, 
                                         -MAX_ACTUATOR_VELOCITY, 
                                         MAX_ACTUATOR_VELOCITY);
        
        // Apply acceleration limiting to get ramped velocity
        motor.ramped_velocity = applyAccelerationLimit(motor.target_velocity, 
                                                       motor.ramped_velocity,
                                                       motor.max_acceleration, 
                                                       dt);
        
        // Apply velocity PID controller
        motor.pwm_output = applyVelocityPID(motor.ramped_velocity, 
                                           motor.current_velocity, 
                                           dt);
        
        
        // Direction first, then magnitude. The old TB6612 code wrote PWM before
        // the DIR pins, which briefly applied the new duty in the old direction
        // on a sign flip.
        digitalWrite(MOTOR_PH_PIN, motor.pwm_output >= 0 ? MOTOR_PH_UP : MOTOR_PH_DOWN);
        analogWrite(MOTOR_EN_PIN, abs((int)motor.pwm_output));
    }
}

/**
 * @brief Read encoder value from motor
 * Updates encoder count for position/velocity calculations
 */
void DriverBoard::readEncoder(void) {
    // Copy encoder count from volatile interrupt data (atomic read)
    noInterrupts();
    int32_t current_count = encoder.count;
    interrupts();
    
    // Update motor encoder count
    motor.encoder_count = current_count;
}

/**
 * @brief Calculate actuator state from encoder
 * Updates position, velocity, and acceleration estimates
 */
void DriverBoard::updateActuatorState(void) {
    static uint32_t last_update_time = 0;
    static uint32_t last_temp_update = 0;
    static int32_t last_count = 0;
    uint32_t current_time = millis();
    float dt = (current_time - last_update_time) / 1000.0; // Convert to seconds
    
    // Initialize timing on first call
    if (last_update_time == 0) {
        last_update_time = current_time;
        last_temp_update = current_time;
        return;
    }
    
    if (dt >= 0.002) { // Update at 500Hz (2ms period)
        // Calculate position from encoder counts
        motor.current_position = (float)motor.encoder_count / ENCODER_COUNTS_PER_METER;
        
        // Calculate velocity from encoder pulses
        int32_t count_diff = motor.encoder_count - last_count;
        last_count = motor.encoder_count;
        
        // Store previous velocity for acceleration calculation
        float previous_velocity = motor.current_velocity;
        
        // Convert encoder counts to velocity (m/s)
        float raw_velocity = 0.0;
        if (dt > 0) {
            raw_velocity = ((float)count_diff / ENCODER_COUNTS_PER_METER) / dt;
            
            // Apply low-pass filter to smooth velocity (reduce encoder quantization noise)
            // Exponential moving average: filtered = alpha * new + (1-alpha) * old
            const float VELOCITY_FILTER_ALPHA = 0.3; // 0.3 = moderate filtering
            motor.current_velocity = VELOCITY_FILTER_ALPHA * raw_velocity + 
                                    (1.0 - VELOCITY_FILTER_ALPHA) * motor.current_velocity;
            
            // Calculate acceleration (change in velocity over time)
            motor.current_acceleration = (motor.current_velocity - previous_velocity) / dt;
            
            // Determine if motor is moving (velocity threshold to avoid noise)
            const float VELOCITY_THRESHOLD = 0.001; // 1 mm/s threshold
            motor.is_moving = (fabs(motor.current_velocity) > VELOCITY_THRESHOLD);
        }
        
        // Read board temperature every 1000ms (1 second) to avoid excessive ADC reads
        if (current_time - last_temp_update >= 1000) {
            actuator_state.board_temp = readBoardTemperature();
            last_temp_update = current_time;
        }
        
        // Limit switch is normally-open to GND with INPUT_PULLUP, so LOW = pressed.
        // CONFIRMED ON HARDWARE: at -5.19 mm (on the switch) the pin reads LOW; at
        // 0.00 mm (clear of it) it reads HIGH. The code previously tested == HIGH,
        // which inverted the sense - a disconnected switch read as permanently
        // triggered, and sitting ON the switch read as clear. The latter is the
        // dangerous half: calibration drives down "until triggered", so it would
        // have kept driving into the hard stop while already on the switch.
        actuator_state.limit_switch_triggered =
            (digitalRead(LIMIT_SWITCH_PIN) == LIMIT_SWITCH_ACTIVE_LEVEL);
        
        // Populate actuator state for ROS publishing
        actuator_state.command_position = actuator_cmd.position;
        actuator_state.command_velocity = actuator_cmd.velocity;
        actuator_state.command_acceleration = actuator_cmd.acceleration;
        actuator_state.is_moving = motor.is_moving;
        actuator_state.current_position = motor.current_position;
        actuator_state.current_velocity = motor.current_velocity;
        actuator_state.current_acceleration = motor.current_acceleration;
        actuator_state.pulses = motor.encoder_count;
        actuator_state.pwm_output = (uint8_t)abs(motor.pwm_output);
        // Fault state snapshot. handleMotorSafety() runs after this in
        // updatePeripherals(), so these lag by one 2 ms cycle - immaterial next
        // to the 50 Hz publish rate, and it keeps all Core 1 reads coming from
        // the same struct rather than reaching into Core 0's motor state.
        actuator_state.error_code = robot_status;
        actuator_state.stall_position = motor.stall_position;
        actuator_state.stall_count = motor.stall_count;
        actuator_state.fault_latched = motor.fault_latched;
        actuator_state.timestamp = current_time;
        new_actuator_state = true;
        
        last_update_time = current_time;
    }
}

/**
 * @brief Check for motor faults and safety conditions
 *
 * Two independent detectors, because IPROPI is strapped to GND and there is
 * therefore no current measurement anywhere in the system:
 *
 *  1. Stall - commanding real duty while the encoder reports no motion. This is
 *     the condition that destroyed the TB6612 and it is inferred, not measured.
 *  2. nFAULT - the driver's own signal. With current regulation disabled it has
 *     exactly one meaning: a real device fault (OCP / TSD / UVLO). No
 *     disambiguation against the commanded state is needed any more.
 *
 * Both latch, with different recovery. A driver fault re-arms itself with an
 * nSLEEP cycle on a timer. A stall latches soft and self-clears after a cooldown
 * for the first STALL_MAX_RETRIES attempts, then escalates to a hard latch that
 * only a command for a materially different position releases - so a transient
 * jam recovers unattended, but a real obstruction is not ground against forever.
 */
void DriverBoard::handleMotorSafety(void) {
    // stall_count and stall_position live on MotorState_t rather than here so
    // they can be published; everything else is private to this detector.
    static uint32_t stall_start_ms = 0;
    static uint32_t stall_latch_ms = 0;
    static float    last_target = 0.0f;
    static uint32_t moving_since_ms = 0;
    static bool     moved_this_attempt = false;
    static uint32_t runaway_start_ms = 0;
    static uint8_t  nfault_low_run = 0;
    static uint32_t driver_fault_ms = 0;

    const uint32_t now = millis();
    const uint16_t pwm_mag = (uint16_t)abs((int)motor.pwm_output);
    const bool nfault_low = (digitalRead(MOTOR_NFAULT_PIN) == LOW);

    // -------------------------------------------------------------------------
    // nFAULT: device fault
    // -------------------------------------------------------------------------
    // Current regulation is disabled (IPROPI strapped to GND), so nFAULT never
    // pulses for chopping. Any sustained low is a real fault - OCP, thermal
    // shutdown or UVLO - whether or not the bridge is being driven. Debounced
    // only because the line is held up by a weak internal pull-up.
    if (nfault_low) {
        if (nfault_low_run < 255) nfault_low_run++;
        if (nfault_low_run >= NFAULT_DEBOUNCE_SAMPLES && !motor.driver_fault) {
            motor.driver_fault = true;
            driver_fault_ms = now;
        }
    } else {
        nfault_low_run = 0;
    }

    // A latched driver fault is cleared on the DRV8876 side by an nSLEEP cycle,
    // which is also what re-latches PMODE/IMODE. MOTOR_PMODE_PIN is a GPIO held
    // LOW as an output, so it keeps its level across this cycle and PH/EN mode
    // re-latches correctly - no need to re-assert it here. Rate-limited so a
    // persistent fault does not become a fast re-arm loop. The ~3 ms of blocking
    // delay costs one control cycle, acceptable on a fault path.
    if (motor.driver_fault && (uint32_t)(now - driver_fault_ms) >= DRIVER_FAULT_RETRY_MS) {
        digitalWrite(MOTOR_NSLEEP_PIN, LOW);
        delayMicroseconds(1500);            // > tSLEEP (1 ms)
        digitalWrite(MOTOR_NSLEEP_PIN, HIGH);
        delayMicroseconds(1500);            // > tWAKE (1 ms)
        motor.driver_fault = false;
        nfault_low_run = 0;
        driver_fault_ms = now;
    }

    // -------------------------------------------------------------------------
    // Runaway detection - the inverse of a stall, and more dangerous
    // -------------------------------------------------------------------------
    // Skipped during calibration: that routine drives the bridge directly, so
    // motor.pwm_output is stale and the reversal test would compare against the
    // wrong command. Calibration is bounded by its own timeout instead.
    if (!calibration_in_progress && !motor.runaway_detected) {
        const float v = motor.current_velocity;

        // Faster than any speed we ever command. Sits above brake-limited free
        // fall (~0.063 m/s for 3 kg) and below no-load speed (0.222 m/s), so it
        // separates powered runaway from every legitimate motion.
        const bool overspeed = fabs(v) > RUNAWAY_VELOCITY_MPS;

        // Moving against the command while genuinely being driven. Catches the
        // sign inversions that overspeed alone would miss.
        const bool reversed = (pwm_mag >= STALL_PWM_THRESHOLD) &&
                              (fabs(v) > RUNAWAY_REVERSE_MPS) &&
                              ((motor.pwm_output > 0) != (v > 0));

        if (overspeed || reversed) {
            if (runaway_start_ms == 0) {
                runaway_start_ms = now;
            } else if ((uint32_t)(now - runaway_start_ms) >= RUNAWAY_CONFIRM_MS) {
                motor.runaway_detected = true;
                emergencyStop();
            }
        } else {
            runaway_start_ms = 0;
        }
    }

    // -------------------------------------------------------------------------
    // Stall detection
    // -------------------------------------------------------------------------
    const bool driving = (pwm_mag >= STALL_PWM_THRESHOLD);
    const bool stopped = (fabs(motor.current_velocity) < STALL_VELOCITY_MPS);

    // Track whether real motion was achieved during THIS move attempt. This is
    // the only evidence available to tell an obstruction from an overload - both
    // present identically as "duty commanded, encoder still".
    if (fabs(motor.target_position - last_target) > POSITION_TOLERANCE_M) {
        last_target = motor.target_position;
        moved_this_attempt = false;
        moving_since_ms = 0;
    }
    if (!stopped) {
        if (moving_since_ms == 0) {
            moving_since_ms = now;
        } else if ((uint32_t)(now - moving_since_ms) >= MOVED_CONFIRM_MS) {
            moved_this_attempt = true;
        }
        motor.stall_count = 0;    // real motion, so retire the retry count
    } else {
        moving_since_ms = 0;
    }

    if (driving && stopped && !motor.stall_detected) {
        if (stall_start_ms == 0) {
            stall_start_ms = now;
        } else if ((uint32_t)(now - stall_start_ms) >= STALL_TIMEOUT_MS) {
            motor.stall_detected = true;
            // Moving first, then stopped => something got in the way. Never
            // moving at all => could not break away from the load.
            motor.motor_blocked = moved_this_attempt;
            stall_latch_ms = now;
            if (motor.stall_count < 255) motor.stall_count++;
            // Brakes the bridge and parks actuator_cmd at the current position,
            // so latch the position AFTER it runs: the release test below asks
            // whether a later command moved away from where we gave up.
            emergencyStop();
            motor.stall_position = actuator_cmd.position;
        }
    } else {
        stall_start_ms = 0;
    }

    if (motor.stall_detected) {
        // A block hard-latches on the first occurrence: whatever stopped a moving
        // actuator could be a hand, and the soft-retry cycle would grind at it
        // three more times over nine seconds. An overload keeps the retries -
        // nothing is in the way and the load is simply heavy.
        if (!motor.motor_blocked && motor.stall_count < STALL_MAX_RETRIES) {
            // Soft latch: self-clearing, for transient jams. Harmless even if it
            // releases into the same condition - the command is parked at the
            // current position, so the PID has nothing to drive toward until a
            // real command arrives.
            if ((uint32_t)(now - stall_latch_ms) >= STALL_RECOVERY_MS) {
                motor.stall_detected = false;
                stall_start_ms = 0;
            }
        } else {
            // Hard latch: repeated stalls mean a genuine obstruction or a load
            // the gearing cannot lift. Only a command for a materially different
            // position releases it.
            if (fabs(actuator_cmd.position - motor.stall_position) > POSITION_TOLERANCE_M) {
                motor.stall_detected = false;
                motor.motor_blocked = false;
                stall_start_ms = 0;
                motor.stall_count = 0;
                moved_this_attempt = false;
            }
        }
    }

    // -------------------------------------------------------------------------
    // Roll up
    // -------------------------------------------------------------------------
    motor.fault_detected = motor.stall_detected || motor.driver_fault ||
                           motor.runaway_detected;

    // Hard latch = will not clear on its own, needs a materially different target.
    // A block latches hard on the first occurrence; an overload only after it has
    // used up its retries. A driver fault always re-arms itself on a timer. A
    // runaway never clears at all - only a reset does.
    motor.fault_latched = motor.runaway_detected ||
                          (motor.stall_detected &&
                           (motor.motor_blocked || motor.stall_count >= STALL_MAX_RETRIES));

    // Runaway outranks everything: it means the hardware is not what the firmware
    // believes it to be, so any other diagnosis derived from that belief is suspect.
    if (motor.runaway_detected) {
        robot_status = ERROR_MOTOR_RUNAWAY;
    } else if (motor.stall_detected) {
        robot_status = motor.motor_blocked ? ERROR_MOTOR_BLOCKED : ERROR_LOAD_EXCEEDED;
    } else if (motor.driver_fault) {
        robot_status = ERROR_DRIVER_FAULT;
    } else {
        robot_status = ERROR_NONE;
    }
}

/**
 * @brief Update status LED based on robot state
 * Provides visual feedback of robot status
 */
void DriverBoard::updateStatusLED(void) {
    // LED status is now handled in main.cpp Core 0 loop
}

/**
 * @brief Emergency stop motor
 * Immediately stops motor output for safety
 */
void DriverBoard::emergencyStop(void) {
    // EN=0 with nSLEEP HIGH is brake (low-side slow decay) on the DRV8876, which
    // is what this function always claimed to do. The old TB6612 code set both
    // DIR pins LOW, which on that part is Stop/high-impedance - a coast, not a
    // brake. At 56:1 the gearbox back-drives, so coasting drops the back.
    analogWrite(MOTOR_EN_PIN, 0);
    // nSLEEP stays HIGH to hold the brake. Taking it LOW would Hi-Z the bridge
    // and let the actuator free-fall.
    
    // Reset command (field-by-field assignment for volatile)
    actuator_cmd.position = motor.current_position;
    actuator_cmd.velocity = 0.0f;
    actuator_cmd.acceleration = MAX_ACTUATOR_ACCELERATION;
    actuator_cmd.timestamp = 0;
    
    // Reset PID state
    motor.velocity_error_integral = 0.0;
    motor.velocity_error_previous = 0.0;
    motor.ramped_velocity = 0.0;
}

/**
 * @brief Reset encoder counter to zero
 */
void DriverBoard::resetEncoders(void) {
    // Reset volatile encoder count (with interrupts disabled for atomic access)
    noInterrupts();
    encoder.count = 0;
    interrupts();
    
    // Reset motor encoder count and position
    motor.encoder_count = 0;
    motor.current_position = 0.0;
}


// =============================================================================
// ENCODER INTERRUPT FUNCTIONS
// =============================================================================

/**
 * @brief Setup encoder interrupts for motor
 * Attaches interrupt handlers to encoder pins
 */
void DriverBoard::setupEncoderInterrupts(void) {
    // Attach interrupts for encoder
    attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_A), encoderISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_B), encoderISR_B, CHANGE);
}

/**
 * @brief Quadrature transition table, indexed by (previous_state << 2) | current
 *
 * State is (A << 1) | B. Entries are +1 for a forward transition, -1 for
 * reverse, and 0 for both "nothing changed" and the illegal both-pins-changed
 * case. Forward is the sequence 00 -> 10 -> 11 -> 01 -> 00, which preserves the
 * sign convention of the decoder this replaced: up is still increasing count.
 */
static const int8_t QUAD_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

/**
 * @brief Process encoder interrupt
 *
 * REWRITTEN - the previous decoder lost position cumulatively. It read both pins
 * at interrupt time, inferred direction from their CURRENT levels, and counted
 * +/-1 unconditionally on every interrupt. Three ways that drifts:
 *
 *   1. No memory of the previous state, so it assumed the pin still held its
 *      post-edge value by the time digitalRead() ran. Under ISR latency or
 *      electrical dither it does not.
 *   2. It counted on EVERY interrupt, including glitches that never advanced the
 *      quadrature state. A correct decoder returns zero for a non-transition.
 *   3. Direction came from levels rather than from the transition, so a stale
 *      read counted the WRONG WAY - two counts of error, not one.
 *
 * Measured on hardware: the actuator ended 20 mm high after one full round trip
 * and 32 mm after two, growing every cycle, with no mechanical slip (pulleys
 * verified tight, no belt skip). The descent registered ~6% more counts per mm
 * than the ascent - descent is faster and jerkier (0.134 m/s peak against a
 * 0.040 steady), which is precisely when edge dither is worst.
 *
 * The table below counts only valid Gray-code transitions and yields 0 for both
 * "no change" and the illegal both-pins-changed case, so dither at an edge nets
 * to zero instead of accumulating. Resolution is unchanged at 4 counts per
 * quadrature cycle.
 *
 * @param pin_state Unused - retained for call-site compatibility. The state
 *                  table needs no knowledge of which pin fired.
 * @param is_pin_a  Unused, same reason.
 */
void DriverBoard::processEncoderInterrupt(bool pin_state, bool is_pin_a) {
    (void)pin_state;
    (void)is_pin_a;

    static uint8_t prev_state = 0xFF;    // 0xFF = uninitialised

    const bool state_a = digitalRead(encoder.pin_a);
    const bool state_b = digitalRead(encoder.pin_b);
    const uint8_t state = (uint8_t)((state_a ? 2 : 0) | (state_b ? 1 : 0));

    motor.encoder_a_state = state_a;
    motor.encoder_b_state = state_b;

    // First edge after boot only establishes the reference; counting a delta
    // against an unknown previous state would be a guess.
    if (prev_state == 0xFF) {
        prev_state = state;
        return;
    }

    const int8_t delta = QUAD_TABLE[(prev_state << 2) | state];
    prev_state = state;

    // Zero means no advance (dither or a glitch). Do not count it, and do not
    // let it refresh the pulse timing - a stalled axis that is merely vibrating
    // must still look stalled to the safety detectors.
    if (delta == 0) {
        return;
    }

    encoder.count += delta;
    encoder.direction = (delta > 0);
    encoder.last_time = micros();
    motor.last_pulse_time = encoder.last_time;

    motor.encoder_count = encoder.count;
}

// Interrupt Service Routines for encoder pins
void encoderISR_A(void) { rp.processEncoderInterrupt(digitalRead(MOTOR_ENCODER_A), true); }
void encoderISR_B(void) { rp.processEncoderInterrupt(digitalRead(MOTOR_ENCODER_B), false); }

// =============================================================================
// LINEAR ACTUATOR CONTROL FUNCTIONS
// =============================================================================

/**
 * @brief Apply velocity PID controller
 * 
 * @param target_velocity Desired velocity (m/s)
 * @param current_velocity Measured velocity (m/s)
 * @param dt Time step (seconds)
 * @return PWM output value (-255 to 255)
 */
int16_t applyVelocityPID(float target_velocity, float current_velocity, float dt) {
    // Calculate velocity error
    float error = target_velocity - current_velocity;
    
    // Proportional term
    float p_term = VELOCITY_PID_KP * error;
    
    // Integral term with anti-windup and reset when target velocity is zero
    // Reset integral when stopped to prevent wind-up during settling
    if (target_velocity == 0.0 && fabs(current_velocity) < 0.003) {
        // Reset integral when we want to be stopped and we're nearly stopped
        rp.motor.velocity_error_integral = 0.0;
    } else {
        rp.motor.velocity_error_integral += error * dt;
        rp.motor.velocity_error_integral = constrain(rp.motor.velocity_error_integral, 
                                                      -VELOCITY_PID_INTEGRAL_LIMIT, 
                                                      VELOCITY_PID_INTEGRAL_LIMIT);
    }
    float i_term = VELOCITY_PID_KI * rp.motor.velocity_error_integral;
    
    // Derivative term
    float d_term = 0.0;
    if (dt > 0) {
        float error_derivative = (error - rp.motor.velocity_error_previous) / dt;
        d_term = VELOCITY_PID_KD * error_derivative;
    }
    rp.motor.velocity_error_previous = error;
    
    // Calculate total PID output
    float pid_output = p_term + i_term + d_term;
    
    // Smoother deadband compensation to overcome static friction
    // Only add compensation when starting from zero velocity, not continuously
    // This prevents oscillation while still helping overcome initial stiction
    const float MIN_PWM_OFFSET_UP = 30.0;   // Offset for upward motion (against gravity)
    const float MIN_PWM_OFFSET_DOWN = 15.0; // Offset for downward motion (with gravity)
    const float VELOCITY_DEADZONE = 0.005;  // Velocity threshold for deadband application (m/s)
    
    // Only apply deadband compensation when:
    // 1. Target velocity is non-zero (we want to move)
    // 2. Current velocity is near zero (we're starting or stopped)
    // 3. PID output is small (might not overcome friction)
    if (target_velocity != 0.0 && fabs(current_velocity) < VELOCITY_DEADZONE) {
        if (pid_output > 0 && pid_output < MIN_PWM_OFFSET_UP) {
            // Add offset to help overcome friction when starting upward motion
            pid_output += MIN_PWM_OFFSET_UP;
        } else if (pid_output < 0 && pid_output > -MIN_PWM_OFFSET_DOWN) {
            // Add offset to help overcome friction when starting downward motion
            pid_output -= MIN_PWM_OFFSET_DOWN;
        }
    }
    
    // Clamp output to PWM range
    int16_t pwm_output = (int16_t)constrain(pid_output, 
                                            -VELOCITY_PID_OUTPUT_LIMIT, 
                                            VELOCITY_PID_OUTPUT_LIMIT);
    
    // Apply minimum PWM thresholds when any non-zero PWM is commanded
    // This ensures motor always has enough power to overcome friction and gravity
    if (pwm_output > 0 && pwm_output < MIN_PWM_UPWARD) {
        pwm_output = MIN_PWM_UPWARD;
    } else if (pwm_output < 0 && pwm_output > -MIN_PWM_DOWNWARD) {
        pwm_output = -MIN_PWM_DOWNWARD;
    }
    
    return pwm_output;
}

/**
 * @brief Apply acceleration limiting to velocity command
 * 
 * @param target_velocity Desired velocity (m/s)
 * @param current_velocity Current ramped velocity (m/s)
 * @param max_acceleration Maximum allowed acceleration (m/s²)
 * @param dt Time step (seconds)
 * @return Ramped velocity respecting acceleration limits (m/s)
 */
float applyAccelerationLimit(float target_velocity, float current_velocity, 
                             float max_acceleration, float dt) {
    if (dt <= 0) {
        return current_velocity;
    }
    
    // Calculate maximum velocity change allowed in this time step
    float max_velocity_change = max_acceleration * dt;
    
    // Calculate desired velocity change
    float velocity_diff = target_velocity - current_velocity;
    
    // Limit velocity change to maximum acceleration
    float limited_velocity_change = constrain(velocity_diff, 
                                              -max_velocity_change, 
                                              max_velocity_change);
    
    // Calculate ramped velocity
    float ramped_velocity = current_velocity + limited_velocity_change;
    
    return ramped_velocity;
}
/**
 * @brief Update RGB LED based on robot and ROS state
 * Provides visual feedback using WS2812B RGB LED with color coding:
 * - Blue: Initializing/Waiting for ROS agent
 * - Yellow: ROS agent connected, creating entities
 * - Green: Fully operational (agent connected, entities ready)
 * - Red: Error state or fault detected
 * - Cyan: Moving (when actuator is in motion)
 */
void DriverBoard::updateRgbLED(void) {
    static uint32_t last_update_time = 0;
    static uint8_t led_step = 0;
    uint32_t current_time = millis();
    
    // Use the global agent_state from ros_interface.cpp (managed by Core 1)
    // This is declared as extern in ros_interface.h
    extern RosAgentState_t agent_state;
    extern bool ros_entities_created;
    
    // Faults first. Solid vs blinking red separates "needs a human" from "will
    // try again on its own", which is the distinction that matters when you are
    // looking at the robot rather than at the topic.
    if (motor.fault_detected || robot_status != ERROR_NONE) {
        if (motor.runaway_detected) {
            // Fast red strobe - the most severe state, and unrecoverable without
            // a reset. Deliberately more urgent than either stall pattern.
            bool on = ((millis() / 80) % 2) == 0;
            statusLED.setColor(on ? 150 : 0, 0, 0);
        } else if (motor.motor_blocked) {
            statusLED.setColor(100, 0, 0);              // solid red - obstruction, hard latched
        } else if (motor.stall_detected) {
            bool on = ((millis() / 250) % 2) == 0;      // blinking red - overload, retrying
            statusLED.setColor(on ? 100 : 0, 0, 0);
        } else {
            statusLED.setColor(100, 0, 0);              // driver fault
        }
        return;
    }

    // Update LED pattern every LED_BLINK_PERIOD_MS (125ms) to match main.cpp pattern timing
    if (current_time - last_update_time >= LED_BLINK_PERIOD_MS) {
        last_update_time = current_time;
        
        // Define blink patterns (same as in main.cpp)
        // Normal pattern: 1,0,1,0,1,0,1,0 (when system loaded but ROS not connected)
        // Heartbeat pattern: 1,0,1,0,0,0,0,0 (when ROS is connected)
        const uint8_t normal_pattern[] = {1, 0, 1, 0, 1, 0, 1, 0};
        const uint8_t heartbeat_pattern[] = {1, 0, 1, 0, 0, 0, 0, 0};
        
        // Show state based on ROS connection and activity
        switch (agent_state) {
            case WAITING_AGENT:
                // Dim blue - waiting for ROS agent connection
                statusLED.setColor(0, 0, 50);
                break;
                
            case AGENT_AVAILABLE:
                // Yellow - agent connected, creating entities
                statusLED.setColor(100, 80, 0);
                break;
                
            case AGENT_CONNECTED:
                if (!ros_entities_created) {
                    // Yellow - agent connected, creating entities
                    statusLED.setColor(100, 80, 0);
                } else {
                    // Check if motor is moving
                    if (motor.is_moving) {
                        // Cyan - actively moving
                        statusLED.setColor(0, 100, 100);
                    } else {
                        // Green heartbeat - fully operational and idle
                        // Use heartbeat pattern: 1,0,1,0,0,0,0,0
                        if (heartbeat_pattern[led_step]) {
                            statusLED.setColor(0, 100, 0);  // Green ON
                        } else {
                            statusLED.setColor(0, 0, 0);    // OFF
                        }
                    }
                }
                break;
                
            case AGENT_DISCONNECTED:
                // Orange - agent disconnected
                statusLED.setColor(100, 30, 0);
                break;
                
            default:
                // Dim white - unknown state
                statusLED.setColor(20, 20, 20);
                break;
        }
        
        // Advance to next step (8 steps total, wraps around)
        led_step = (led_step + 1) % 8;
    }
}
