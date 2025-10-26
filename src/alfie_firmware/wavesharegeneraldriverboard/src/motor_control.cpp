#include "config.h"
#include "motor_control.h"
#include "driverboard.h"

extern DriverBoard b;

void disableAllMotors()
{
  b.drivercmdbuf[0] = 0;
  b.drivercmdbuf[1] = 0;
  b.A_target_velocity = 0.0f;
  b.B_target_velocity = 0.0f;
  // Reset PID state
  b.A_velocity_error_integral = 0.0f;
  b.A_velocity_error_previous = 0.0f;
  b.B_velocity_error_integral = 0.0f;
  b.B_velocity_error_previous = 0.0f;
}

void driveMotorsDirectPWM() {
  DriveA(b.drivercmdbuf[0]);
  DriveB(b.drivercmdbuf[1]);
}

void driveMotors() {
  // Default to velocity control
  driveMotorsWithVelocityControl();
}

void initMotors(){
  //Setting the operating mode of the ESP32 pin used to control the TB6612FNG
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  //Set the channel, frequency, and precision of the ESP32 pin used to control the PWM outputs
  ledcSetup(CHANNEL_A, PWMFREQUENCY, RESOLUTION);
  //Bind PWMA pin to A-channel
  ledcAttachPin(PWMA, CHANNEL_A);

  ledcSetup(CHANNEL_B, PWMFREQUENCY, RESOLUTION);
  ledcAttachPin(PWMB, CHANNEL_B);

    // Setting the operating mode of the encoder-related pins
  // When using encoders, it is often necessary to enable pull-up resistors on their pins. This is because encoders usually use open-drain outputs or outputs that are passive (open collector).

  // When the encoder pins are configured in input mode, if no external pull-up resistor or internal pull-up resistor (using the INPUT_PULLUP option) is connected to the pins.
  // The level of the pin may drift or be in an indeterminate state. This may result in errors or instability when reading the encoder signals.

   // By using the INPUT_PULLUP option, you can enable an internal pull-up resistor on the pin to set the default level of the pin to high (logic 1).
  // This effectively prevents the pin level from drifting and ensures that the pin stays in the defined state when there is no external signal. When the encoder generates a signal
  // the level of the pin will change and the state change can be detected by means of an interrupt or polling.

  // Therefore, it is common practice to use the INPUT_PULLUP option when configuring encoder pins to improve the reliability and stability of the encoder signals.
  
  pinMode(BENCB , INPUT_PULLUP);
  pinMode(BENCA , INPUT_PULLUP);

  pinMode(AENCB , INPUT_PULLUP);
  pinMode(AENCA , INPUT_PULLUP);



}

void DriveA(int16_t pwm){
// expected values -255 to 255
// positive values move forward, negative values move backward
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    //Setting the direction of rotation of the A motor
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    //Setting the PWM output duty cycle of channel A
    ledcWrite(CHANNEL_A, pwm);
  } else if (pwm < 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(CHANNEL_A, -pwm);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(CHANNEL_A, 0);
  }

}

void DriveB(int16_t pwm){
  // expected values -255 to 255
  // positive values move forward, negative values move backward
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    //Setting the direction of rotation of the B motor
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    //Setting the PWM output duty cycle of channel B
    ledcWrite(CHANNEL_B, pwm);
  } else if (pwm < 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    ledcWrite(CHANNEL_B, -pwm);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(CHANNEL_B, 0);
  }

}

// IRAM_ATTR is a macro definition for modifying functions and variables.
// Indicates that the function or variable will be placed in IRAM (Instruction RAM), which is the instruction memory of the ESP32 chip.
// Functions and variables modified with IRAM_ATTR will be placed in IRAM by the compiler.
// This can increase the execution speed of these functions and variables, especially for code segments that need to be executed frequently.
// This can be useful in real-time demanding applications.

// The callback function for the interrupt function, refer to the attachInterrupt() function later, is called when the level of a particular Hall encoder
// function is called when the level of a particular Hall encoder changes from low to high.
// In this function, the levels of another Hall sensor along the way are judged to determine the direction of rotation.

void IRAM_ATTR B_wheel_pulse() {

  if(digitalRead(BENCA)){
    b.B_wheel_pulse_count--;
  }
  else{
    b.B_wheel_pulse_count++;
  }
}


void IRAM_ATTR A_wheel_pulse() {

  if(digitalRead(AENCA)){
    b.A_wheel_pulse_count--;
  }
  else{
    b.A_wheel_pulse_count++;
  }

}

void calculateMotorDynamics() {
  unsigned long current_time = millis();
  
  // Calculate Motor A dynamics
  if (b.A_last_update_time > 0) {
    float delta_time = (current_time - b.A_last_update_time) / 1000.0f; // Convert to seconds
    
    if (delta_time > 0.0f) {
      long delta_pulses = b.A_wheel_pulse_count - b.A_last_pulse_count;
      
      // Calculate velocity in pulses per second
      float pulses_per_second = delta_pulses / delta_time;
      
      // Convert to meters per second using wheel parameters
      // velocity (m/s) = pulses/sec * meters/pulse
      b.A_velocity = pulses_per_second * METERS_PER_PULSE;
      
      // Calculate acceleration in m/s^2
      b.A_acceleration = (b.A_velocity - b.A_last_velocity) / delta_time;
      
      // Determine if motor is moving (velocity threshold to account for noise)
      // Threshold: 5 pulses/sec = approximately 0.001 m/s for typical small wheels
      b.A_is_moving = (abs(pulses_per_second) > 5.0f);
      
      // Update last values
      b.A_last_velocity = b.A_velocity;
      b.A_last_pulse_count = b.A_wheel_pulse_count;
    }
  } else {
    // First run initialization
    b.A_last_pulse_count = b.A_wheel_pulse_count;
  }
  b.A_last_update_time = current_time;
  
  // Calculate Motor B dynamics
  if (b.B_last_update_time > 0) {
    float delta_time = (current_time - b.B_last_update_time) / 1000.0f; // Convert to seconds
    
    if (delta_time > 0.0f) {
      long delta_pulses = b.B_wheel_pulse_count - b.B_last_pulse_count;
      
      // Calculate velocity in pulses per second
      float pulses_per_second = delta_pulses / delta_time;
      
      // Convert to meters per second using wheel parameters
      // velocity (m/s) = pulses/sec * meters/pulse
      b.B_velocity = pulses_per_second * METERS_PER_PULSE;
      
      // Calculate acceleration in m/s^2
      b.B_acceleration = (b.B_velocity - b.B_last_velocity) / delta_time;
      
      // Determine if motor is moving (velocity threshold to account for noise)
      // Threshold: 5 pulses/sec = approximately 0.001 m/s for typical small wheels
      b.B_is_moving = (abs(pulses_per_second) > 5.0f);
      
      // Update last values
      b.B_last_velocity = b.B_velocity;
      b.B_last_pulse_count = b.B_wheel_pulse_count;
    }
  } else {
    // First run initialization
    b.B_last_pulse_count = b.B_wheel_pulse_count;
  }
  b.B_last_update_time = current_time;
}

int16_t calculateVelocityPID(float target_velocity, float current_velocity, 
                             float &error_integral, float &error_previous, float dt) {
  // Calculate velocity error
  float error = target_velocity - current_velocity;
  
  // Feedforward term - provides baseline PWM proportional to target velocity
  // Helps compensate for non-linear motor characteristics
  float FF = VELOCITY_FEEDFORWARD * target_velocity;
  
  // Proportional term - corrects based on current error
  float P = VELOCITY_KP * error;
  
  // Integral term with anti-windup - eliminates steady-state error
  error_integral += error * dt;
  error_integral = constrain(error_integral, -VELOCITY_INTEGRAL_MAX, VELOCITY_INTEGRAL_MAX);
  float I = VELOCITY_KI * error_integral;
  
  // Derivative term - dampens oscillation
  float error_derivative = (error - error_previous) / dt;
  float D = VELOCITY_KD * error_derivative;
  
  // Update previous error
  error_previous = error;
  
  // Calculate total output: Feedforward + PID
  float output = FF + P + I + D;
  
  // Dead zone for small target velocities
  if (abs(target_velocity) < MIN_VELOCITY_THRESHOLD) {
    output = 0.0f;
    error_integral = 0.0f;  // Reset integral when stopped
  }
  
  // Constrain output to PWM range [-255, 255]
  int16_t pwm = constrain((int16_t)output, -255, 255);
  
  return pwm;
  
  return pwm;
}

void setMotorATargetVelocity(float velocity) {
  b.A_target_velocity = constrain(velocity, -MAX_VELOCITY_MPS, MAX_VELOCITY_MPS);
}

void setMotorBTargetVelocity(float velocity) {
  b.B_target_velocity = constrain(velocity, -MAX_VELOCITY_MPS, MAX_VELOCITY_MPS);
}

void driveMotorsWithVelocityControl() {
  unsigned long current_time = millis();
  
  // Motor A velocity control
  if (b.A_pid_last_update_time > 0) {
    float dt = (current_time - b.A_pid_last_update_time) / 1000.0f;  // Convert to seconds
    
    if (dt > 0.0f) {
      int16_t pwm_a = calculateVelocityPID(
        b.A_target_velocity,
        b.A_velocity,
        b.A_velocity_error_integral,
        b.A_velocity_error_previous,
        dt
      );
      DriveA(pwm_a);
      b.drivercmdbuf[0] = pwm_a;  // Update for state reporting
    }
  }
  b.A_pid_last_update_time = current_time;
  
  // Motor B velocity control
  if (b.B_pid_last_update_time > 0) {
    float dt = (current_time - b.B_pid_last_update_time) / 1000.0f;  // Convert to seconds
    
    if (dt > 0.0f) {
      int16_t pwm_b = calculateVelocityPID(
        b.B_target_velocity,
        b.B_velocity,
        b.B_velocity_error_integral,
        b.B_velocity_error_previous,
        dt
      );
      DriveB(pwm_b);
      b.drivercmdbuf[1] = pwm_b;  // Update for state reporting
    }
  }
  b.B_pid_last_update_time = current_time;
}
