#include "config.h"
#include "motor_control.h"
#include "driverboard.h"

extern DriverBoard b;

void disableAllMotors()
{
  b.drivercmdbuf[0] = 0;
  b.drivercmdbuf[1] = 0;
}

void driveMotors() {
  DriveA(b.drivercmdbuf[0]);
  DriveB(b.drivercmdbuf[1]);
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
  
  #if DRIVERBOARD == 0
  pinMode(BENCB , INPUT_PULLUP);
  pinMode(BENCA , INPUT_PULLUP);

  pinMode(AENCB , INPUT_PULLUP);
  pinMode(AENCA , INPUT_PULLUP);
  #endif


}

void DriveA(int16_t pwm){
// expected values -255 to 255
// positive values move forward, negative values move backward
  pwm = constrain(pwm, -255, 255);

#if DRIVERBOARD == 1
// then this is the eye driver and they're wired backwards
// take the negative of the abs value for pwm
  pwm = -abs(pwm);
#endif

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

#if DRIVERBOARD == 1
// then this is the eye driver and they're wired backwards
// take the negative of the abs value for pwm
  pwm = -abs(pwm);
#endif

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
    b.B_wheel_pulse_count++;
  }
  else{
    b.B_wheel_pulse_count--;
  }
}

void IRAM_ATTR A_wheel_pulse() {
  if(digitalRead(AENCA)){
    b.A_wheel_pulse_count++;
  }
  else{
    b.A_wheel_pulse_count--;
  }
}