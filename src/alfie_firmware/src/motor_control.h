#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "config.h"

void initMotors();
void DriveA(int16_t pwm);
void DriveB(int16_t pwm);
void driveMotors();
void disableAllMotors();
void IRAM_ATTR A_wheel_pulse();
void IRAM_ATTR B_wheel_pulse();

#endif // _MOTOR_CONTROL_H_