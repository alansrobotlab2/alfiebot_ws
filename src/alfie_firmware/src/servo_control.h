#ifndef _SERVO_CONTROL_H_
#define _SERVO_CONTROL_H_

#include "config.h"
#include "driverboard.h"

extern DriverBoard b;

void getServoData();
void flushSerialServoLine();

/**
 * @brief Updates servo status by reading current servo states via sync read
 * 
 * Performs a synchronous read operation on all servos to get their current
 * status including position, speed, load, voltage, temperature, and other
 * parameters. Updates the global mBuf array with fresh servo data.
 * 
 * The function reads 15 bytes starting from memory address 56, which includes:
 * - Current location (position)
 * - Current speed
 * - Current load
 * - Current voltage  
 * - Current temperature
 * - Servo status flags
 * - Mobile sign and other status data
 * 
 * @note Called at 100Hz from the servo control loop
 * @note Blocks for serial communication duration (1-5ms depending on servo count)
 * @warning If any servo fails to respond, the function returns early and flushes the serial line
 * @see ServoLoop() for usage context
 * @see mBuf global array for where data is stored
 */
void updateServoStatus();
void updateServoCommands();

void disableAllServoTorques();
int16_t convertcorrection_fromservo(int16_t servovalue);
int16_t convertcorrection_toservo(int16_t controllervalue);

#endif
// _SERVO_CONTROL_H_