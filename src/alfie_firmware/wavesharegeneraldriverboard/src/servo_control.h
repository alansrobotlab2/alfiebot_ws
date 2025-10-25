#pragma once

#include "config.h"
#include "driverboard.h"
#include "scservo/INST.h"

extern DriverBoard b;

#define SBS_POSITIONCORRECTION  0x1F

#define SBS_TORQUEENABLE        0x28
#define SBS_ACCELERATION        0x29
#define SBS_TARGETLOCATION      0x2A

#define SBS_CURRENTLOCATION     0x38
#define SBS_CURRENTSPEED        0x3A
#define SBS_CURRENTLOAD         0x3C
#define SBS_CURRENTVOLTAGE      0x3E
#define SBS_CURRENTTEMPERATURE  0x3F
#define SBS_STATUSFLAGS         0x40
#define SBS_SERVOSTATUS         0x41
#define SBS_MOVEFLAG            0x42
#define SBS_CURRENTCURRENT      0x45


/**
 * @brief Flushes the serial servo communication line to reset packet synchronization
 * 
 * Sends ping commands to all servos to interrupt and reset the synchronous
 * communication protocol. This is used as a recovery mechanism when servo
 * communication packets get out of sync, which can cause the control loop
 * to slow down from 8ms to 120ms with corrupted data.
 * 
 * The function iterates through all servos (NUMSERVOS) and sends individual
 * ping commands to reset the communication state.
 * 
 * @note Called automatically by updateServoStatus() when a servo fails to respond
 * @note This is a workaround solution but works effectively to maintain sync
 * @see updateServoStatus() for automatic usage context
 * @warning Adds communication overhead but necessary for protocol stability
 */
void flushSerialServoLine();


/**
 * @brief Updates servo status by reading current servo states via sync read
 * 
 * Performs a synchronous read operation on all servos to get their current
 * status including position, speed, load, voltage, temperature, and other
 * parameters. Updates the global mBuf array with fresh servo data.
 * 
 * The function reads 13 bytes starting from memory address 0x38 (SBS_CURRENTLOCATION), which includes:
 * - Current location (position) - 2 bytes at 0x38
 * - Current speed - 2 bytes at 0x3A
 * - Current load - 2 bytes at 0x3C
 * - Current voltage - 1 byte at 0x3E
 * - Current temperature - 1 byte at 0x3F
 * - Status flags - 1 byte at 0x40
 * - Servo status - 1 byte at 0x41
 * - Move flag - 1 byte at 0x42
 * - Additional status - 2 bytes
 * 
 * @note Called at 100Hz from the servo control loop
 * @note Blocks for serial communication duration (1-5ms depending on servo count)
 * @warning If any servo fails to respond, the function returns early and flushes the serial line
 * @see ServoLoop() for usage context
 * @see mBuf global array for where data is stored
 */
void updateServoStatus();


/**
 * @brief Disables torque on servos that have torqueSwitch set to 0
 * 
 * First stage of servo command processing - identifies servos with torqueSwitch = 0
 * and sends a synchronous write command to disable torque on those servos immediately.
 * This provides immediate safety control by disabling servos that should not be active.
 * 
 * The function:
 * 1. Iterates through all servos in mBuf array
 * 2. Builds a list of servo IDs that have torqueSwitch = 0
 * 3. Sends sync write to memory address 0x28 (SBS_TORQUEENABLE register)
 * 4. Transmits 1 byte (value 0) to disable torque on identified servos
 * 
 * @note Called as part of servo control loop for safety management
 * @note Uses sync write for efficient multi-servo communication
 * @see updateServoActive() for the complementary active servo management
 * @see mBuf global array for torque switch state source
 */
void updateServoIdle();


/**
 * @brief Sends position and acceleration commands to active servos
 * 
 * Second stage of servo command processing - identifies servos with torqueSwitch = 1
 * and sends acceleration and position command data via sync write. This function
 * only transmits commands if at least one servo is active.
 * 
 * The function:
 * 1. Iterates through servos with torqueSwitch = 1 (active servos)
 * 2. Extracts acceleration (1 byte) and position (2 bytes) data from mBuf memory structure
 * 3. Formats 3 bytes per servo into command buffer: [acceleration, position_low, position_high]
 * 4. Sends sync write to memory address 0x29 (register 41) with 3 bytes per servo
 * 5. Only transmits if at least one servo is active (index > 0)
 * 
 * @note Position commands automatically enable torque when sent to servo
 * @note Uses memory address 0x29 which maps to acceleration register, followed by target position
 * @note Called in coordination with updateServoIdle() for complete servo management
 * @see updateServoIdle() for torque disable functionality
 * @see mBuf global array for command data source
 */
void updateServoActive();


/**
 * @brief Disables torque on all servos by setting torqueSwitch to 0
 * 
 * Iterates through all servos and sets their torqueSwitch value to 0 in the
 * local memory buffer (mBuf). This prepares the servos to be disabled on the
 * next call to updateServoIdle(). Used for emergency stops, watchdog
 * timeouts, or controlled shutdown sequences.
 * 
 * @note Only updates local buffer - actual servo disable happens in updateServoIdle()
 * @note Called automatically during watchdog timeout conditions
 * @see updateServoIdle() for actual transmission to servos
 * @warning This only stages the command - servo torque remains active until next update cycle
 */
void disableAllServoTorques();


/**
 * @brief Converts 12-bit signed servo position value to 16-bit signed value
 * 
 * Servo position values are transmitted as 12-bit signed integers in the range
 * -2048 to +2047, but stored with a special encoding where negative values
 * have bit 11 (0x800) set and the magnitude stored in the lower 11 bits.
 * This function converts from the servo's encoded format to a standard
 * 16-bit signed integer.
 * 
 * @param servovalue Raw 12-bit encoded position value from servo
 * @return Standard 16-bit signed position value
 * @note Used when reading position data from servo memory
 * @see positioncorrectionto12bitservo() for the inverse conversion
 */
int16_t positioncorrectionfrom12bitservo(int16_t servovalue);


/**
 * @brief Converts 16-bit signed value to 12-bit signed servo position format
 * 
 * Converts standard 16-bit signed position values to the servo's 12-bit
 * encoded format. Negative values are encoded by setting bit 11 (0x800)
 * and storing the absolute value in the lower 11 bits. Positive values
 * are stored directly.
 * 
 * @param controllervalue Standard 16-bit signed position value  
 * @return 12-bit encoded position value for servo transmission
 * @note Used when sending position commands to servos
 * @see positioncorrectionfrom12bitservo() for the inverse conversion
 * @warning Input values outside ±2047 range may cause undefined behavior
 */
int16_t positioncorrectionto12bitservo(int16_t controllervalue);


