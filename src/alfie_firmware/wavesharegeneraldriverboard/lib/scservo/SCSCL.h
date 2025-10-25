/*
 * SCSCL.h
 * application layer for waveshare serial bus servo
 * date: 2023.6.23 
 */

#pragma once

//memory table definition
//-------EPROM(read only)--------
#define SCSCL_VERSION_L 3
#define SCSCL_VERSION_H 4

//-------EPROM(read & write)--------
#define SCSCL_ID 5
#define SCSCL_BAUD_RATE 6
#define SCSCL_MIN_ANGLE_LIMIT_L 9
#define SCSCL_MIN_ANGLE_LIMIT_H 10
#define SCSCL_MAX_ANGLE_LIMIT_L 11
#define SCSCL_MAX_ANGLE_LIMIT_H 12
#define SCSCL_CW_DEAD 26
#define SCSCL_CCW_DEAD 27

//-------SRAM(read & write)--------
#define SCSCL_TORQUE_ENABLE 40
#define SCSCL_GOAL_POSITION_L 42
#define SCSCL_GOAL_POSITION_H 43
#define SCSCL_GOAL_TIME_L 44
#define SCSCL_GOAL_TIME_H 45
#define SCSCL_GOAL_SPEED_L 46
#define SCSCL_GOAL_SPEED_H 47
#define SCSCL_LOCK 48

//-------SRAM(read & write)--------
#define SCSCL_PRESENT_POSITION_L 56
#define SCSCL_PRESENT_POSITION_H 57
#define SCSCL_PRESENT_SPEED_L 58
#define SCSCL_PRESENT_SPEED_H 59
#define SCSCL_PRESENT_LOAD_L 60
#define SCSCL_PRESENT_LOAD_H 61
#define SCSCL_PRESENT_VOLTAGE 62
#define SCSCL_PRESENT_TEMPERATURE 63
#define SCSCL_MOVING 66
#define SCSCL_PRESENT_CURRENT_L 69
#define SCSCL_PRESENT_CURRENT_H 70

#include "SCSerial.h"

class SCSCL : public SCSerial
{
public:
	/**
	 * @brief Default constructor for SCSCL servo controller
	 */
	SCSCL();
	
	/**
	 * @brief Constructor with endian configuration
	 * @param End Processor endian structure (0=little endian, 1=big endian)
	 */
	SCSCL(u8 End);
	
	/**
	 * @brief Constructor with endian and response level configuration
	 * @param End Processor endian structure (0=little endian, 1=big endian)
	 * @param Level Servo response level (0=no response, 1=respond to all except broadcast)
	 */
	SCSCL(u8 End, u8 Level);
	
	/**
	 * @brief Write position with timing control to a single servo
	 * @param ID Servo ID (1-253, 254 for broadcast)
	 * @param Position Target position (0-4095)
	 * @param Time Time to reach position in milliseconds (0-65535)
	 * @param Speed Maximum speed limit (0-65535)
	 * @return 1 on success, 0 on failure
	 */
	virtual int WritePos(u8 ID, u16 Position, u16 Time, u16 Speed);
	
	/**
	 * @brief Write position with speed control (extended version)
	 * @param ID Servo ID (1-253, 254 for broadcast)
	 * @param Position Target position (signed 16-bit)
	 * @param Speed Maximum speed limit (0-65535)
	 * @param ACC Acceleration (unused in SCSCL, provided for compatibility)
	 * @return 1 on success, 0 on failure
	 */
	virtual int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC);
	
	/**
	 * @brief Register position command for asynchronous execution (requires RegWriteAction to execute)
	 * @param ID Servo ID (1-253)
	 * @param Position Target position (0-4095)
	 * @param Time Time to reach position in milliseconds (0-65535)
	 * @param Speed Maximum speed limit (0-65535, default 0)
	 * @return 1 on success, 0 on failure
	 */
	virtual int RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0);
	
	/**
	 * @brief Synchronously write position commands to multiple servos at once
	 * @param ID Array of servo IDs
	 * @param IDN Number of servos in the array
	 * @param Position Array of target positions (one per servo)
	 * @param Time Array of time values (one per servo, can be NULL for default)
	 * @param Speed Array of speed values (one per servo, can be NULL for default)
	 */
	virtual void SyncWritePos(u8 ID[], u8 IDN, u16 Position[], u16 Time[], u16 Speed[]);
	
	/**
	 * @brief Switch servo to PWM mode by setting angle limits to zero
	 * @param ID Servo ID (1-253)
	 * @return 1 on success, 0 on failure
	 */
	virtual int PWMMode(u8 ID);
	
	/**
	 * @brief Control PWM output directly (only works in PWM mode)
	 * @param ID Servo ID (1-253)
	 * @param pwmOut PWM value (-1000 to 1000, negative for reverse)
	 * @return 1 on success, 0 on failure
	 */
	virtual int WritePWM(u8 ID, s16 pwmOut);
	
	/**
	 * @brief Enable or disable servo torque (motor power)
	 * @param ID Servo ID (1-253, 254 for broadcast)
	 * @param Enable 1 to enable torque, 0 to disable
	 * @return 1 on success, 0 on failure
	 */
	virtual int EnableTorque(u8 ID, u8 Enable);
	
	/**
	 * @brief Unlock EPROM to allow writing to permanent storage
	 * @param ID Servo ID (1-253)
	 * @return 1 on success, 0 on failure
	 */
	virtual int unLockEprom(u8 ID);
	
	/**
	 * @brief Lock EPROM to prevent writing to permanent storage
	 * @param ID Servo ID (1-253)
	 * @return 1 on success, 0 on failure
	 */
	virtual int LockEprom(u8 ID);
	
	/**
	 * @brief Read all status information from servo into internal memory buffer
	 * @param ID Servo ID (1-253)
	 * @return Number of bytes read on success, -1 on failure
	 */
	virtual int FeedBack(int ID);
	
	/**
	 * @brief Read current position from servo
	 * @param ID Servo ID (1-253), or -1 to read from internal buffer after FeedBack()
	 * @return Current position (0-4095), or -1 on error
	 */
	virtual int ReadPos(int ID);
	
	/**
	 * @brief Read current speed from servo
	 * @param ID Servo ID (1-253), or -1 to read from internal buffer after FeedBack()
	 * @return Current speed (-32767 to 32767, negative for reverse), or -1 on error
	 */
	virtual int ReadSpeed(int ID);
	
	/**
	 * @brief Read current load on servo motor
	 * @param ID Servo ID (1-253), or -1 to read from internal buffer after FeedBack()
	 * @return Current load (0-1000 where 1000 = 100% max load, negative for reverse direction), or -1 on error
	 */
	virtual int ReadLoad(int ID);
	
	/**
	 * @brief Read current input voltage
	 * @param ID Servo ID (1-253), or -1 to read from internal buffer after FeedBack()
	 * @return Voltage in units (multiply by 0.1V for actual voltage), or -1 on error
	 */
	virtual int ReadVoltage(int ID);
	
	/**
	 * @brief Read current internal temperature
	 * @param ID Servo ID (1-253), or -1 to read from internal buffer after FeedBack()
	 * @return Temperature in degrees Celsius, or -1 on error
	 */
	virtual int ReadTemper(int ID);
	
	/**
	 * @brief Read whether servo is currently moving
	 * @param ID Servo ID (1-253), or -1 to read from internal buffer after FeedBack()
	 * @return 1 if moving, 0 if stopped, -1 on error
	 */
	virtual int ReadMove(int ID);
	
	/**
	 * @brief Read current draw of servo motor
	 * @param ID Servo ID (1-253), or -1 to read from internal buffer after FeedBack()
	 * @return Current in milliamps (negative for reverse direction), or -1 on error
	 */
	virtual int ReadCurrent(int ID);
	
	/**
	 * @brief Read servo operating mode by checking angle limits
	 * @param ID Servo ID (1-253), or -1 to read from internal buffer after FeedBack()
	 * @return Mode value (0=servo mode, 3=PWM mode), or -1 on error
	 */
	virtual int ReadMode(int ID);
	
	/**
	 * @brief Calibrate center position (placeholder - not implemented for SCSCL)
	 * @param ID Servo ID (1-253)
	 * @return -1 (not supported)
	 */
	virtual int CalibrationOfs(u8 ID);
	
	/**
	 * @brief Read arbitrary value from servo memory address
	 * @param ID Servo ID (1-253)
	 * @param AddInput Memory address to read from
	 * @return 16-bit value at the specified address, or -1 on error
	 */
	virtual int ReadInfoValue(int ID, int AddInput);
private:
	u8 Mem[SCSCL_PRESENT_CURRENT_H-SCSCL_PRESENT_POSITION_L+1];
};

