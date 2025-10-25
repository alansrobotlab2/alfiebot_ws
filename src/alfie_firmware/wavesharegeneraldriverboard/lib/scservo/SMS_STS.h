/*
 * SMS_STS.h
 * application layer for waveshare ST servos.
 * date: 2023.6.11 
 */

#pragma once

//memory table definition
//-------EPROM(read only)--------
#define SMS_STS_FIRMWARE_L 0
#define SMS_STS_FIRMWARE_H 1
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

//-------EPROM(read & write)--------
#define SMS_STS_ID 5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_RETURN_DELAY_TIME 7
#define SMS_STS_RESPONSE_STATUS_LEVEL 8
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_CW_DEAD 26
#define SMS_STS_CCW_DEAD 27
#define SMS_STS_OFS_L 31
#define SMS_STS_OFS_H 32
#define SMS_STS_MODE 33

//-------SRAM(read & write)--------
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_TORQUE_LIMIT_L 48
#define SMS_STS_TORQUE_LIMIT_H 49
#define SMS_STS_LOCK 55

//-------SRAM(read only)--------
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70

#include "SCSerial.h"

class SMS_STS : public SCSerial
{
public:
	/**
	 * @brief Default constructor for SMS_STS servo controller
	 */
	SMS_STS();
	
	/**
	 * @brief Constructor with endian configuration
	 * @param End Processor endian structure (0=little endian, 1=big endian)
	 */
	SMS_STS(u8 End);
	
	/**
	 * @brief Constructor with endian and response level configuration
	 * @param End Processor endian structure (0=little endian, 1=big endian)
	 * @param Level Servo response level (0=no response, 1=respond to all except broadcast)
	 */
	SMS_STS(u8 End, u8 Level);
	
	/**
	 * @brief Write position command with speed and acceleration to a single servo
	 * @param ID Servo ID (1-253, 254 for broadcast)
	 * @param Position Target position (-2048 to 2047, negative values use sign bit encoding)
	 * @param Speed Target speed (0-65535)
	 * @param ACC Acceleration value (0-255, default 0)
	 * @return 1 on success, 0 on failure
	 */
	virtual int WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);
	
	/**
	 * @brief Register position command for asynchronous execution (requires RegWriteAction to execute)
	 * @param ID Servo ID (1-253)
	 * @param Position Target position (-2048 to 2047, negative values use sign bit encoding)
	 * @param Speed Target speed (0-65535)
	 * @param ACC Acceleration value (0-255, default 0)
	 * @return 1 on success, 0 on failure
	 */
	virtual int RegWritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC = 0);
	
	/**
	 * @brief Synchronously write position commands to multiple servos at once
	 * @param ID Array of servo IDs
	 * @param IDN Number of servos in the array
	 * @param Position Array of target positions (one per servo, -2048 to 2047)
	 * @param Speed Array of target speeds (one per servo, can be NULL for default)
	 * @param ACC Array of acceleration values (one per servo, can be NULL for default)
	 */
	virtual void SyncWritePosEx(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[]);
	
	/**
	 * @brief Switch servo to continuous rotation (wheel) mode
	 * @param ID Servo ID (1-253)
	 * @return 1 on success, 0 on failure
	 */
	virtual int WheelMode(u8 ID);
	
	/**
	 * @brief Control speed in wheel mode
	 * @param ID Servo ID (1-253)
	 * @param Speed Target speed (-32767 to 32767, negative for reverse)
	 * @param ACC Acceleration value (0-255, default 0)
	 * @return 1 on success, 0 on failure
	 */
	virtual int WriteSpe(u8 ID, s16 Speed, u8 ACC = 0);
	
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
	 * @brief Calibrate current position as the servo's center/zero position
	 * @param ID Servo ID (1-253)
	 * @return 1 on success, 0 on failure
	 */
	virtual int CalibrationOfs(u8 ID);
	
	/**
	 * @brief Read all status information from servo into internal memory buffer
	 * @param ID Servo ID (1-253)
	 * @return Number of bytes read on success, -1 on failure
	 */
	virtual int FeedBack(int ID);
	
	/**
	 * @brief Read current position from servo
	 * @param ID Servo ID (1-253), or -1 to read from internal buffer after FeedBack()
	 * @return Current position (-2048 to 2047), or -1 on error
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
	 * @brief Read servo operating mode
	 * @param ID Servo ID (1-253), or -1 to read from internal buffer after FeedBack()
	 * @return Mode value (0=servo mode, 1=wheel mode), or -1 on error
	 */
	virtual int ReadMode(int ID);
private:
	u8 Mem[SMS_STS_PRESENT_CURRENT_H-SMS_STS_PRESENT_POSITION_L+1];
};

