/*
 * SCSerial.h
 * hardware interface layer for waveshare serial bus servo
 * date: 2023.6.28 
 */

#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "SCS.h"

class SCSerial : public SCS
{
public:
	/**
	 * @brief Default constructor for SCSerial hardware interface layer
	 */
	SCSerial();
	
	/**
	 * @brief Constructor with endian configuration
	 * @param End Processor endian structure (0=little endian, 1=big endian)
	 */
	SCSerial(u8 End);
	
	/**
	 * @brief Constructor with endian and response level configuration
	 * @param End Processor endian structure (0=little endian, 1=big endian)
	 * @param Level Servo response level (0=no response, 1=respond to all except broadcast)
	 */
	SCSerial(u8 End, u8 Level);

protected:
	/**
	 * @brief Write data array to serial port
	 * @param nDat Pointer to data array to write
	 * @param nLen Number of bytes to write
	 * @return Number of bytes successfully written
	 */
	virtual int writeSCS(unsigned char *nDat, int nLen);
	
	/**
	 * @brief Read data from serial port with timeout
	 * @param nDat Pointer to buffer to store read data
	 * @param nLen Number of bytes to read
	 * @return Number of bytes successfully read
	 */
	virtual int readSCS(unsigned char *nDat, int nLen);
	
	/**
	 * @brief Write single byte to serial port
	 * @param bDat Byte to write
	 * @return Number of bytes written (1 on success)
	 */
	virtual int writeSCS(unsigned char bDat);
	
	/**
	 * @brief Flush/clear all data from serial read buffer
	 */
	virtual void rFlushSCS();
	
	/**
	 * @brief Flush/complete serial write buffer (no-op in current implementation)
	 */
	virtual void wFlushSCS();
	
public:
	/**
	 * @brief I/O timeout duration in milliseconds (default: 100ms)
	 */
	unsigned long int IOTimeOut;
	
	/**
	 * @brief Pointer to hardware serial port interface
	 */
	HardwareSerial *pSerial;
	
	/**
	 * @brief Error flag for operation status
	 */
	int Err;
	
public:
	/**
	 * @brief Get the current error status
	 * @return Error code (0 = no error, non-zero = error occurred)
	 */
	virtual int getErr(){  return Err;  }
};

