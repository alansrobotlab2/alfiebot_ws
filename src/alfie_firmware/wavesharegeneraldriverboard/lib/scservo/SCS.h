/*
 * SCS.h
 * communication layer for waveshare serial bus servo
 * date: 2019.12.18 
 */

#pragma once

#include "INST.h"

class SCS{
public:
	/**
	 * @brief Default constructor for SCS communication layer
	 */
	SCS();
	
	/**
	 * @brief Constructor with endian configuration
	 * @param End Processor endian structure (0=little endian, 1=big endian)
	 */
	SCS(u8 End);
	
	/**
	 * @brief Constructor with endian and response level configuration
	 * @param End Processor endian structure (0=little endian, 1=big endian)
	 * @param Level Servo response level (0=no response, 1=respond to all except broadcast)
	 */
	SCS(u8 End, u8 Level);
	
	/**
	 * @brief General write command to servo memory
	 * @param ID Servo ID (1-253, 254 for broadcast)
	 * @param MemAddr Memory address to write to
	 * @param nDat Pointer to data buffer to write
	 * @param nLen Length of data to write in bytes
	 * @return 1 on success, 0 on failure
	 */
	int genWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);
	
	/**
	 * @brief Register write command for asynchronous execution (waits for RegWriteAction)
	 * @param ID Servo ID (1-253)
	 * @param MemAddr Memory address to write to
	 * @param nDat Pointer to data buffer to write
	 * @param nLen Length of data to write in bytes
	 * @return 1 on success, 0 on failure
	 */
	int regWrite(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen);
	
	/**
	 * @brief Trigger execution of all previously registered write commands
	 * @param ID Servo ID (1-253, 0xfe/254 for all servos)
	 * @return 1 on success, 0 on failure
	 */
	int RegWriteAction(u8 ID = 0xfe);
	
	/**
	 * @brief Synchronously write same data to multiple servos at once
	 * @param ID Array of servo IDs
	 * @param IDN Number of servos in the array
	 * @param MemAddr Starting memory address to write to
	 * @param nDat Pointer to data buffer (contains data for all servos)
	 * @param nLen Length of data per servo in bytes
	 */
	void syncWrite(u8 ID[], u8 IDN, u8 MemAddr, u8 *nDat, u8 nLen);
	
	/**
	 * @brief Write a single byte to servo memory
	 * @param ID Servo ID (1-253, 254 for broadcast)
	 * @param MemAddr Memory address to write to
	 * @param bDat Byte value to write
	 * @return 1 on success, 0 on failure
	 */
	int writeByte(u8 ID, u8 MemAddr, u8 bDat);
	
	/**
	 * @brief Write a 16-bit word (2 bytes) to servo memory
	 * @param ID Servo ID (1-253, 254 for broadcast)
	 * @param MemAddr Memory address to write to
	 * @param wDat 16-bit word value to write
	 * @return 1 on success, 0 on failure
	 */
	int writeWord(u8 ID, u8 MemAddr, u16 wDat);
	
	/**
	 * @brief Read data from servo memory
	 * @param ID Servo ID (1-253)
	 * @param MemAddr Memory address to read from
	 * @param nData Pointer to buffer to store read data
	 * @param nLen Number of bytes to read
	 * @return Number of bytes successfully read, 0 on failure
	 */
	int Read(u8 ID, u8 MemAddr, u8 *nData, u8 nLen);
	
	/**
	 * @brief Read a single byte from servo memory
	 * @param ID Servo ID (1-253)
	 * @param MemAddr Memory address to read from
	 * @return Byte value read, or -1 on timeout/error
	 */
	int readByte(u8 ID, u8 MemAddr);
	
	/**
	 * @brief Read a 16-bit word (2 bytes) from servo memory
	 * @param ID Servo ID (1-253)
	 * @param MemAddr Memory address to read from
	 * @return 16-bit word value read, or -1 on timeout/error
	 */
	int readWord(u8 ID, u8 MemAddr);
	
	/**
	 * @brief Send ping command to check servo connectivity
	 * @param ID Servo ID (1-253, 254 to ping all)
	 * @return Servo ID on success, -1 on timeout/error
	 */
	int Ping(u8 ID);
	
	/**
	 * @brief Transmit synchronous read command to multiple servos
	 * @param ID Array of servo IDs to read from
	 * @param IDN Number of servos in the array
	 * @param MemAddr Starting memory address to read from
	 * @param nLen Number of bytes to read from each servo
	 * @return Number of bytes to expect per servo
	 */
	int syncReadPacketTx(u8 ID[], u8 IDN, u8 MemAddr, u8 nLen);
	
	/**
	 * @brief Receive synchronous read response from a specific servo
	 * @param ID Servo ID to receive from
	 * @param nDat Pointer to buffer to store received data
	 * @return Number of bytes received on success, 0 on failure
	 */
	int syncReadPacketRx(u8 ID, u8 *nDat);
	
	/**
	 * @brief Decode one byte from the synchronous read response buffer
	 * @return Byte value, or -1 if buffer exhausted
	 */
	int syncReadRxPacketToByte();
	
	/**
	 * @brief Decode 16-bit word from the synchronous read response buffer
	 * @param negBit Bit position for sign interpretation (0 for unsigned, other values check this bit as sign)
	 * @return 16-bit word value, or -1 if buffer exhausted
	 */
	int syncReadRxPacketToWrod(u8 negBit=0);
public:
	u8 Level; // the level of the servo return
	u8 End; // processor endian structure
	u8 Error; // the status of servo
	u8 syncReadRxPacketIndex;
	u8 syncReadRxPacketLen;
	u8 *syncReadRxPacket;
protected:
	/**
	 * @brief Abstract method to write data array to serial port (implemented by derived classes)
	 * @param nDat Pointer to data array to write
	 * @param nLen Number of bytes to write
	 * @return Number of bytes written
	 */
	virtual int writeSCS(unsigned char *nDat, int nLen) = 0;
	
	/**
	 * @brief Abstract method to read data from serial port (implemented by derived classes)
	 * @param nDat Pointer to buffer to store read data
	 * @param nLen Number of bytes to read
	 * @return Number of bytes read
	 */
	virtual int readSCS(unsigned char *nDat, int nLen) = 0;
	
	/**
	 * @brief Abstract method to write single byte to serial port (implemented by derived classes)
	 * @param bDat Byte to write
	 * @return Number of bytes written (should be 1)
	 */
	virtual int writeSCS(unsigned char bDat) = 0;
	
	/**
	 * @brief Abstract method to flush/clear read buffer (implemented by derived classes)
	 */
	virtual void rFlushSCS() = 0;
	
	/**
	 * @brief Abstract method to flush/complete write buffer (implemented by derived classes)
	 */
	virtual void wFlushSCS() = 0;
	
protected:
	/**
	 * @brief Build and write a command packet buffer to the servo
	 * @param ID Servo ID
	 * @param MemAddr Memory address
	 * @param nDat Pointer to data payload
	 * @param nLen Length of data payload
	 * @param Fun Function/instruction code (INST_WRITE, INST_READ, etc.)
	 */
	void writeBuf(u8 ID, u8 MemAddr, u8 *nDat, u8 nLen, u8 Fun);
	
	/**
	 * @brief Convert 16-bit value from host endian to servo protocol endian format
	 * @param DataL Pointer to store low byte
	 * @param DataH Pointer to store high byte
	 * @param Data 16-bit value to convert
	 */
	void Host2SCS(u8 *DataL, u8* DataH, u16 Data);
	
	/**
	 * @brief Convert two bytes from servo protocol endian to host 16-bit value
	 * @param DataL Low byte
	 * @param DataH High byte
	 * @return Combined 16-bit value
	 */
	u16	SCS2Host(u8 DataL, u8 DataH);
	
	/**
	 * @brief Wait for and validate acknowledgement response from servo
	 * @param ID Expected servo ID in response
	 * @return 1 on valid acknowledgement, 0 on failure/timeout
	 */
	int	Ack(u8 ID);
	
	/**
	 * @brief Check for valid packet header (0xFF 0xFF) in serial stream
	 * @return 1 if valid header found, 0 on timeout/failure
	 */
	int checkHead();
};
