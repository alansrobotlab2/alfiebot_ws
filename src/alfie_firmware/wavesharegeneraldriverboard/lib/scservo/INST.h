/*
 * INST.h
 * directive definition header file for waveshare serial bus servos.
 * date: 2023.6.28 
 */

#pragma once

// Type definitions for cross-platform compatibility
typedef	char s8;               ///< Signed 8-bit integer
typedef	unsigned char u8;      ///< Unsigned 8-bit integer
typedef	unsigned short u16;    ///< Unsigned 16-bit integer
typedef	short s16;             ///< Signed 16-bit integer
typedef	unsigned long u32;     ///< Unsigned 32-bit integer
typedef	long s32;              ///< Signed 32-bit integer

// Servo protocol instruction codes
#define INST_PING 0x01         ///< Ping command - check servo connectivity
#define INST_READ 0x02         ///< Read command - read data from servo memory
#define INST_WRITE 0x03        ///< Write command - write data to servo memory immediately
#define INST_REG_WRITE 0x04    ///< Register write - queue write for later execution
#define INST_REG_ACTION 0x05   ///< Action command - execute all queued register writes
#define INST_SYNC_READ 0x82    ///< Synchronous read - read same data from multiple servos
#define INST_SYNC_WRITE 0x83   ///< Synchronous write - write same data to multiple servos

// Baud rate definitions (Chinese comment: 波特率定义)
#define	_1M 0                  ///< 1 Mbps baud rate
#define	_0_5M 1                ///< 500 Kbps baud rate
#define	_250K 2                ///< 250 Kbps baud rate
#define	_128K 3                ///< 128 Kbps baud rate
#define	_115200 4              ///< 115200 bps baud rate
#define	_76800 5               ///< 76800 bps baud rate
#define	_57600 6               ///< 57600 bps baud rate
#define	_38400 7               ///< 38400 bps baud rate
#define	_19200 8               ///< 19200 bps baud rate
#define	_14400 9               ///< 14400 bps baud rate
#define	_9600 10               ///< 9600 bps baud rate
#define	_4800 11               ///< 4800 bps baud rate

