#ifndef _STATUSSTRUCT_H_
#define _STATUSSTRUCT_H_

#include "scservo/INST.h"

// luckily the esp32 and the serial bus servos use the same endian format, so we can use the same struct for both

typedef struct {
    
    // u8 torqueSwitch;        // 00
    // u8 acceleration;        // 01
    // u16 targetLocation;     // 02
    // u16 runningTime;        // 04
    // u16 runningSpeed;       // 06
    // u16 torqueLimit;        // 08
    // u16 unassigned1;        // 10
    // u16 unassigned2;        // 12
    // u8 unassigned3;         // 14
    // u8 lockMark;            // 15
    u16 currentLocation;    // 16
    u16 currentSpeed;       // 18
    u16 currentLoad;        // 20
    u8 currentVoltage;      // 22
    u8 currentTemperature;  // 23
    u8 asyncWriteFlag;      // 24
    u8 servoStatus;         // 25
    u8 mobileSign;          // 26
    u16 unassigned4;        // 27          
    u16 currentCurrent;     // 29
} StatusStruct;

typedef union {
    StatusStruct status;
    u8 bytes[sizeof(StatusStruct)];
} StatusReplyBuf ;


#endif // _STATUSSTRUCT_H_