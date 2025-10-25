#pragma once

#include "scservo/INST.h"

// luckily the esp32 and the serial bus servos use the same endian format, so we can use the same struct for both

#pragma pack(push, 1)
typedef struct {
    u8 torqueSwitch;        // 00
    u8 acceleration;        // 01
    u16 targetLocation;     // 02
} CmdStruct;
#pragma pack(pop)

typedef union {
    CmdStruct command;
    u8 bytes[sizeof(CmdStruct)];
} CommandBuf ;
