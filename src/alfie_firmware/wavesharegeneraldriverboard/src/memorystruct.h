#pragma once

#include "scservo/INST.h"

// luckily the esp32 and the serial bus servos use the same endian format, 
// so no byte swapping is needed for the structs.

#pragma pack(push, 1)
typedef struct {
    
    uint8_t firmwareMajor;     
    uint8_t firmwareSub; 
    uint8_t unassigned0;      
    uint8_t servoMajor;    
    uint8_t servoSub;       
    uint8_t servoID;       
    uint8_t baudRate;
    uint8_t returnDelay; 
    uint8_t responseStatusLevel;
    uint16_t minAngleLimit;
    uint16_t maxAngleLimit;
    uint8_t maxTempLimit;
    uint8_t minInputVoltage;
    uint8_t maxInputVoltage;
    uint16_t maxTorque;
    uint8_t phase;
    uint8_t unloadingCondition;
    uint8_t LEDAlarmCondition;
    uint8_t Pcoefficient;
    uint8_t Dcoefficient;
    uint8_t Icoefficient;
    uint16_t minStartupForce;
    uint8_t clockwiseInsensitiveArea;
    uint8_t counterclockwiseInsensitiveRegion;
    uint16_t protectionCurrent;
    uint8_t angularResolution;
    int16_t positionCorrection;
    uint8_t operationMode;
    uint8_t protectiveTorque;
    uint8_t protectionTime;
    uint8_t overloadTorque;
    uint8_t speedClosedLoopPcoefficient;
    uint8_t OvercurrentProtectionTime;
    uint8_t velocityClosedLoopIcoefficient;
    uint8_t torqueSwitch;       
    uint8_t acceleration;       
    int16_t targetLocation;    
    uint16_t runningTime;       
    uint16_t runningSpeed;      
    uint16_t torqueLimit;       
    uint8_t unassigned1;       
    uint8_t unassigned2;       
    uint8_t unassigned3;        
    uint8_t unassigned4;        
    uint8_t unassigned5;         
    uint8_t lockMark;
    int16_t currentLocation;
    int16_t currentSpeed;    
    uint16_t currentLoad;       
    uint8_t currentVoltage;     
    uint8_t currentTemperature; 
    uint8_t asyncWriteFlag;     
    uint8_t servoStatus;        
    uint8_t mobileSign;         
    uint8_t unassigned6;        
    uint8_t unassigned7;  
    uint16_t currentCurrent;  
} MemoryStruct;
#pragma pack(pop)

typedef union {
    MemoryStruct memory;
    uint8_t bytes[sizeof(MemoryStruct)];
} MemoryReplyBuf ;
