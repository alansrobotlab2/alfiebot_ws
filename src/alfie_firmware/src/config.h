#ifndef _CONFIG_H_
#define _CONFIG_H_


/*
Same sourcecode for both boards with #if DRIVERBOARD defines for specifics
Set DRIVERBOARD to [0,1] and make sure it matches the usb port

/dev/ttyUSB0 = driverboard 0
/dev/ttyUSB1 = driverboard 1
*/
#define DRIVERBOARD 1



#if DRIVERBOARD == 0
// driver board 0
// /dev/ttyUSB0
// right arm 
// head
// drive wheels
#define NODENAME "driver0"
#define STATEPUBLISHER "driver0state"
#define CMDSUBSCRIBER "driver0cmd"
#define SERVOSERVICE "driver0servoservice"
#define PWMFREQUENCY 512
#define NUMSERVOS 10

#else 
// driver board 1
// /dev/ttyUSB1
// left arm
// eye lights
// shoulder limit switch
#define NODENAME "driver1"
#define STATEPUBLISHER "driver1state"
#define CMDSUBSCRIBER "driver1cmd"
#define SERVOSERVICE "driver1servoservice"
#define PWMFREQUENCY 8268
#define NUMSERVOS 7
#endif 



#include <cstdint>
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "scservo/SCServo.h"

#include "imu/IMU.h"

#include "memorystruct.h"
#include "commandstruct.h"

// #include <alfie_msgs/msg/servo_memory_map.h>
#include <alfie_msgs/msg/driver_state.h>
#include <alfie_msgs/msg/driver_cmd.h>
#include <alfie_msgs/srv/servo_service.h>

// baudrate for serial communication with micro-ROS agent
//#define BAUDRATE 115200
#define BAUDRATE 921600

// for motor drivers
// lower frequency allows for lower pwm values to work better

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

#define WATCHDOG_TIMEOUT_MS 100


#define HEARTBEAT_TIMEOUT_MS 10

//The following defines the ESP32 pin of the TB6612 control  
//Motor A
#define PWMA 25      
#define AIN2 17
#define AIN1 21

//Motor B
#define BIN1 22
#define BIN2 23
#define PWMB 26

//Define the precision of the PWM, with a precision of 8, the PWM value is in the range of 0-255 (2^8-1)
#define ANALOG_WRITE_BITS 8


//Defines PWM channel
#define CHANNEL_A 0
#define CHANNEL_B 1
#define RESOLUTION ANALOG_WRITE_BITS

// === === === MOTOR PIN DEFINITION Motor Related Pin Definition === === ===

// Define the pins for the A/B encoders. Each encoder is feedback by two signal wires, which are connected to the corresponding Hall component.
// For one encoder alone: the motor speed is calculated by detecting the frequency of one of the Hall components, and the direction of rotation is determined by judging the status of the other Hall sensor.
// The direction of rotation is determined by judging the high and low level status of the other Hall sensor.

// Encoder A pin definition
#define AENCA 35        // Encoder A input A_C2(B)
#define AENCB 34        // Encoder A input A_C1(A)

// Encoder B pin definition
#define BENCB 16        // Encoder B input B_C2(B)
#define BENCA 27        // Encoder B input B_C1(A)

// The reduction ratio of the motor, the motor speed of the geared motor and the output shaft speed are not the same
// For example, in the case of the DCGM3865 motor, the reduction ratio is 1:42, which means that the motor makes 42 revolutions and the output shaft makes 1 revolution.
// Corresponding to one revolution of the output shaft, the more revolutions the motor needs to make, the greater the reduction ratio and usually the greater the torque
// The following takes the DCGM3865 motor as an example (the reduction ratio is 1:42)
#define REDUCTION_RATIO 169

// Number of encoder lines, one revolution of the motor, the number of level changes of one Hall sensor of the encoder
#define PPR_NUM 11

// Number of level changes of one Hall sensor of the encoder for one revolution of the output shaft
#define SHAFT_PPR (REDUCTION_RATIO * PPR_NUM)

// Wheel radius in meters - PLACEHOLDER VALUE, MEASURE YOUR ACTUAL WHEEL
#define WHEEL_RADIUS_M 0.038f  // 38mm radius (76mm diameter) - compressed radius for alfiebot

// Wheel circumference in meters
#define WHEEL_CIRCUMFERENCE_M (2.0f * 3.14159265359f * WHEEL_RADIUS_M)

// Distance traveled per encoder pulse (meters per pulse)
#define METERS_PER_PULSE (WHEEL_CIRCUMFERENCE_M / SHAFT_PPR)


/*
struct DriverConfig0 {
    
    uint8_t heartbeatTimeout = 10;

};

struct DriverConfig1 : DriverConfig0 {

};
*/

#endif // _CONFIG_H_