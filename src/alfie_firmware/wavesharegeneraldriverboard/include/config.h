#pragma once

/*
Board-specific configuration is now loaded at runtime from EEPROM.
Use the programmer to set the board ID (0 or 1) in EEPROM.

Board 0: /dev/ttyUSB0 - right arm, head, drive wheels
Board 1: /dev/ttyUSB1 - left arm, eye lights, shoulder limit switch
*/

#define NAMESPACE "alfie/low"

// PWM frequency - higher frequency = smoother motor control, especially important for faster motors
// 512 Hz proven optimal in experimental testing with 110 RPM motors
#define PWMFREQUENCY 512

#include <cstdint>
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "SCServo.h"

#include "IMU.h"

#include "memorystruct.h"
#include "commandstruct.h"

// #include <alfie_msgs/msg/servo_memory_map.h>
#include <alfie_msgs/msg/gdb_state.h>
#include <alfie_msgs/msg/gdb_cmd.h>
#include <alfie_msgs/srv/gdb_servo_service.h>

// baudrate for serial communication with micro-ROS agent
//#define BAUDRATE 115200
//#define BAUDRATE 921600
#define BAUDRATE 1500000

#define SERVOCOMMANDPACKETSIZE 9  // bytes per servo command packet


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
#define AENCA 34        // Encoder A input A_C2(B)
#define AENCB 35        // Encoder A input A_C1(A)

// Encoder B pin definition
#define BENCB 27        // Encoder B input B_C2(B)
#define BENCA 16        // Encoder B input B_C1(A)

// The reduction ratio of the motor, the motor speed of the geared motor and the output shaft speed are not the same
// For example, in the case of the DCGM3865 motor, the reduction ratio is 1:42, which means that the motor makes 42 revolutions and the output shaft makes 1 revolution.
// Corresponding to one revolution of the output shaft, the more revolutions the motor needs to make, the greater the reduction ratio and usually the greater the torque
// The following takes the DCGM3865 motor as an example (the reduction ratio is 1:42)
#define REDUCTION_RATIO 90

//
#define RPM 110 

// Number of encoder lines, one revolution of the motor, the number of level changes of one Hall sensor of the encoder
// Confirmed in experimental testing: 22 pulses per revolution
#define PPR_NUM 22

// Number of level changes of one Hall sensor of the encoder for one revolution of the output shaft
#define SHAFT_PPR (REDUCTION_RATIO * PPR_NUM)

// Wheel radius in meters - PLACEHOLDER VALUE, MEASURE YOUR ACTUAL WHEEL
#define WHEEL_RADIUS_M 0.040f  // 40mm radius (80mm diameter) - compressed radius for alfiebot

// Wheel circumference in meters
#define WHEEL_CIRCUMFERENCE_M (2.0f * 3.14159265359f * WHEEL_RADIUS_M)

// Distance traveled per encoder pulse (meters per pulse)
#define METERS_PER_PULSE (WHEEL_CIRCUMFERENCE_M / SHAFT_PPR)

// === PID VELOCITY CONTROL PARAMETERS ===
// PID gains for wheel velocity control (m/s)
// Tuned for 100Hz update rate with experimental testing
// These values proven to work in standalone mecanum project testing
// Kp=800, Ki=400, Kd=5 provides stable control without oscillation
#define VELOCITY_KP 800.0f     // Proportional gain - primary control
#define VELOCITY_KI 400.0f     // Integral gain - eliminates steady-state error
#define VELOCITY_KD 5.0f       // Derivative gain - damping

// Maximum integral windup limit (prevents integral term from growing too large)
#define VELOCITY_INTEGRAL_MAX 50.0f   // Anti-windup limit

// Minimum PWM to overcome static friction
// Experimental value: motors don't move below ~59 PWM
#define MIN_PWM 32

// Minimum velocity threshold (m/s) - below this, set PWM to 0 to avoid stall
#define MIN_VELOCITY_THRESHOLD 0.0025f

// PID control loop update rate (Hz)
// 100Hz proven optimal in experimental testing
#define VELOCITY_CONTROL_HZ 100.0f
#define VELOCITY_CONTROL_PERIOD_MS (1000.0f / VELOCITY_CONTROL_HZ)

// Maximum velocity limit (m/s) - safety limit
// Experimental calculated: ~0.73 m/s with PWM saturated at 255
#define MAX_VELOCITY_MPS 0.70f



