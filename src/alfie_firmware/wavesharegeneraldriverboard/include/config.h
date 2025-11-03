#pragma once


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
#define NODENAME "gdb0"
#define STATEPUBLISHER "gdb0state"
#define CMDSUBSCRIBER "gdb0cmd"
#define SERVOSERVICE "gdb0servoservice"
#define NUMSERVOS 10

#else 
// driver board 1
// /dev/ttyUSB1
// left arm
// eye lights
// shoulder limit switch
#define NODENAME "gdb1"
#define STATEPUBLISHER "gdb1state"
#define CMDSUBSCRIBER "gdb1cmd"
#define SERVOSERVICE "gdb1servoservice"
#define NUMSERVOS 7
#endif 

#define NAMESPACE "alfie"

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
#define REDUCTION_RATIO 169

// Number of encoder lines, one revolution of the motor, the number of level changes of one Hall sensor of the encoder
#define PPR_NUM 11

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
// Tuned for 100Hz update rate
// History:
//   - Original: Kp=400, Ki=150, Kd=8 (12s settling @ 0.1m/s - too slow)
//   - Update 1: Kp=1200, Ki=400, Kd=20 (4.6s settling @ 0.1m/s - better, no overshoot)
//   - Update 2: Kp=2400, Ki=800, Kd=30 (targeting 0.5s settling)
//   - Update 3: Adding feedforward to compensate for non-linear motor response (2025-10-25)
#define VELOCITY_KP 2400.0f   // Proportional gain - responsiveness to error
#define VELOCITY_KI 800.0f    // Integral gain - eliminates steady-state error
#define VELOCITY_KD 30.0f     // Derivative gain - reduces overshoot and oscillation

// Feedforward term - provides baseline PWM proportional to target velocity
// Compensates for non-linear motor characteristics (deadband, back-EMF, friction)
// Value represents PWM per m/s of target velocity
#define VELOCITY_FEEDFORWARD 600.0f   // FF gain - baseline PWM for target velocity

// Maximum integral windup limit (prevents integral term from growing too large)
#define VELOCITY_INTEGRAL_MAX 300.0f   // Increased to match higher Ki

// Minimum velocity threshold (m/s) - below this, set PWM to 0 to avoid stall
#define MIN_VELOCITY_THRESHOLD 0.005f

// PID control loop update rate (Hz)
#define VELOCITY_CONTROL_HZ 100.0f
#define VELOCITY_CONTROL_PERIOD_MS (1000.0f / VELOCITY_CONTROL_HZ)

// Maximum velocity limit (m/s) - safety limit
#define MAX_VELOCITY_MPS 1.0f


/*
struct DriverConfig0 {
    
    uint8_t heartbeatTimeout = 10;

};

struct DriverConfig1 : DriverConfig0 {

};
*/


