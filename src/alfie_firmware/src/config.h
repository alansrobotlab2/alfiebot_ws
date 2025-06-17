#include <cstdint>
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "scservo/SCServo.h"

#include "memorystruct.h"
#include "commandstruct.h"

// #include <alfie_msgs/msg/servo_memory_map.h>
#include <alfie_msgs/msg/driver_state.h>
#include <alfie_msgs/msg/driver_cmd.h>
#include <alfie_msgs/srv/servo_service.h>

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

#define HEARTBEAT_TIMEOUT_MS 10

struct DriverConfig0 {
    
    uint8_t heartbeatTimeout = 10;

};

struct DriverConfig1 : DriverConfig0 {

};