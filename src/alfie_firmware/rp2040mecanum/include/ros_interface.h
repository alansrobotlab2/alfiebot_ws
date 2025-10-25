/**
 * @file ros_interface.h
 * @brief ROS2 micro-ROS interface for mecanum drive robot
 * 
 * This module handles all ROS2 communication including:
 * - Micro-ROS initialization and management
 * - Velocity command subscription (/mecanumdrive)
 * - Odometry publishing (/odom)
 * - Inter-core communication with motor control
 * 
 * @author Alfie Bot Project
 * @date 2025-10-24
 */

#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include "config.h"

// =============================================================================
// ROS INTERFACE DATA STRUCTURES
// =============================================================================

/**
 * @brief Robot velocity command structure
 * Matches geometry_msgs/Twist linear and angular components
 */
typedef struct {
    float linear_x;     ///< Forward/backward velocity (m/s)
    float linear_y;     ///< Left/right strafe velocity (m/s)
    float angular_z;    ///< Rotational velocity (rad/s)
    uint32_t timestamp; ///< Command timestamp for timeout detection
} VelocityCommand_t;

/**
 * @brief Robot odometry data structure
 */
typedef struct {
    float position_x;       ///< Robot X position (m)
    float position_y;       ///< Robot Y position (m)
    float orientation;      ///< Robot orientation (rad)
    float linear_velocity_x;    ///< Current linear X velocity (m/s)
    float linear_velocity_y;    ///< Current linear Y velocity (m/s)
    float angular_velocity;     ///< Current angular velocity (rad/s)
    uint32_t timestamp;     ///< Odometry timestamp
} Odometry_t;

// =============================================================================
// GLOBAL ROS INTERFACE VARIABLES
// =============================================================================

// Micro-ROS entities
extern rcl_allocator_t allocator;
extern rclc_support_t support;
extern rcl_node_t node;
extern rclc_executor_t executor;

// Subscribers and Publishers
extern rcl_subscription_t mecanum_subscriber;
extern rcl_publisher_t odom_publisher;

// Message instances
extern geometry_msgs__msg__Twist twist_msg;
extern nav_msgs__msg__Odometry odom_msg;

// ROS state
extern bool micro_ros_initialized;
extern uint32_t last_command_time;

// ROS State Machine Variables
extern RosAgentState_t agent_state;
extern uint32_t last_state_time;
extern bool ros_entities_created;

// Inter-core communication
extern volatile VelocityCommand_t g_velocity_cmd;
extern volatile Odometry_t g_odometry;
extern volatile bool new_velocity_command;
extern volatile bool new_odometry_data;

// =============================================================================
// ROS INTERFACE FUNCTION DECLARATIONS
// =============================================================================

/**
 * @brief Initialize ROS2 communication systems
 * Sets up micro-ROS transport and initializes state machine
 */
void initializeRosInterface(void);

/**
 * @brief Main ROS communication loop (deprecated - use rosStateMachineTask instead)
 * Should be called regularly from Core 1 loop
 */
void updateRosInterface(void);

/**
 * @brief Callback function for /mecanumdrive topic subscriber
 * Receives geometry_msgs/Twist messages and updates velocity command
 * 
 * @param msgin Pointer to incoming Twist message
 */
void mecanumDriveCallback(const void *msgin);

/**
 * @brief Process velocity commands
 * Handles new velocity commands received from ROS
 */
void processVelocityCommand(void);

/**
 * @brief Publish odometry data to ROS2
 * Sends current robot odometry to /odom topic
 */
void publishOdometry(void);

/**
 * @brief Monitor communication watchdog
 * Checks for command timeouts and safety conditions
 */
void handleWatchdog(void);

/**
 * @brief Get system time in milliseconds
 * @return Current system time in milliseconds
 */
uint32_t getSystemTimeMs(void);

/**
 * @brief Create all ROS entities (node, subscribers, publishers, executor)
 * @return true if successful, false if failed
 */
bool createRosEntities(void);

/**
 * @brief Destroy all ROS entities and cleanup resources
 */
void destroyRosEntities(void);

/**
 * @brief ROS state machine task function
 * Implements the main ROS connectivity state machine
 */
void rosStateMachineTask(void);

#endif // ROS_INTERFACE_H