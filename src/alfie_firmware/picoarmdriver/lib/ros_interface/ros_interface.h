/**
 * @file ros_interface.h
 * @brief micro-ROS interface for the arm driver.
 *
 * Node:      arm_controller   (namespace = b.getNamespace(), left/right per serial)
 * Subscribe: armcmd    (alfie_msgs/ArmCmd)   best-effort  (6 logical joints)
 * Publish:   armstate  (alfie_msgs/ArmState) best-effort  (6 joints + board id)
 *
 * Runs on Core 1 as a connection state machine (WAITING/AVAILABLE/CONNECTED/
 * DISCONNECTED). Incoming ArmCmd is expanded into b.mBuf[] via applyArmCmd(); the
 * outgoing ArmState is built from b.mBuf[] via fillArmState(). A command watchdog
 * disables servo torque if commands stop arriving.
 *
 * @author Alfie Bot Project
 */

#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <alfie_msgs/msg/arm_cmd.h>
#include <alfie_msgs/msg/arm_state.h>
#include "config.h"

// ROS state machine (shared with the status LED on Core 0).
extern RosAgentState_t agent_state;
extern bool ros_entities_created;
extern bool micro_ros_initialized;

void initializeRosInterface(void);
bool createRosEntities(void);
void destroyRosEntities(void);
void rosStateMachineTask(void);

void armCmdCallback(const void *msgin);
void publishArmState(void);
void handleWatchdog(void);
