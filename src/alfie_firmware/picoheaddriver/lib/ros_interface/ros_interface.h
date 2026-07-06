/**
 * @file ros_interface.h
 * @brief micro-ROS interface for the head module.
 *
 * Node:      head_controller   (namespace NAMESPACE = "alfie/low")
 * Subscribe: headcmd    (alfie_msgs/HeadCmd)   best-effort
 * Publish:   headstate  (alfie_msgs/HeadState) best-effort
 *
 * Runs on Core 1 as a connection state machine (WAITING/AVAILABLE/CONNECTED/
 * DISCONNECTED). Incoming HeadCmd is decoded into b.mBuf[] + b.eye_pwm[]; the
 * outgoing HeadState is built from b.mBuf[] + b.eye_state[]. A command watchdog
 * disables servo torque and turns the eyes off if commands stop arriving.
 *
 * @author Alfie Bot Project
 */

#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <alfie_msgs/msg/head_cmd.h>
#include <alfie_msgs/msg/head_state.h>
#include "config.h"

// ROS state machine (shared with the status LED on Core 0).
extern RosAgentState_t agent_state;
extern bool ros_entities_created;
extern bool micro_ros_initialized;

void initializeRosInterface(void);
bool createRosEntities(void);
void destroyRosEntities(void);
void rosStateMachineTask(void);

void headCmdCallback(const void *msgin);
void publishHeadState(void);
void handleWatchdog(void);
