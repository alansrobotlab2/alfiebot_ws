/**
 * @file arm_control.h
 * @brief Arm identity + 6-joint <-> 7-servo mapping.
 *
 * The arm exposes 6 logical joints over ROS; physically it has 7 servos because
 * the shoulder-pitch joint is a mirrored coupled pair. This module reproduces
 * the expansion/collapse that used to run host-side in alfie_bringup
 * (master_cmd.py / master_status.py / servo_config.py):
 *
 *   - armSelectSide() : read the Pico serial, pick left/right, polarity + namespace.
 *   - applyArmCmd()   : 6 joint commands -> 7 servo register writes (b.mBuf).
 *   - fillArmState()  : 7 servo feedbacks -> 6 joint states (primary-only pair).
 *
 * @author Alfie Bot Project
 */

#pragma once

#include <Arduino.h>
#include <alfie_msgs/msg/arm_cmd.h>
#include <alfie_msgs/msg/arm_state.h>

/// Read the Pico unique board serial and set b.arm_side / b.polarity /
/// b.namespace_str / b.board_serial(_str). Call once at boot (Core 0) before
/// the micro-ROS node is created.
void armSelectSide();

/// Expand a 6-joint ArmCmd into the 7-servo register mirror b.mBuf[]. Applies
/// per-side polarity and the shoulder-pitch mirror. Forces all torque off if the
/// board side is unknown (never drive with an unknown polarity).
void applyArmCmd(const alfie_msgs__msg__ArmCmd *msg);

/// Collapse the 7-servo feedback in b.mBuf[] into a 6-joint ArmState (the derived
/// shoulder-pitch servo is ignored). Also fills board_serial and arm_side.
void fillArmState(alfie_msgs__msg__ArmState *st);
