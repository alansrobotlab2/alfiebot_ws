/**
 * @file heartbeat_sync.h
 * @brief Status-LED heartbeat phase-locked to the micro-ROS agent clock.
 *
 * Shared by every Alfie pico (pulled in via `lib_extra_dirs = ../shared_lib`)
 * so all five boards blink in unison.
 *
 * The heartbeat is an 8-step x 125 ms (1 s) pattern. The step is derived from
 * the clock rather than counted, so loop latency only delays an edge and never
 * accumulates. Once Core 1 has synced with the agent, the step is taken from
 * agent (Orin) epoch time, which every board shares.
 *
 * Core 1 (micro-ROS):  heartbeatSyncNow() right after entities are created,
 *                      heartbeatSyncTask() every tick while connected.
 * Core 0 (LED):        heartbeatStep() / heartbeatStepChanged().
 *
 * Syncing also makes rmw_uros_epoch_nanos() return real epoch time, so header
 * stamps on published state messages are no longer boot-relative.
 *
 * @author Alfie Bot Project
 */

#pragma once

#include <Arduino.h>

#define HEARTBEAT_STEP_MS        125     ///< one pattern step (8 steps = 1 s)
#define HEARTBEAT_STEPS          8
#define HEARTBEAT_PERIOD_MS      (HEARTBEAT_STEP_MS * HEARTBEAT_STEPS)
#define HEARTBEAT_RESYNC_MS      30000   ///< crystal drift is ~ms/min, so 30 s is plenty
#define HEARTBEAT_RETRY_MS       1000    ///< retry interval after a failed sync
#define HEARTBEAT_SYNC_TIMEOUT_MS 50     ///< blocks Core 1 for at most this long

/// Core 1: sync with the agent now and update the phase. Returns true on success.
bool heartbeatSyncNow(void);

/// Core 1: re-sync every HEARTBEAT_RESYNC_MS (HEARTBEAT_RETRY_MS after a failure).
void heartbeatSyncTask(void);

/// True once at least one sync has succeeded since boot.
bool heartbeatSynced(void);

/// Current pattern step, 0..HEARTBEAT_STEPS-1, aligned across boards once synced.
uint8_t heartbeatStep(void);

/// Core 0: true when the step differs from the one seen on the previous call
/// (per call site via the caller-owned `last_step`). Use to rate-limit LED writes.
bool heartbeatStepChanged(uint8_t &last_step, uint8_t &step);
