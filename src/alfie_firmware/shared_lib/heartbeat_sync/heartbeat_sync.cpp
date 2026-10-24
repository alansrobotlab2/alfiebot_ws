#include "heartbeat_sync.h"
#include <rmw_microros/rmw_microros.h>

// (agent_epoch_ms - millis()) mod HEARTBEAT_PERIOD_MS. Written by Core 1, read by Core 0; a
// 32-bit aligned store is atomic on the RP2040 (a 64-bit offset would tear).
static volatile uint32_t phase_ms = 0;
static volatile bool     synced   = false;

static uint32_t last_attempt_ms = 0;
static bool     last_ok         = false;

bool heartbeatSyncNow(void)
{
    last_attempt_ms = millis();
    last_ok = (RMW_RET_OK == rmw_uros_sync_session(HEARTBEAT_SYNC_TIMEOUT_MS)) &&
              rmw_uros_epoch_synchronized();
    if (!last_ok) {
        return false;
    }

    int64_t offset = rmw_uros_epoch_millis() - (int64_t)millis();
    int32_t phase  = (int32_t)(offset % HEARTBEAT_PERIOD_MS);
    if (phase < 0) {
        phase += HEARTBEAT_PERIOD_MS;
    }
    phase_ms = (uint32_t)phase;
    synced   = true;
    return true;
}

void heartbeatSyncTask(void)
{
    uint32_t interval = last_ok ? HEARTBEAT_RESYNC_MS : HEARTBEAT_RETRY_MS;
    if (millis() - last_attempt_ms >= interval) {
        heartbeatSyncNow();
    }
}

bool heartbeatSynced(void)
{
    return synced;
}

uint8_t heartbeatStep(void)
{
    // millis() wrapping (every ~49.7 days) shifts the phase until the next
    // re-sync, at most HEARTBEAT_RESYNC_MS later.
    return (uint8_t)(((millis() + phase_ms) % HEARTBEAT_PERIOD_MS) / HEARTBEAT_STEP_MS);
}

bool heartbeatStepChanged(uint8_t &last_step, uint8_t &step)
{
    step = heartbeatStep();
    if (step == last_step) {
        return false;
    }
    last_step = step;
    return true;
}
