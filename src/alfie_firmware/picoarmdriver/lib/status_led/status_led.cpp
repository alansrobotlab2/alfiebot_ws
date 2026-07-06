#include "status_led.h"

extern DriverBoard b;

// Managed by Core 1 (ros_interface.cpp).
extern RosAgentState_t agent_state;
extern bool ros_entities_created;

// ---------------------------------------------------------------------------
// Status LED (onboard WS2812) — ported from picoheaddriver updateStatusLED()
// ---------------------------------------------------------------------------

void updateStatusLED()
{
    static uint32_t last_update_time = 0;
    static uint8_t  led_step = 0;
    uint32_t now = millis();

    if (now - last_update_time < LED_BLINK_PERIOD_MS) {
        return;
    }
    last_update_time = now;

    // Heartbeat pattern (ROS connected & idle): 1,0,1,0,0,0,0,0
    const uint8_t heartbeat_pattern[] = {1, 0, 1, 0, 0, 0, 0, 0};

    // Board could not identify itself as left/right: purple heartbeat, and the
    // arm is kept limp (see applyArmCmd). This overrides the agent state so the
    // condition is obvious on the bench.
    if (b.arm_side == ARM_SIDE_UNKNOWN) {
        b.statusLED.setColor(heartbeat_pattern[led_step] ? 80 : 0, 0,
                             heartbeat_pattern[led_step] ? 80 : 0);   // purple
        led_step = (led_step + 1) % 8;
        return;
    }

    switch (agent_state) {
        case WAITING_AGENT:
            b.statusLED.setColor(0, 0, 50);        // dim blue
            break;
        case AGENT_AVAILABLE:
            b.statusLED.setColor(100, 80, 0);      // yellow
            break;
        case AGENT_CONNECTED:
            if (!ros_entities_created) {
                b.statusLED.setColor(100, 80, 0);  // yellow
            } else if (heartbeat_pattern[led_step]) {
                b.statusLED.setColor(0, 100, 0);   // green ON
            } else {
                b.statusLED.setColor(0, 0, 0);     // OFF
            }
            break;
        case AGENT_DISCONNECTED:
            b.statusLED.setColor(100, 30, 0);      // orange
            break;
        default:
            b.statusLED.setColor(20, 20, 20);      // dim white
            break;
    }

    led_step = (led_step + 1) % 8;
}
