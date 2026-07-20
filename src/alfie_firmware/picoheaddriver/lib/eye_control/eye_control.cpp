#include "eye_control.h"

extern DriverBoard b;

// Managed by Core 1 (ros_interface.cpp).
extern RosAgentState_t agent_state;
extern bool ros_entities_created;

static const uint8_t EYE_PWM_PIN[NUM_EYES] = { EYE_A_PWM_PIN, EYE_B_PWM_PIN };

// ---------------------------------------------------------------------------
// Eye LEDs (TB6612FNG)
// ---------------------------------------------------------------------------

void initEyes()
{
    pinMode(EYE_STBY_PIN, OUTPUT);
    pinMode(EYE_A_IN1_PIN, OUTPUT);
    pinMode(EYE_A_IN2_PIN, OUTPUT);
    pinMode(EYE_B_IN1_PIN, OUTPUT);
    pinMode(EYE_B_IN2_PIN, OUTPUT);
    pinMode(EYE_A_PWM_PIN, OUTPUT);
    pinMode(EYE_B_PWM_PIN, OUTPUT);

    // Fixed drive polarity per channel (LED brightness via PWM only).
    digitalWrite(EYE_A_IN1_PIN, HIGH);
    digitalWrite(EYE_A_IN2_PIN, LOW);
    digitalWrite(EYE_B_IN1_PIN, HIGH);
    digitalWrite(EYE_B_IN2_PIN, LOW);

    // Enable the driver.
    digitalWrite(EYE_STBY_PIN, HIGH);

    // 100 Hz, 12-bit PWM range.
    analogWriteFreq(EYE_PWM_FREQ_HZ);
    analogWriteRange(EYE_PWM_MAX);

    eyesOff();
}

void setEye(uint8_t idx, uint16_t duty)
{
    if (idx >= NUM_EYES) {
        return;
    }
    analogWrite(EYE_PWM_PIN[idx], duty);
    b.eye_state[idx] = duty;
}

void applyEyes()
{
    for (uint8_t i = 0; i < NUM_EYES; i++) {
        setEye(i, b.eye_pwm[i]);
    }
}

void eyesOff()
{
    for (uint8_t i = 0; i < NUM_EYES; i++) {
        analogWrite(EYE_PWM_PIN[i], 0);
        b.eye_pwm[i]   = 0;
        b.eye_state[i] = 0;
    }
}

// ---------------------------------------------------------------------------
// Status LED (onboard WS2812) — ported from picobackdriver updateRgbLED()
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
