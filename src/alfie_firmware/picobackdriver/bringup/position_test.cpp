/**
 * @file main.cpp
 * @brief TEMPORARY bring-up test 3: position moves + GEAR_RATIO calibration.
 *
 * The real application is parked in main.cpp.orig - restore it with
 *     git mv src/main.cpp.orig src/main.cpp
 *
 * WHAT THIS ANSWERS
 *   1. Does the actuator reach a commanded height, and how close does it get?
 *   2. How fast does it actually travel, in counts/s and in indicated mm/s?
 *   3. What is the REAL counts-per-metre scale? (the 'c' command)
 *
 * WHY THAT THIRD ONE IS THE POINT
 *   GEAR_RATIO is 209.7 and has never been measured. config.h admits an
 *   unexplained 2.33x gap against the datasheet geometry, and every derived
 *   number in this project - travel, speed, the duty ceiling's thermal
 *   arithmetic, ACTUATOR_MAX_POSITION - is scaled by it. Until a rule is put on
 *   the mechanism, "mm" here means "indicated mm", not millimetres.
 *
 *   Procedure:  0            zero the datum at a marked start point
 *               g 100        drive to +100 indicated mm
 *               <measure the REAL displacement with a rule>
 *               c 43.5       enter what you measured; it prints the corrected
 *                            GEAR_RATIO and the true speed of that move
 *
 * SAFETY - READ THIS, IT DIFFERS FROM THE BURST TEST
 *   The previous test could only ever run for BURST_MS. This one runs until the
 *   target is reached, so it needs real aborts, and it has them:
 *
 *     - STALL      no encoder progress for STALL_MS while driving
 *     - OVERSPEED  above RUNAWAY_VELOCITY_MPS
 *     - REVERSED   moving away from the target after a grace period
 *     - LIMIT      bottom switch, on descending moves only
 *     - TIMEOUT    generous, sized from the distance
 *     - 'x' + Enter at any time
 *
 *   *** THERE IS NO TOP LIMIT SWITCH. *** Only the bottom is switched. On an
 *   upward move nothing but the stall detector stops the actuator at the top of
 *   travel, and by then it is pushing at full duty. Command upward moves in
 *   small steps until the true scale is known - a commanded 100 indicated mm may
 *   be 233 real mm.
 *
 *   Still open loop in the sense that matters: fixed duty, no PID. Direction is
 *   decided once per move from the sign of the position error, so a sign fault
 *   cannot become positive feedback the way a velocity loop can.
 *
 * BEFORE RUNNING: confirm IPROPI/CS is strapped to GND and VREF is above GND.
 * That disables the ITRIP chopper by design (datasheet 7.3.3.2). Overcurrent and
 * thermal shutdown are unaffected and still protect the part.
 *
 * COMMANDS ARE LINE-BASED NOW - type the command and press Enter. The burst test
 * was single-keypress; numeric arguments make that impossible, and requiring
 * Enter also means a stray keystroke cannot start a move.
 */

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include "../include/config.h"

// -----------------------------------------------------------------------------
// Test parameters
// -----------------------------------------------------------------------------
static const uint32_t BURST_MS          = 300;  ///< Manual jog burst length
static const uint32_t REVERSAL_GRACE_MS = 250;  ///< Ignore direction this long (break-away)
static const int      DUTY_MIN          = 30;
static const int      DUTY_MAX          = VELOCITY_PID_OUTPUT_LIMIT;
static const float    REVERSAL_MPS      = RUNAWAY_REVERSE_MPS;

/// Positional deadband. Below this the move is "arrived" and stops.
static const float    ARRIVE_MM         = 0.5f;

/// Largest single commanded move, in INDICATED mm. Deliberately modest: with the
/// scale unverified a commanded 500 could be 1165 real mm.
static const float    MAX_MOVE_MM       = 500.0f;

/// Stall: fewer than this many counts of progress within STALL_MS aborts.
static const uint32_t STALL_MS          = 600;
static const int32_t  STALL_MIN_COUNTS  = 20;

/// nFAULT low-fraction (%) that counts as a real fault rather than pull-up noise.
static const int      NFAULT_SUSTAINED_PCT = 8;

static int  duty = 120;     ///< Above the measured reliable-motion floor of 100

/// Which logic level means "bottom limit reached". CONFIRMED HIGH on hardware:
/// the switch is normally closed, so it reads LOW during travel and HIGH only
/// when actuated at the bottom. 'L' still toggles it for re-checking.
static int  limit_active_level = LIMIT_SWITCH_ACTIVE_LEVEL;

// -----------------------------------------------------------------------------
// Encoder (same decode as the real firmware)
// -----------------------------------------------------------------------------
volatile int32_t enc_count = 0;

/**
 * Quadrature transition table, indexed by (prev_state << 2) | current_state,
 * where state is (A << 1) | B. +1 forward, -1 reverse, 0 for no-change AND for
 * the illegal both-pins-changed case.
 *
 * REPLACES a decoder that counted +/-1 on EVERY interrupt using the pins'
 * current levels, with no memory of the previous state. That lost position
 * cumulatively: 20 mm high after one round trip, 32 mm after two, growing every
 * cycle, with no mechanical slip. Edge dither injected counts with a systematic
 * sign instead of cancelling. See the long note in motor_control.cpp.
 */
static const int8_t QUAD_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static volatile uint8_t quad_prev = 0xFF;

static inline void processEdge() {
    const bool a = digitalRead(MOTOR_ENCODER_A);
    const bool b = digitalRead(MOTOR_ENCODER_B);
    const uint8_t state = (uint8_t)((a ? 2 : 0) | (b ? 1 : 0));

    if (quad_prev == 0xFF) { quad_prev = state; return; }

    enc_count += QUAD_TABLE[(quad_prev << 2) | state];
    quad_prev  = state;
}
void isrA() { processEdge(); }
void isrB() { processEdge(); }

static int32_t readCount() {
    noInterrupts();
    int32_t c = enc_count;
    interrupts();
    return c;
}

static float countsToMm(int32_t c) {
    return 1000.0f * (float)c / (float)ENCODER_COUNTS_PER_METER;
}
static int32_t mmToCounts(float mm) {
    return (int32_t)(mm * (float)ENCODER_COUNTS_PER_METER / 1000.0f);
}

static bool limitTriggered() {
    return digitalRead(LIMIT_SWITCH_PIN) == limit_active_level;
}

// -----------------------------------------------------------------------------
// Calibration bookkeeping
// -----------------------------------------------------------------------------
static int32_t  datum_count      = 0;   ///< Set by '0'; 'c' measures from here
static int32_t  last_move_counts = 0;   ///< Signed counts of the last completed move
static uint32_t last_move_ms     = 0;   ///< Duration of that move, driving only

static void brake() {
    analogWrite(MOTOR_EN_PIN, 0);       // EN=0 with nSLEEP high = brake
}

/// Drain any pending serial so a queued command cannot fire straight after a move.
static void flushInput() {
    while (Serial.available()) Serial.read();
}

// -----------------------------------------------------------------------------
/**
 * @brief Drive to an absolute encoder target at fixed duty, fully monitored.
 *
 * Direction is fixed at entry from the sign of the error. No PID: the duty is
 * constant for the whole move and the loop only decides when to stop. That keeps
 * the failure modes the same as the burst test - a wrong sign gives a runaway in
 * one direction, caught by REVERSED/OVERSPEED, not an oscillation.
 */
static void moveToCount(int32_t target) {
    const int32_t start   = readCount();
    const int32_t delta   = target - start;

    if (labs((long)delta) < labs((long)mmToCounts(ARRIVE_MM))) {
        Serial.println("already there");
        return;
    }

    const bool up = (delta > 0);

    if (!up && limitTriggered()) {
        Serial.println("REFUSED: bottom limit already reads triggered.");
        Serial.println("  If that is wrong, press 'L' to flip the assumed polarity");
        Serial.println("  and check the raw level with 's' first.");
        return;
    }

    // Generous: real problems are caught by the stall detector long before this.
    const float dist_mm   = fabsf(countsToMm(delta));
    const uint32_t timeout_ms = 3000 + (uint32_t)(dist_mm / 1.5f * 1000.0f);

    const int32_t q20 = start + (int32_t)(delta * 0.2f);   // steady-speed window
    const int32_t q80 = start + (int32_t)(delta * 0.8f);
    uint32_t t20 = 0, t80 = 0;
    int32_t  c20 = 0, c80 = 0;

    uint32_t nfault_lows = 0, samples = 0;
    float    peak_mps = 0.0f;
    const char *abort_reason = nullptr;

    int32_t  last_count    = start;
    uint32_t last_t        = millis();
    int32_t  progress_mark = start;
    uint32_t progress_t    = last_t;

    const uint32_t t_start = millis();

    digitalWrite(MOTOR_PH_PIN, up ? MOTOR_PH_UP : MOTOR_PH_DOWN);
    analogWrite(MOTOR_EN_PIN, duty);

    for (;;) {
        delay(5);

        const uint32_t now = millis();
        const int32_t  c   = readCount();

        samples++;
        if (digitalRead(MOTOR_NFAULT_PIN) == LOW) nfault_lows++;

        // Arrived? Sign-aware so it cannot run past the target.
        if (( up && c >= target) || (!up && c <= target)) break;

        if (!up && limitTriggered()) { abort_reason = "LIMIT";    break; }
        if ((now - t_start) > timeout_ms) { abort_reason = "TIMEOUT"; break; }

        if (Serial.available()) {
            const int ch = Serial.peek();
            if (ch == 'x' || ch == 'X') { abort_reason = "OPERATOR"; break; }
        }

        // Steady-speed window marks
        if (t20 == 0 && (( up && c >= q20) || (!up && c <= q20))) { t20 = now; c20 = c; }
        if (t80 == 0 && (( up && c >= q80) || (!up && c <= q80))) { t80 = now; c80 = c; }

        const uint32_t dt_ms = now - last_t;
        if (dt_ms >= 20) {
            const float v = ((float)(c - last_count) / (float)ENCODER_COUNTS_PER_METER)
                            / ((float)dt_ms / 1000.0f);
            last_count = c;
            last_t     = now;

            if (fabsf(v) > peak_mps) peak_mps = fabsf(v);
            if (fabsf(v) > RUNAWAY_VELOCITY_MPS) { abort_reason = "OVERSPEED"; break; }

            if ((now - t_start) > REVERSAL_GRACE_MS && fabsf(v) > REVERSAL_MPS) {
                if ((v > 0) != up) { abort_reason = "REVERSED"; break; }
            }

            // Stall: no meaningful progress in STALL_MS while driving.
            if (labs((long)(c - progress_mark)) >= STALL_MIN_COUNTS) {
                progress_mark = c;
                progress_t    = now;
            } else if ((now - progress_t) > STALL_MS) {
                abort_reason = "STALL";
                break;
            }
        }
    }

    const uint32_t drive_ms = millis() - t_start;
    brake();
    delay(200);                          // settle before measuring

    const int32_t moved = readCount() - start;
    const int32_t err   = target - readCount();

    last_move_counts = moved;
    last_move_ms     = drive_ms;

    const float secs    = drive_ms / 1000.0f;
    const float avg_cps = secs > 0 ? (float)moved / secs : 0.0f;

    Serial.printf("\n%s duty=%d -> %+ld counts (%+.2f indicated mm) in %.2f s\n",
                  up ? "UP  " : "DOWN", duty,
                  (long)moved, (double)countsToMm(moved), (double)secs);
    Serial.printf("  avg    %8.1f counts/s   %6.2f indicated mm/s\n",
                  (double)avg_cps, (double)countsToMm((int32_t)avg_cps));

    if (t20 && t80 && t80 > t20) {
        const float ss = (float)(c80 - c20) / ((t80 - t20) / 1000.0f);
        Serial.printf("  steady %8.1f counts/s   %6.2f indicated mm/s   <- use THIS for speed\n",
                      (double)ss, (double)countsToMm((int32_t)ss));
    } else {
        Serial.println("  steady    (move too short to measure a steady speed)");
    }

    Serial.printf("  peak   %.3f indicated m/s | residual error %+ld counts (%+.2f mm)\n",
                  (double)peak_mps, (long)err, (double)countsToMm(err));
    Serial.printf("  since datum: %+ld counts (%+.2f indicated mm)   nFAULT %lu/%lu\n",
                  (long)(readCount() - datum_count),
                  (double)countsToMm(readCount() - datum_count),
                  (unsigned long)nfault_lows, (unsigned long)samples);

    if (abort_reason) {
        Serial.printf("  *** ABORTED: %s ***\n", abort_reason);
        if (!strcmp(abort_reason, "STALL")) {
            Serial.println("  No progress while driving. Top of travel, an obstruction,");
            Serial.println("  or duty below break-away (measured floor is 100).");
        } else if (!strcmp(abort_reason, "REVERSED")) {
            Serial.println("  Moved away from the target. Check PH polarity and PMODE.");
        } else if (!strcmp(abort_reason, "LIMIT")) {
            Serial.println("  Bottom limit asserted. This is the expected way to end a");
            Serial.println("  descent to the stop - zero the datum here with '0'.");
        }
    }

    if (nfault_lows * 100 >= samples * NFAULT_SUSTAINED_PCT) {
        Serial.println("  nFAULT sustained - with chopping disabled this is a REAL fault:");
        Serial.println("  overcurrent, thermal shutdown or UVLO. Check VM and wiring.");
    }
    flushInput();
}

static void moveToMm(float mm) {
    if (fabsf(mm) > MAX_MOVE_MM) {
        Serial.printf("refused: |%.1f| exceeds the %.0f indicated mm per-move cap\n",
                      (double)mm, (double)MAX_MOVE_MM);
        return;
    }
    moveToCount(datum_count + mmToCounts(mm));
}

static void moveRelMm(float mm) {
    if (fabsf(mm) > MAX_MOVE_MM) {
        Serial.printf("refused: |%.1f| exceeds the %.0f indicated mm per-move cap\n",
                      (double)mm, (double)MAX_MOVE_MM);
        return;
    }
    moveToCount(readCount() + mmToCounts(mm));
}

// -----------------------------------------------------------------------------
/**
 * @brief Turn a rule measurement into a corrected GEAR_RATIO.
 *
 * The scale enters as counts-per-metre, which is
 *     ENCODER_PPR x 4 x GEAR_RATIO / METERS_PER_REVOLUTION
 * so a pure scale error lands entirely on GEAR_RATIO regardless of whether the
 * real culprit is the PPR, the gearbox ratio or the pulley. That is why this
 * prints one corrected constant rather than trying to attribute the error.
 */
static void calibrate(float actual_mm) {
    const int32_t counts = readCount() - datum_count;
    const float indicated_mm = countsToMm(counts);

    if (fabsf(actual_mm) < 1.0f) {
        Serial.println("need a real measured distance, e.g.  c 43.5");
        return;
    }
    if (labs((long)counts) < 100) {
        Serial.println("barely moved since the datum - zero with '0', make a move, then measure");
        return;
    }
    // Compare magnitudes: the rule has no sign, the encoder does.
    const float ratio    = fabsf(indicated_mm) / fabsf(actual_mm);
    const float new_gear = GEAR_RATIO * ratio;
    const float new_cpm  = (float)ENCODER_COUNTS_PER_METER * ratio;

    Serial.println();
    Serial.println("---- GEAR_RATIO CALIBRATION ----------------------------------");
    Serial.printf("  counts since datum   %+ld\n", (long)counts);
    Serial.printf("  indicated            %.2f mm\n", (double)indicated_mm);
    Serial.printf("  actual (measured)    %.2f mm\n", (double)actual_mm);
    Serial.printf("  scale error          %.3fx  (indicated / actual)\n", (double)ratio);
    Serial.println();
    Serial.printf("  GEAR_RATIO           %.1f  ->  %.1f\n", (double)GEAR_RATIO, (double)new_gear);
    Serial.printf("  counts per metre     %.0f  ->  %.0f\n",
                  (double)ENCODER_COUNTS_PER_METER, (double)new_cpm);
    Serial.printf("  counts per mm        %.1f  ->  %.1f\n",
                  (double)ENCODER_COUNTS_PER_METER / 1000.0, (double)new_cpm / 1000.0);
    Serial.println();

    if (last_move_ms > 0 && last_move_counts != 0) {
        const float true_mms = (float)last_move_counts / (new_cpm / 1000.0f)
                               / (last_move_ms / 1000.0f);
        Serial.printf("  TRUE speed of the last move: %.1f mm/s\n", (double)fabsf(true_mms));
        if (fabsf(true_mms) > 0.1f) {
            Serial.printf("  -> 400 mm of REAL travel would take %.1f s\n",
                          (double)(400.0f / fabsf(true_mms)));
        }
        const float no_load_mms = MOTOR_FREE_SPEED_RPM / 60.0f * 40.0f;
        Serial.printf("  -> %.0f%% of the %.0f mm/s no-load ceiling for this motor\n",
                      (double)(fabsf(true_mms) / no_load_mms * 100.0f), (double)no_load_mms);
        Serial.println("     (a healthy drive sits near 50%; far below means overloaded)");
    }
    Serial.println();
    Serial.println("  Put the new GEAR_RATIO in config.h, then RE-DERIVE:");
    Serial.println("    ACTUATOR_MAX_POSITION   - real stroke, not indicated");
    Serial.println("    MAX_ACTUATOR_VELOCITY   - rescales with the same factor");
    Serial.println("    VELOCITY_PID_OUTPUT_LIMIT - its thermal case assumed 400 real mm");
    Serial.println("--------------------------------------------------------------");
    Serial.println();
}

// -----------------------------------------------------------------------------
static void jog(bool up) {
    const int32_t start = readCount();
    const uint32_t t0 = millis();

    if (!up && limitTriggered()) {
        Serial.println("REFUSED: bottom limit reads triggered ('L' flips polarity)");
        return;
    }

    digitalWrite(MOTOR_PH_PIN, up ? MOTOR_PH_UP : MOTOR_PH_DOWN);
    analogWrite(MOTOR_EN_PIN, duty);
    while ((millis() - t0) < BURST_MS) {
        delay(5);
        if (!up && limitTriggered()) break;
    }
    brake();
    delay(150);

    const int32_t moved = readCount() - start;
    Serial.printf("%s duty=%d -> %+ld counts (%+.2f indicated mm)\n",
                  up ? "UP  " : "DOWN", duty, (long)moved, (double)countsToMm(moved));
}

static void nfaultCheck() {
    brake();
    delay(50);
    uint32_t lows = 0;
    const uint32_t N = 2000;
    for (uint32_t i = 0; i < N; i++) {
        if (digitalRead(MOTOR_NFAULT_PIN) == LOW) lows++;
        delayMicroseconds(200);
    }
    Serial.printf("\nnFAULT idle check: %lu/%lu low over 0.4 s, bridge braked\n",
                  (unsigned long)lows, (unsigned long)N);
    if (lows == 0)            Serial.println("  Clean.");
    else if (lows >= N / 2)   Serial.println("  Held low with no current = genuine device fault.");
    else                      Serial.println("  Intermittent with no current = noise. Fit 10k to 3.3 V.");
}

static void monitor() {
    brake();
    Serial.println("\nMonitoring 8 s - MOVE THE ACTUATOR BY HAND.");
    const uint32_t t0 = millis();
    while ((millis() - t0) < 8000) {
        const int32_t c = readCount();
        Serial.printf("  count=%8ld  pos=%8.2f mm | A=%d B=%d | limit raw=%s %s\n",
                      (long)c, (double)countsToMm(c),
                      digitalRead(MOTOR_ENCODER_A), digitalRead(MOTOR_ENCODER_B),
                      digitalRead(LIMIT_SWITCH_PIN) == HIGH ? "HIGH" : "LOW ",
                      limitTriggered() ? "<TRIGGERED>" : "");
        delay(250);
    }
    Serial.println("  ...done");
}

static void status() {
    const int32_t c = readCount();
    Serial.println();
    Serial.printf("  count      %ld  (%.2f indicated mm)\n", (long)c, (double)countsToMm(c));
    Serial.printf("  datum      %ld  -> %+.2f indicated mm since datum\n",
                  (long)datum_count, (double)countsToMm(c - datum_count));
    Serial.printf("  duty       %d   (floor 100 measured, ceiling %d)\n", duty, DUTY_MAX);
    Serial.printf("  limit      raw=%s, treating %s as TRIGGERED -> %s\n",
                  digitalRead(LIMIT_SWITCH_PIN) == HIGH ? "HIGH" : "LOW",
                  limit_active_level == HIGH ? "HIGH" : "LOW",
                  limitTriggered() ? "TRIGGERED" : "clear");
    Serial.printf("  scale      %.0f counts/m  (GEAR_RATIO %.1f, UNVERIFIED)\n",
                  (double)ENCODER_COUNTS_PER_METER, (double)GEAR_RATIO);
    Serial.println();
}

static void help() {
    Serial.println();
    Serial.println("  Type a command then press Enter.");
    Serial.println("    g <mm>   go to absolute height, indicated mm from datum");
    Serial.println("    m <mm>   move relative, signed (m -20 goes down 20)");
    Serial.println("    c <mm>   calibrate: enter the REAL measured travel since datum");
    Serial.println("    0        set the datum here (zero)");
    Serial.println("    p <n>    set duty (30-200)");
    Serial.println("    u / d    one 300 ms burst up / down");
    Serial.println("    s        status          w  hand-move monitor");
    Serial.println("    L        flip assumed limit-switch polarity");
    Serial.println("    n        nFAULT idle check");
    Serial.println("    x        brake / abort a move in progress");
    Serial.println("    ?        this help");
    Serial.println();
    Serial.println("  Calibration:  0  ->  g 100  ->  measure with a rule  ->  c <actual>");
    Serial.println();
}

// -----------------------------------------------------------------------------
static char    cmdbuf[32];
static uint8_t cmdlen = 0;

static void handleLine(char *s) {
    while (*s == ' ' || *s == '\t') s++;
    if (!*s) return;

    const char cmd = *s;
    const float arg = (float)atof(s + 1);        // 0.0 when no argument given

    switch (cmd) {
        case 'g': case 'G': moveToMm(arg);  break;
        case 'm': case 'M': moveRelMm(arg); break;
        case 'c': case 'C': calibrate(arg); break;
        case 'p': case 'P':
            duty = (int)constrain((int)arg, DUTY_MIN, DUTY_MAX);
            Serial.printf("duty=%d\n", duty);
            break;
        case '0':
            datum_count = readCount();
            Serial.printf("datum set at count %ld\n", (long)datum_count);
            break;
        case 'u': case 'U': jog(true);  break;
        case 'd': case 'D': jog(false); break;
        case 's': case 'S': status();   break;
        case 'w': case 'W': monitor();  break;
        case 'n': case 'N': nfaultCheck(); break;
        case 'L': case 'l':
            limit_active_level = (limit_active_level == LOW) ? HIGH : LOW;
            Serial.printf("limit TRIGGERED level is now %s\n",
                          limit_active_level == HIGH ? "HIGH" : "LOW");
            break;
        case 'x': case 'X': brake(); Serial.println("braked"); break;
        case '?': help(); break;
        default:  Serial.println("? - press ? for help"); break;
    }
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 3000) delay(10);

    // PMODE before nSLEEP: the rising edge on nSLEEP is what latches it.
    pinMode(MOTOR_PMODE_PIN,  OUTPUT); digitalWrite(MOTOR_PMODE_PIN,  LOW);
    pinMode(MOTOR_EN_PIN,     OUTPUT); digitalWrite(MOTOR_EN_PIN,     LOW);
    pinMode(MOTOR_PH_PIN,     OUTPUT); digitalWrite(MOTOR_PH_PIN,     MOTOR_PH_UP);
    pinMode(MOTOR_NFAULT_PIN, INPUT_PULLUP);
    pinMode(MOTOR_NSLEEP_PIN, OUTPUT); digitalWrite(MOTOR_NSLEEP_PIN, HIGH);
    delay(2);                                  // tWAKE is 1 ms

    analogWriteFreq(PWM_FREQUENCY);
    analogWriteRange(PWM_MAX_DUTY);

    pinMode(MOTOR_ENCODER_A,  INPUT_PULLUP);
    pinMode(MOTOR_ENCODER_B,  INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_A), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_B), isrB, CHANGE);

    datum_count = readCount();

    Serial.println();
    Serial.println("=====================================================");
    Serial.println(" POSITION MOVE + GEAR RATIO CALIBRATION");
    Serial.println("=====================================================");
    Serial.println(" Fixed duty, no PID. Moves abort on stall, overspeed,");
    Serial.println(" reversal, bottom limit, timeout, or 'x'.");
    Serial.println();
    Serial.println(" *** NO TOP LIMIT SWITCH - only the bottom is switched. ***");
    Serial.println(" On upward moves only the stall detector stops it at the");
    Serial.println(" top. Step up in small increments until the scale is known.");
    Serial.println();
    Serial.println(" 'mm' means INDICATED mm. GEAR_RATIO is unverified and may");
    Serial.println(" be wrong by ~2.3x - that is what 'c' exists to settle.");
    help();
}

void loop() {
    while (Serial.available()) {
        const char ch = (char)Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (cmdlen) {
                cmdbuf[cmdlen] = '\0';
                handleLine(cmdbuf);
                cmdlen = 0;
            }
        } else if (cmdlen < sizeof(cmdbuf) - 1) {
            cmdbuf[cmdlen++] = ch;
        }
    }
}
