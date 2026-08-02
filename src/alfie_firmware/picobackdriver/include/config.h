/**
 * @file config.h
 * @brief Configuration file for RP2040 back Drive Robot Base
 * 
 * This file contains all configuration defines for a complete back drive
 * robot implementation including pin assignments, communication settings,
 * physical dimensions, and operational parameters.
 * 
 * @author Alfie Bot Project
 * @date 2025-10-24
 */

#pragma once

#define NAMESPACE "alfie/low"

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

/**
 * @brief Serial Communication Configuration
 */
#define SERIAL_BAUD_RATE        1000000  ///< Serial port baud rate for ROS2 communication
#define SERIAL_TIMEOUT_MS       1000    ///< Serial communication timeout

/**
 * @brief Status LED Configuration
 * Built-in rgb LED for status indication
 */

#define WS2812B_PIN             16      ///< WS2812B RGB LED data pin
#define LED_BLINK_PERIOD_MS     125          ///< LED blink timing unit (125ms = 8 steps per second)


// ==========================================================================
// PIN MAP (gen2: DRV8876 driver)
// ==========================================================================
/**
 * MOTOR DRIVER (DRV8876)
 *   GP28  EN          PWM                       (was TB6612 PWMA)
 *   GP27  PH          direction                 (was TB6612 AIN1)
 *   GP15  nSLEEP      HIGH = active             (was TB6612 STBY)
 *   GP26  PMODE       held LOW for PH/EN mode
 *   GP7   nFAULT      open-drain in, pulled up  (was TB6612 AIN2)
 *   --    CS          DISCONNECTED - see below
 *   --    VREF        tie the header to 3.3 V - see MOTOR DRIVER section
 *
 * ACTUATOR FEEDBACK
 *   GP2   ENC_A       quadrature A  (A/B swapped after bench test - see below)
 *   GP3   ENC_B       quadrature B
 *   GP1   LIMIT_SW    lower limit switch
 *
 * IMU (BNO085 on I2C0)
 *   GP12  I2C0 SDA    [RP2040 I2C0 SDA is only GP0/4/8/12/16/20]
 *   GP13  I2C0 SCL    [RP2040 I2C0 SCL is only GP1/5/9/13/17/21]
 *   GP11  IMU_RESET   active low
 *
 * STATUS
 *   GP16  WS2812B     status LED
 *
 * Free: GP0, GP4, GP5, GP6, GP8, GP9, GP10, GP14, GP17-22. A second I2C bus
 * would have to use GP18/GP19 - I2C1's other natural pairs (GP2/GP3, GP6/GP7,
 * GP14/GP15) are now taken or split.
 *
 * IMODE and VREF remain resistor-strapped on the carrier. PMODE is the one that
 * moved to a GPIO; it is latched when nSLEEP rises. See MOTOR DRIVER section.
 *
 * *** CS IS TIED TO GND - INTERNAL CURRENT REGULATION IS DISABLED ***
 *
 * CS is the DRV8876's IPROPI output: an ANALOG current-sense signal, not a chip
 * select - the part has no serial interface of any kind. Per datasheet 7.3.3.2,
 * grounding IPROPI disables the ITRIP current chopper outright. That is
 * deliberate here - see the MOTOR DRIVER section for the reasoning.
 *
 * This costs nothing in device protection. Overcurrent (3.5-5.5 A, 2 ms
 * auto-retry) and thermal shutdown are separate silicon and stay active with
 * IPROPI grounded. What is given up is the ability to chop at a chosen current
 * below those thresholds - which the motors in use cannot reach anyway.
 *
 * To measure motor current later, CS would come off GND onto an ADC pin through
 * a sense resistor sized for the range. All three ADC pins (GP26/27/28) carry
 * digital motor signals, so PMODE would have to move off GP26 first (GP5, GP6,
 * GP8 are free). Note that doing so RE-ENABLES chopping: any resistor other than
 * a short lets V_IPROPI reach VREF. ITRIP would then have to be set deliberately
 * above the stall current rather than left to fall where it may.
 */

// ==========================================================================
// LIMIT SWITCH PIN ASSIGNMENT
// ==========================================================================
// GP1. Note this is UART0 RX in the RP2040's default mux; harmless here because
// the board talks to the agent over USB CDC, not UART0, but it would collide if
// anyone ever enables a hardware serial console.
#define LIMIT_SWITCH_PIN    1       ///< Minimum position limit switch pin

/**
 * @brief Limit switch active level - HIGH, because the switch is NORMALLY CLOSED
 *
 * CORRECTED ON HARDWARE. Every read site previously hard-coded `== LOW` and was
 * inverted. With the pin at INPUT_PULLUP and an NC switch:
 *
 *   not at the bottom -> contacts CLOSED to GND -> reads LOW
 *   at the bottom     -> contacts OPEN, pull-up wins -> reads HIGH
 *
 * The bring-up logs show exactly that: LOW for an entire 19,000-count sweep and
 * HIGH only at the lowest point reached. Reading it as active-low means the
 * firmware believes the actuator is sitting on the bottom stop essentially all
 * the time.
 *
 * WHAT THAT BROKE: calibrateActuator() tests `if (!limit_switch_triggered)`
 * before driving down. Inverted, that test failed immediately, so calibration
 * skipped the descent entirely and zeroed wherever the actuator happened to be.
 * A silent false datum - no error, no timeout, just a wrong origin for every
 * subsequent move.
 *
 * NC is the correct choice for a safety limit and worth keeping: a broken wire
 * or unplugged connector reads as TRIGGERED (fail-safe) rather than as clear.
 * Do not "fix" this by rewiring to normally-open.
 */
#define LIMIT_SWITCH_ACTIVE_LEVEL   HIGH    ///< NC switch: HIGH = at the bottom stop


// ==========================================================================
// IMU CONFIGURATION (BNO085 on I2C0)
// ==========================================================================
/**
 * @brief BNO085 9-DOF IMU Configuration
 * Connected on I2C0: SDA=GP12, SCL=GP13, RESET=GP11.
 * On-chip sensor fusion provides a true quaternion (rotation vector) plus
 * calibrated gyro (rad/s) and accelerometer (m/s^2, gravity included).
 */
#define IMU_I2C_SDA_PIN         12      ///< BNO085 I2C0 SDA pin
#define IMU_I2C_SCL_PIN         13      ///< BNO085 I2C0 SCL pin
#define IMU_RESET_PIN           11      ///< BNO085 active-low reset line
#define IMU_I2C_ADDR            0x4B    ///< BNO085 I2C address (SA0/ADR high)
#define IMU_I2C_CLOCK_HZ        100000  ///< I2C bus clock (Standard Mode)
// All BNO085 reports run at 50 Hz. The INT line is not wired, so the driver polls
// the reports over the 100 kHz I2C bus from the Core 0 control loop; capping every
// report at 50 Hz keeps that polling load well within budget (4x50 Hz < the old
// 3x100 Hz) so the two fused-quaternion reports needed for compass decouple do not
// overrun Core 0 and halve the BackState publish rate. 50 Hz matches the BackState
// publish rate and stays fresh within COMPASS_DECOUPLE_MS.
#define IMU_REPORT_INTERVAL_MS  20      ///< On-chip report interval for all reports (50 Hz)

// Publish the compass-free game rotation vector instead of the mag-referenced
// rotation vector for this long after the last "power applied" signal (local
// actuator PWM or the neck_power heartbeat), then re-couple the compass.
#define COMPASS_DECOUPLE_MS     100     ///< Compass decouple hold time (ms)


// =============================================================================
// MOTOR DRIVER PIN ASSIGNMENTS
// =============================================================================

/**
 * @brief Motor Driver Pin Configuration
 * DRV8876 motor driver in PH/EN mode (gen2; replaces the TB6612FNG).
 *
 * Only three pins are driven. PWM goes on EN, direction is a static level on PH.
 * Per the DRV8876 PH/EN truth table:
 *   nSLEEP=0                 -> outputs Hi-Z (sleep)
 *   nSLEEP=1, EN=0           -> brake (low-side slow decay)
 *   nSLEEP=1, EN=1, PH=0     -> OUT1=L OUT2=H
 *   nSLEEP=1, EN=1, PH=1     -> OUT1=H OUT2=L
 * Note there is no coast state in PH/EN mode; EN=0 always brakes. To coast you
 * must drop nSLEEP. On a back-driveable 56:1 gearbox brake is what we want, so
 * this also fixes the old emergencyStop() which claimed to brake but set the
 * TB6612 to L/L = Stop/high-impedance (coast).
 *
 * Dead time is generated internally (300 ns), so a direction flip cannot cause
 * shoot-through. Logic is 3.3 V safe: VIH min is 1.5 V. All three inputs have
 * internal 100k pulldowns, so the bridge stays off if the Pico is unprogrammed.
 */

// Linear Actuator Motor - DRV8876 Control Pins
#define MOTOR_EN_PIN            28      ///< DRV8876 EN/IN1 - PWM (was TB6612 PWMA)
#define MOTOR_PH_PIN            27      ///< DRV8876 PH/IN2 - direction (was TB6612 AIN1)
#define MOTOR_NSLEEP_PIN        15      ///< DRV8876 nSLEEP - HIGH=active, LOW=sleep (was TB6612 STBY)
// PMODE driven from a GPIO instead of a solder bridge to GND. This is safe
// because the latch only happens on an nSLEEP rising edge, and nSLEEP has an
// internal pulldown that holds the driver asleep while the Pico's GPIOs are
// still Hi-Z during boot - so a floating PMODE cannot be sampled.
//
// The pulldown measures 160k on this part (datasheet publishes 100k TYP with no
// min/max; an unpowered ohmmeter reading goes through ESD structures and
// unbiased junctions, and on-die resistors have wide process tolerance). The
// margin holds either way: RP2040 Hi-Z input leakage is +/-1 uA, so 1 uA x 160k
// = 0.16 V against a VIL max of 0.8 V - about 5x margin.
//
// Two absolute requirements:
//   - initializePeripherals() must drive PMODE LOW before raising nSLEEP.
//   - Neither pin may ever be reconfigured to INPUT_PULLUP. On PMODE, Hi-Z on a
//     tri-level input selects independent half-bridge (inverts the down
//     direction, disables current regulation - see the strap notes below). On
//     nSLEEP, the RP2040's ~65k pull-up against this 160k pulldown divides to
//     2.35 V, above VIH, and the driver would wake on its own.
#define MOTOR_PMODE_PIN         26      ///< DRV8876 PMODE - held LOW for PH/EN mode (frees the solder bridge)
// CS/IPROPI is TIED TO GND - no MOTOR_CS_PIN define, GP8 is free. Grounding
// IPROPI disables internal current regulation (datasheet 7.3.3.2), which is
// deliberate. See the PIN MAP note and the MOTOR DRIVER section.
// Logic levels need no thought here: the DRV8876 has NO logic supply pin (the
// 16 pins are CPH, CPL, EN/IN1, GND, IMODE, IPROPI, nFAULT, nSLEEP, OUT1, OUT2,
// PGND, PH/IN2, PMODE, VCP, VM, VREF). VIH is fixed at 1.5 V min in silicon, so
// the Pico's 3.3 V drives it directly. The carrier in use exposes only VM - no
// VCC - which confirms it.
//
// nFAULT is OPEN DRAIN, so its high level comes from whatever pulls it up. With
// no VCC on the board there is no external pull-up, so the RP2040's internal one
// (INPUT_PULLUP in initializePeripherals) is it, sitting at a safe 3.3 V.
//
// If a future carrier does offer a pull-up rail, DO NOT tie it to 5 V: the
// RP2040 is not 5 V tolerant (GPIO abs max ~3.6 V) and an idle-high nFAULT would
// park 5 V on GP7 continuously. The internal pull-up is weak (~50-80k); if that
// picks up noise and causes fault chatter, add an external 10k to 3.3 V.
#define MOTOR_NFAULT_PIN        7       ///< DRV8876 nFAULT - open-drain input, LOW=fault (was TB6612 AIN2)
// No MOTOR_IPROPI_PIN: IPROPI is grounded, so there is no current measurement
// anywhere in the system. nFAULT is the only feedback the driver gives us - but
// with chopping disabled, every nFAULT assertion is now a REAL fault (OCP / TSD
// / UVLO). No disambiguation needed, which is a large simplification over the
// version of this firmware that had to guess.

/**
 * @brief DRV8876 strap pins - NOT GPIOs
 *
 * PMODE, IMODE and VREF are set with resistors on the carrier and are latched
 * when nSLEEP goes high. To change one: nSLEEP low, wait 1 ms, change it, nSLEEP
 * high. Recorded here so the intended board config lives with the firmware.
 *
 * PMODE -> must be LOW for PH/EN mode, which is what this firmware drives. Left
 *   floating it reads Hi-Z (internal 156k/44k divider parks it at ~1.1 V) which
 *   selects independent half-bridge mode and DISABLES internal current
 *   regulation. If the carrier straps PMODE high instead, the part is in PWM
 *   mode and the single-PH drive below is wrong.
 *
 * IMODE -> 20k to GND, quad-level 2. With IPROPI grounded the chopping-mode half
 *   of this setting is dead, so the only thing it still selects is the
 *   OVERCURRENT RESPONSE: level 2 auto-retries after 2 ms rather than latching
 *   the outputs off. Levels 1 and 2 auto-retry; 3 and 4 latch. Leave it at 20k.
 *
 *   The old warning that level 2 was unsafe at 100% duty no longer applies. It
 *   was about cycle-by-cycle chopping needing a fresh EN edge to leave the brake
 *   state, and there is no chopping now. VELOCITY_PID_OUTPUT_LIMIT is still held
 *   well below 255, but for current headroom - see that constant, not this one.
 *
 * VREF  -> with IPROPI grounded, VREF only has to sit ABOVE GND; its exact value
 *   sets nothing. Datasheet 7.3.3.2 wants IPROPI-to-GND *and* VREF > GND to
 *   disable regulation cleanly, and VREF is tied to 3.3 V, which satisfies it.
 *   Leave it there. Do NOT drive it from a GPIO.
 *
 *   *** CS (pin 6) AND VREF (pin 5) ARE ADJACENT. DO NOT SWAP THEM. ***
 *
 *   Still true and still worth the shout, even though nothing now depends on the
 *   trip point. Landing the 3.3 V lead on CS instead of VREF cost two motor swaps
 *   and days of misdiagnosis: IPROPI is a current OUTPUT, and forcing it to 3.3 V
 *   pinned V_IPROPI above a floating VREF, so the trip comparator sat permanently
 *   satisfied and the driver chopped on EVERY PWM cycle. Near-zero usable torque
 *   in both directions - which looks exactly like an inadequate gearbox. No
 *   damage results (IPROPI abs max -0.3 to 5.75 V) but nothing works.
 *
 *   NOTE: an earlier version of this comment drew the rule "if nFAULT chops at a
 *   duty whose stall current is well under the expected trip point, suspect the
 *   SENSE WIRING". That rule is WRONG and cost days. The sense wiring was fine.
 *   ITRIP trips on INSTANTANEOUS current with 1.1 us blanking, so commutation
 *   and switching transients breach it while the average sits at half the trip
 *   point - chopping well below the computed ITRIP is entirely normal. See
 *   "WHY THERE IS NO CURRENT TRIP" below.
 *
 * CARRIER STATUS as confirmed on the board in hand:
 *   VM, GND      exposed (no VCC - the chip has no logic rail, VM feeds an
 *                internal regulator plus the CPH/CPL/VCP charge pump)
 *   nFAULT       exposed - the only driver feedback there is, and with chopping
 *                off it now means unambiguously "real fault"
 *   CS/IPROPI    exposed. Shipped with 2.486k to GND (measured at the pad);
 *                now strapped to GND to disable regulation.
 *   VREF         exposed, tied to 3.3 V
 *   IMODE        20k to GND as shipped = quad-level 2. Already correct.
 *   PMODE        OPEN AS SHIPPED - now driven LOW from GP26 (MOTOR_PMODE_PIN)
 *                rather than solder-bridged.
 *
 * *** WHY PMODE MUST NEVER BE LEFT FLOATING ***
 *
 * Left open, the internal 156k/44k divider parks PMODE at ~1.1 V = Hi-Z =
 * independent half-bridge mode. That used to carry two failures; disabling
 * current regulation deliberately has retired the first one, and the second is
 * the dangerous one anyway:
 *
 *   1. (No longer relevant.) Hi-Z also disables internal current regulation.
 *      This design now disables it on purpose, so nothing is lost here.
 *
 *   2. The DOWN direction inverts. Independent half-bridge drives each output
 *      directly (Table 5: INx=0 -> low side on, INx=1 -> high side on). With
 *      MOTOR_PH_DOWN = HIGH, OUT2 sits at VM and the EN duty is inverted, so
 *      MIN_PWM_DOWNWARD of 30 applies ~88% duty downward. The velocity PID then
 *      sees excess speed and reduces its output, which inverted means MORE
 *      drive - positive feedback, and a 2 kg mass slams down with gravity
 *      assisting. The runaway detector is what guards this now.
 *
 * The fix is a solder bridge - PMODE and GND are adjacent pins in both packages:
 *   HTSSOP (PWP): pin 15 = GND, pin 16 = PMODE
 *   VQFN   (RGT): pin 13 = GND, pin 14 = PMODE
 *
 * VERIFY THE STRAPS BY PROBING THE IC LEGS if they are not on the header - they
 * are whatever the board decided. Pin numbers by package:
 *   HTSSOP (PWP): PMODE 16, VREF 5, IPROPI 6, IMODE 7
 *   VQFN  (RGT):  PMODE 14, VREF 3, IPROPI 4, IMODE 5
 */

// =============================================================================
// WHY THERE IS NO CURRENT TRIP
// =============================================================================
// The DRV8876's ITRIP chopper is deliberately disabled (IPROPI strapped to GND,
// datasheet 7.3.3.2). No firmware constant sets a trip point, because there is
// no trip point.
//
// TWO reasons, and the second is the one that actually forced it.
//
// FIRST: the chopper was preventing the actuator from lifting at all. Measured
// on hardware - at duty 60 the actuator sagged 121 counts with nFAULT at 13/60;
// with IPROPI strapped to GND the same duty gives net UPWARD motion at 0/60.
//
// That happened at a duty demanding at most 0.86 A against a computed ITRIP of
// 1.33 A, i.e. chopping where the arithmetic says it is impossible. The reason
// is that ITRIP compares INSTANTANEOUS current with only tBLK = 1.1 us blanking
// and tDEG = 0.6 us deglitch, so brush commutation spikes breach it while the
// average sits well below. The datasheet concedes this in 7.3.3.2 by
// recommending a 10 nF cap on IPROPI to stop transients "prematurely
// triggering" regulation. The computed trip point was never the real one.
//
// ANYONE RE-ENABLING CURRENT SENSING: fit that 10 nF cap, and bench-confirm that
// a duty which should NOT chop indeed does not. Do not trust the equation alone.
//
// SECOND: neither motor can draw enough current to threaten the driver anyway.
//
//   90:1 / 110 rpm   3.3 ohm winding measured  ->  3.64 A locked rotor at 12 V
//   56:1 / 178 rpm   (the intended end state)  ->  same family, same order
//
// against a DRV8876 rated 3.5 A continuous with OCP at 3.5-5.5 A and a 44 C/W
// package. The duty ceiling matters more than any of that: at
// VELOCITY_PID_OUTPUT_LIMIT the bridge can only apply 160/255 x 12 V = 7.5 V,
// so the worst case the motor can actually reach is 7.5 / 3.3 = 2.28 A. That is
// below the continuous rating, let alone OCP.
//
// *** VELOCITY_PID_OUTPUT_LIMIT IS NOW THE CURRENT LIMIT. ***
// It is the only thing bounding motor current. Raising it toward 255 raises the
// worst case toward 3.64 A, which crosses into OCP territory. Do not raise it
// without redoing this arithmetic for the motor actually fitted.
//
// What remains protecting the hardware, in order of who acts first:
//   1. VELOCITY_PID_OUTPUT_LIMIT     duty ceiling, hence current ceiling
//   2. Firmware stall detection      no motion under real duty -> latch
//   3. DRV8876 OCP                   3.5-5.5 A, 2 ms auto-retry (IMODE level 2)
//   4. DRV8876 thermal shutdown      160-190 C, hysteretic
//
// This is a real change in posture from the TB6612 era. That part had NO
// overcurrent protection at all and died of it; the DRV8876 has two independent
// hardware backstops even with regulation off. Chopping was never what made this
// design safe - it was a torque-shaping feature that this application does not
// need, and its misconfiguration cost far more than it ever protected.

// Linear Actuator Encoder Pins
//
// SWAPPED TWICE. The first swap (to A=GP2/B=GP3) was verified by hand against
// the 30:1 motor's encoder. Fitting the 110 rpm motor brought a DIFFERENT
// encoder, phased the opposite way, so it is back to A=GP3/B=GP2.
//
// LESSON: encoder phasing is a property of the motor, not the harness. Re-verify
// it on every motor change, by hand with the driver asleep, before applying
// power. It is cheap and it is the failure that turns the velocity loop into
// positive feedback.
//
// The physical wiring did not change; only the role assignment here.
//
// This has to stay right. An inverted encoder makes the velocity PID read the
// wrong sign, which turns the loop into positive feedback on a gravity-loaded
// actuator and drives it into a hard stop. The runaway guard in
// handleMotorSafety() is the backstop, but catching it cold like this is better.
#define MOTOR_ENCODER_A         3       ///< Encoder A (swapped again for the 110 rpm motor's encoder)
#define MOTOR_ENCODER_B         2       ///< Encoder B (swapped again for the 110 rpm motor's encoder)

/**
 * @brief Motor Direction Polarity (DRV8876 PH pin level)
 *
 * "Up" means positive PWM / away from the lower limit switch; "down" means
 * toward the limit switch at position 0. Four TB6612 DIR defines collapse to
 * two, because PH/EN encodes direction in a single level.
 *
 * Carried over from the TB6612 wiring: up was AIN1=LOW/AIN2=HIGH, which on that
 * part is CCW = OUT1 low, OUT2 high. The DRV8876 produces the same output state
 * with PH=0, so up maps to PH LOW - PROVIDED the motor leads keep the same
 * orientation on OUT1/OUT2 through the rewire. If the back drives the wrong way
 * on first test, flip these two lines (or swap the motor leads, not both).
 *
 * The encoder rides the same shaft either way, so position sense (up = +position)
 * is unchanged and MOTOR_ENCODER_A/B do not need revisiting.
 */
#define MOTOR_PH_UP             HIGH    ///< PH level to drive actuator up (flipped for the 110 rpm motor)
#define MOTOR_PH_DOWN           LOW     ///< PH level to drive actuator down (flipped for the 110 rpm motor)

/**
 * @brief PWM Configuration for Motors
 *
 * PWM_FREQUENCY and PWM_MAX_DUTY are now applied - initializePeripherals() calls
 * analogWriteFreq() and analogWriteRange() with them. They were dead defines for
 * a long time, and the bridge ran at whatever the core defaulted to.
 *
 * 24 kHz, RAISED FROM 4 kHz after a bench sweep. At identical duty the actuator
 * moved 3.2x further at 24 kHz than at 4 kHz (-28 -> -90 counts per burst).
 *
 * That measurement was taken while current regulation was still active, and the
 * mechanism was believed to be ripple against a PEAK-sensing trip point: ITRIP
 * compares instantaneous current with only 1.1 us of blanking, peak-to-peak
 * ripple is V*D*(1-D)/(L*f), and at 4 kHz that ripple breached the trip point on
 * nearly every cycle even with the AVERAGE current well under it.
 *
 * WITH CHOPPING NOW DISABLED THAT MECHANISM IS GONE, so the 3.2x result no
 * longer explains anything about the present configuration. 24 kHz is kept
 * regardless - it is above audible, it cuts ripple current and therefore motor
 * heating, and nothing argues for going back down. But do not cite the 3.2x
 * number as evidence for the current setup; re-measure if it ever matters.
 *
 * The old tOFF argument for 4 kHz was wrong on its own terms too: tOFF applies
 * only to FIXED OFF-TIME mode (IMODE levels 1 and 4), and this board runs level
 * 2. It is doubly moot now that regulation is off.
 *
 * PWM_MAX_DUTY sets the analogWrite RANGE, not a safety limit. The enforced duty
 * ceiling is VELOCITY_PID_OUTPUT_LIMIT - do not confuse the two.
 */
#define PWM_FREQUENCY           24000   ///< PWM frequency in Hz - applied via analogWriteFreq()
#define PWM_RESOLUTION          255     ///< PWM resolution (8-bit) - unused, see PWM_MAX_DUTY
#define PWM_MIN_DUTY            0       ///< Minimum PWM duty cycle - unused
#define PWM_MAX_DUTY            255     ///< analogWrite range - NOT a safety limit

// =============================================================================
// ROBOT PHYSICAL DIMENSIONS
// =============================================================================

/**
 * @brief Linear Actuator Configuration
 */
// Set to 0.350 as specified. Now a REAL measurement, not an indicated one -
// GEAR_RATIO is calibrated, so 0.350 here is 350 mm of actual travel from the
// limit switch (23,320 counts).
//
// *** THIS LEAVES ALMOST NO MARGIN, AND THERE IS NO TOP LIMIT SWITCH ***
//
// The calibration move reached 347.0 mm without stalling, so the hard stop is
// somewhere above that but its exact position has not been measured. If 350 mm
// IS the hard stop, this constant commands the actuator straight into it, and
// the only thing that intervenes is the stall detector - after it has already
// been pushing at up to VELOCITY_PID_OUTPUT_LIMIT for STALL_TIMEOUT_MS.
//
// Recommend 0.340 unless 350 mm is known to be usable travel rather than the
// stop itself. Cheap insurance: 10 mm costs nothing and the failure it prevents
// is grinding the belt against a hard stop on every full-extension command.
#define ACTUATOR_MIN_POSITION    0.0     ///< Minimum actuator position (meters)
#define ACTUATOR_MAX_POSITION    0.350   ///< Maximum actuator position (m) - REAL, measured scale. See margin note above.
#define ACTUATOR_HOME_POSITION   0.0     ///< Home/zero position (meters)

// =============================================================================
// MOTOR AND GEARING SPECIFICATIONS
// =============================================================================

/**
 * @brief Motor Specifications
 */
// JGB37-520 manufacturer matrix, 12 V rows for every gearbox this machine has
// run. Output RPM identifies the variant:
//
//   Ratio  NoLoad  NoLoad  Rated    Rated  Rated  Stall    Stall
//          RPM     mA      kg.cm    RPM    A      kg.cm    A
//   30:1   333     120     3.5      250    1.0    5        2.3   <- REJECTED
//   56:1   178     120     6.5      140    1.0    9        2.3   <- gen1, on order
//   90:1   110     120     10       85     1.0    15       2.3   <- CURRENT
//
// The 30:1 was fitted during the gen2 rebuild and could not lift the back. That
// was measured, not inferred: hands off, it produced no motion at duty 50 and
// sagged under gravity at duty 60, and at duty 120 into the top stop it simply
// chopped at ITRIP. Its 5 kg.cm stall torque is an absolute ceiling no driver,
// duty or trip setting can raise.
//
// Now on the 90:1 (110 RPM) as an interim, with the 56:1 (178 RPM) on order.
// The 56:1 is the intended end state - it was the gen1 configuration and is the
// speed/torque compromise this mechanism was built around.
//
// A second manufacturer sheet gives stall 2.4 A and 0.65 A at 1 kg.cm. Taken
// together with the matrix and the bench measurement:
//
// 1. Stall current. Two sheets agree on 2.3-2.4 A; measured terminal resistance
//    says 12 V / 3.3 ohm = 3.64 A. Both are right - they are different points on
//    the commutation cycle. 2.4 A implies ~5.0 ohm, which sits inside the
//    measured 3.3-5.9 ohm range, so the datasheet figure is stall at a TYPICAL
//    rotor position while 3.64 A is the worst-case minimum-resistance position.
//    (Brush voltage drop under load, which a low-current ohmmeter does not see,
//    closes part of the rest.) Protection sizing uses the worst case, 3.64 A.
//    At 3.64 A the TB6612 was dissipating ~6.6 W in a 0.78 W package.
//
// 2. Torque per amp, fitting T = Kt*(I - I_noload) with I_noload = 0.12 A. The
//    matrix's RATED-torque column is not self-consistent and is discounted: it
//    claims 70% of stall torque at 40% of stall current, impossible in a linear
//    machine. Stall torque and stall current are the trustworthy pair, and the
//    second sheet's 1 kg.cm @ 0.65 A corroborated them on the 30:1 within 15%.
//
//      30:1   5 kg.cm / (2.4-0.12) =  2.19 kg.cm/A
//      56:1   9 kg.cm / (2.4-0.12) =  4.13 kg.cm/A
//      90:1  15 kg.cm / (2.4-0.12) =  6.88 kg.cm/A   <- CURRENT
//
//    Kt scales with the gearbox ratio, as it must. The 90:1 makes 3.1x the
//    30:1's torque per amp. Beyond ~2.4 A extra current makes heat, not torque -
//    saturation and the stated stall rating cap it there.
//
// 3. At the belt. 20T GT2 -> 40 mm pitch circumference -> 6.37 mm pitch radius,
//    so F = T/r gives force(kgf) = torque(kg.cm) x 1.571. For the 90:1 that is
//    force(kgf) = 10.81 x (I - 0.12):
//
//      0.54 A (est. load)      4.5 kgf    44 N   <- what the lift needs
//      1.0 A  (motor rated)    9.5 kgf    93 N
//      1.33 A (ITRIP)         13.1 kgf   128 N   <- practical working limit
//      2.3 A  (stall)         23.6 kgf   231 N   <- absolute ceiling
//
//    For contrast the 30:1 offered 4.19 kgf at ITRIP and 7.9 kgf at stall, which
//    is why it failed. The 90:1 has roughly 3x margin over the load rather than
//    sitting on top of it.
//
//    LOAD ESTIMATE: back plus arms is 2 kg and the arms lift at most 1 kg, so
//    3 kgf if the belt is 1:1. The bench says otherwise - the 30:1 sagged at
//    2.6 kgf and barely moved at 4.5 kgf - so the real figure is nearer 4-5 kgf.
//    The gap is friction, or the 1:1 belt assumption, and it is still UNMEASURED.
//    Put a spring gauge on the belt and settle it; every margin here depends on
//    a number nobody has actually weighed.
//
// 4. The MOTOR has a continuous rating too: 1.0 A. At an estimated 0.54 A the
//    90:1 lift now sits comfortably INSIDE it, unlike the 30:1 which needed
//    ~1.5 A - over 2x rated - for the same job. Holding at rest still costs
//    nothing either way: short-brake produces zero torque at zero speed, so the
//    motor draws no current while parked and geartrain stiction does the work.
//    The 90:1 also back-drives less readily than the 30:1, which improves the
//    holding margin discussed under the brake notes.
//
// MOTOR_MAX_CURRENT_MA, MOTOR_STALL_TORQUE_NM and OUTPUT_SHAFT_MAX_RPM are
// documentation only - no source file reads them.
#define MOTOR_VOLTAGE           12.0    ///< Motor operating voltage (V)
// Terminal resistance measured across a full rotor turn: 3.3 to 5.9 ohm. That
// ~1.8x swing is normal 5-pole commutation, not a fault - depending on rotor
// position the brushes bridge either 1 coil against 4 in series (0.8R) or 2
// against 3 (1.2R), plus variable brush contact resistance. No position reads
// near-zero or open, which is what rules out a shorted or broken winding.
//
// USE THE MINIMUM for anything protection-related: a stalled rotor settles
// wherever the torque balance leaves it, so it can genuinely sit at the
// low-resistance position and pull the full 12 V / 3.3 ohm. Do NOT replace this
// with the average - it is deliberately worst case.
//
// The flip side, for torque estimates rather than current limits: at a given
// duty the motor makes almost 2x less current (hence torque) at the 5.9 ohm
// position than at 3.3. Break-away from rest is therefore position-dependent,
// so a first-lift test can pass or fail on where the rotor happened to stop.
// Try it from several positions before concluding anything.
#define MOTOR_WINDING_OHMS      3.3     ///< MEASURED minimum across rotor positions (range 3.3-5.9)
#define MOTOR_WINDING_OHMS_MAX  5.9     ///< MEASURED maximum - documentation only
#define MOTOR_STALL_CURRENT_MA  3640    ///< 12 V / 3.3 ohm measured. Matrix claims 2300; see note above
#define MOTOR_STALL_CURRENT_SPEC_MA 2300 ///< Datasheet figure, retained for reference only
#define MOTOR_RATED_CURRENT_MA  650     ///< Rated continuous current (mA), 2nd sheet @ 1 kg.cm. Matrix says 1000; see note
// THE MOTOR IS THE 90:1 / 110 RPM UNIT AFTER ALL. These constants are original
// and CORRECT - a brief detour rescaled them to a supposed 60.6:1 motor, which
// was an artifact of the broken quadrature decoder undercounting by 1.487x.
// Once the decoder was fixed the calibration landed on 90.1:1, i.e. exactly the
// datasheet ratio, and the whole motor-identity question evaporated.
//
// Confirmed independently by speed, which does not use the encoder scale:
// measured speed vs applied voltage has slope 6.06 mm/s per volt, against a
// theoretical 6.11 for 110 rpm through a 40 mm pulley. 0.8% apart.
//
// The same fit gives the intercept: the load costs 2.96 V, i.e. ~0.90 A through
// the 3.3 ohm winding. Force = 10.81 x (0.90 - 0.12) = 8.4 kgf against an
// inferred 4.5 kgf load - a comfortable margin, and the drive runs at ~47% of
// its voltage-adjusted no-load speed, which is a healthy operating point.
#define MOTOR_KT_KGCM_PER_A     6.88    ///< Torque constant at 90:1 (15 kg.cm / 2.18 A) - documentation only
#define PULLEY_PITCH_RADIUS_M   0.006366 ///< 20T GT2: 40mm circumference / 2pi. Belt force = torque / this
#define MOTOR_MAX_CURRENT_MA    1500    ///< Target DRV8876 ITRIP setting (mA) - documentation only, not enforced
#define MOTOR_STALL_TORQUE_NM   1.47    ///< 15 kg.cm stall torque at 90:1
#define MOTOR_FREE_SPEED_RPM    110     ///< No-load speed at gearbox output (RPM), 90:1 @ 12 V - CONFIRMED by voltage-speed slope
#define MOTOR_RATED_RPM         85      ///< Rated (loaded) speed at gearbox output (RPM)

/**
 * @brief Gear Ratio Configuration
 */
/**
 * GEAR_RATIO is an effective counts-per-meter calibration constant (motor:output
 * shaft x any encoder-PPR / pulley error), NOT the datasheet gearbox ratio. It
 * was empirically tuned on the gen1 motor: 106 gave 325 mm on a 400 mm command,
 * so 106 x 400/325 = 130.5.
 *
 * The encoder, pulley and belt never changed, so this constant scales with the
 * gearbox ratio and nothing else:
 *
 *   56:1 (gen1, measured)  130.5
 *   30:1                   130.5 x 30/56 =  69.9   <- rejected motor
 *   90:1 (CURRENT)         130.5 x 90/56 = 209.7
 *
 * Note the 2.33x gap between the measured 130.5 and the datasheet 56:1 - that is
 * a fixed error in the assumed encoder PPR or pulley geometry, not something the
 * gearbox affects, which is exactly why straight ratio scaling is valid here.
 *
 * *** MEASURED ON HARDWARE. 209.7 WAS WRONG BY 3.46x. ***
 *
 * Homed down onto the limit switch, zeroed there, drove up, measured with a rule:
 *
 *     23120 counts  =  347.00 mm actual   ->  66.63 counts/mm
 *     indicated 100.23 mm / actual 347.00 mm  =  0.289x
 *     GEAR_RATIO  209.7 -> 60.6            counts/m  230670 -> 66628
 *
 * Where 209.7 came from: it was scaled off gen1's empirical 130.5 by 90/56, on
 * the assumption that the constant tracks the gearbox ratio and nothing else.
 * Both ends of that were wrong - see MOTOR IDENTITY below.
 *
 * EVERY DERIVED NUMBER IN THIS FILE MOVED WITH IT. Speeds were understated 3.46x,
 * which is what made the actuator look like it was crawling at 19-33% of no-load
 * when it is in fact running at a healthy ~49%.
 *
 * *** 90:1 - MEASURED, AND IT IS THE DATASHEET RATIO ***
 *
 * Calibrated after fixing the quadrature decoder: 34685 counts = 350.00 mm.
 *
 *     measured   99,100 counts/m
 *     model      11 PPR x 4 x 90 / 0.040 m = 99,000 counts/m      0.1% apart
 *
 * Every geometry assumption in this file is therefore correct: 11 PPR, 4x
 * quadrature, 20T 2GT pulley (40 mm), 90:1 gearbox, 110 rpm output.
 *
 * WHY 209.7 WAS WRONG, AND WHY IT MISLED FOR SO LONG. The old encoder ISR lost
 * roughly one count in three (99,100 / 66,660 = 1.487x). Every calibration ever
 * done on this machine - including gen1's empirical 130.5, which is where 209.7
 * was scaled from - was measured through that bug. It is also the source of the
 * "unexplained 2.33x gap" this comment used to describe. There was no geometry
 * error to find. See processEncoderInterrupt() in motor_control.cpp.
 *
 * Dead theories, recorded so nobody re-runs them:
 *   - 18T pulley with a 56:1 gearbox      (pulley confirmed 20T)
 *   - 3 mm belt pitch with the 90:1       (belt confirmed 2GT, 2 mm)
 *   - a ~60.6:1 motor of unknown origin   (decoder artifact)
 * All three were attempts to explain a discrepancy that was pure software.
 *
 * DO NOT adjust PULLEY_TEETH or BELT_PITCH_MM to "fix" anything here. They
 * multiply with GEAR_RATIO into counts-per-metre, and that product is measured.
 *
 * RE-CALIBRATE AFTER ANY ENCODER OR DECODER CHANGE. That is the lesson: this
 * constant is only as good as the counting underneath it.
 */
#define GEAR_RATIO              90.0    ///< MEASURED 90.1 (34685 counts = 350.00 mm); set to the exact datasheet 90:1
#define OUTPUT_SHAFT_MAX_RPM    (MOTOR_FREE_SPEED_RPM / GEAR_RATIO)  ///< Max output shaft RPM

/**
 * @brief Linear Actuator Mechanical Configuration
 */
#define PULLEY_TEETH            20      ///< Number of teeth on drive pulley (20T spline gear)
#define BELT_PITCH_MM           2.0     ///< GT2 timing belt pitch (mm between tooth centers)
#define PULLEY_CIRCUMFERENCE_M  ((PULLEY_TEETH * BELT_PITCH_MM) / 1000.0)  ///< Pulley circumference in meters
#define METERS_PER_REVOLUTION   PULLEY_CIRCUMFERENCE_M  ///< Linear distance per output shaft revolution

/**
 * @brief Encoder Configuration
 */
#define ENCODER_PPR             11     ///< Encoder pulses per revolution
#define ENCODER_COUNTS_PER_REV  (ENCODER_PPR * 4)  ///< Quadrature encoding (4x)
#define GEARED_COUNTS_PER_REV   (ENCODER_COUNTS_PER_REV * GEAR_RATIO)  ///< Counts per output shaft revolution
#define ENCODER_COUNTS_PER_METER (GEARED_COUNTS_PER_REV / METERS_PER_REVOLUTION)  ///< Encoder counts per meter of linear travel



// =============================================================================
// MOTION CONTROL PARAMETERS
// =============================================================================

/**
 * @brief Maximum Velocities and Accelerations for Linear Actuator
 *
 * Gen2 bring-up values (JGB37-520B 333 rpm + DRV8876), deliberately conservative.
 * No-load linear speed is now 333 rpm x 0.04 m/rev / 60 = 0.222 m/s, up from
 * 0.117 m/s with the 176 rpm motor. The old 0.10 m/s cap was ~85% of the old
 * motor's free speed (so it always demanded near-full duty); 0.06 m/s is ~27% of
 * the new free speed and leaves the velocity PID a wide unsaturated band.
 * Acceleration is halved because the accel phase is the peak-current event and
 * the DRV8876 is only thermally comfortable to ~1.5 A continuous.
 * Raise both only after measuring real current on IPROPI.
 */
// *** 0.035 IS NOT ACHIEVABLE ON THIS HARDWARE - MEASURED ***
//
// The duty sweep (see MIN_PWM_UPWARD) puts the actuator at 0.012-0.014 m/s peak
// at duty 160, the ceiling. Averaged over a burst it is ~0.010 m/s. That is
// roughly 2.7x short of this constant, at maximum allowed duty, lifting.
//
// Theory agrees it was always optimistic: 110 rpm x 40 mm belt circumference is
// 0.073 m/s no-load at 12 V, so duty 160 (63%) is 0.046 m/s no-load. Measuring
// 0.013 means the actuator runs at ~28% of no-load speed - deep into the loaded
// part of the torque-speed curve. Asking for 0.035 m/s (48% of the FULL-duty
// no-load speed) needs the motor near its max-power point continuously, which
// it plainly cannot sustain against this load.
//
// CONSEQUENCE IF LEFT AT 0.035: the velocity PID can never reach setpoint, so it
// saturates at OUTPUT_LIMIT and the integral winds to its limit on every move.
// The loop degenerates into bang-bang at the duty ceiling and the KP/KI values
// below stop meaning anything. RUNAWAY_VELOCITY_MPS is also derived from this
// constant, so an inflated value raises the overspeed threshold to 0.070 m/s -
// about 5x the fastest speed the actuator can actually reach, which makes the
// overspeed half of the runaway detector effectively dead.
//
// 0.046, from two clean full-stroke measurements on the fixed decoder.
//
//     duty 200 (9.41 V) -> 46.06 mm/s steady, 351.3 mm in 7.68 s
//     duty 220 (10.35 V) -> 51.54 mm/s steady, 351.6 mm in 6.87 s
//     fit:  v = 5.82 x (V_applied - 1.50)  mm/s
//
// Slope sanity check: 5.82 mm/s per volt against a theoretical 6.11 for 110 rpm
// through a 40 mm pulley - within 5%, confirming the motor identity without
// using the encoder scale at all.
//
// The intercept says the load costs only 1.50 V, i.e. ~0.45 A through the 3.3
// ohm winding. That is less than half the motor's 1.0 A continuous rating and
// ~0.14 W in the driver. This actuator is lightly loaded; earlier revisions of
// this comment claimed 0.90 A and before that 2.2 A, both computed through the
// broken decoder.
//
// 0.046 m/s is ~90% of what duty 220 delivers, reached at ~duty 200, leaving 20
// counts of headroom under the ceiling for load variation. A 350 mm transition
// takes 7.6 s.
//
// ON 6 s: reachable, but not free. The fit puts 58.3 mm/s at duty 245, which is
// exactly the OCP floor (3.50 A on a stall) - so a jam would be answered by the
// driver's 2 ms retry chatter rather than the firmware stall detector. Duty 220
// already gives 6.87 s measured, so the last 0.9 s costs the entire OCP margin.
// Not worth it. If 6 s becomes a hard requirement, fit the 56:1/178 rpm motor.
#define MAX_ACTUATOR_VELOCITY       0.046    ///< Max linear velocity (m/s) - MEASURED, fixed decoder
#define MAX_ACTUATOR_ACCELERATION   0.35     ///< Max linear acceleration (m/s²) - halved to cap accel-phase current

/**
 * @brief PID Controller Parameters for Velocity Control
 *
 * Retuned for the gen2 plant (56:1 vs 130:1) and the DRV8876's current limit.
 * The old gains were sized for the 130:1 gearbox and ran permanently saturated,
 * which is what let the TB6612 sit at 100% duty into a near-stall load.
 *
 * KP: units are PWM counts per (m/s) of velocity error. Plant gain is ~1150
 *   counts/(m/s) (255 counts -> 0.222 m/s no-load). KP=2000 puts a full-scale
 *   0.06 m/s error at ~120 counts (about half output) and only saturates at
 *   0.128 m/s error - above the velocity cap, so the loop no longer bottoms out
 *   in saturation the way KP=7000 did (it saturated at just 0.036 m/s error).
 *
 * KI: with KP lowered, the integral now has to supply the steady-state duty that
 *   holds the back against gravity, so it must be larger than the old 600.
 *   KI=3000 with Ti = KP/KI ~= 0.67 s builds the ~75-count gravity bias in
 *   roughly a second of small error - deliberately unhurried.
 *
 * INTEGRAL_LIMIT: clamps the accumulator, whose units are (m/s)*s = meters.
 *   The old value of 100 was dimensionally meaningless (it would need 0.1 m/s of
 *   error sustained for 1000 s to engage), so anti-windup was effectively off.
 *   0.04 caps the integral term at 3000 x 0.04 = 120 counts, just under the
 *   output ceiling below, so windup cannot push past the duty limit.
 *
 * KD: stays disabled. Velocity is differentiated from an 11 PPR encoder and is
 *   too noisy to differentiate again.
 *
 * OUTPUT_LIMIT: the real conservative lever, and the only duty clamp that is
 *   actually enforced (see applyVelocityPID; the PWM_MAX_DUTY define below is
 *   dead code). With the winding MEASURED at 3.3 ohm, the duty ceiling is a hard
 *   current bound - worst case is a dead stall, where current = duty x 12 / 3.3:
 *
 *   The winding is unchanged, so stall current per duty is too - what changed at
 *   90:1 is that each amp now buys 10.81 kgf at the belt instead of 3.46:
 *
 *     duty  counts   stall current   belt force
 *       8%     20       0.29 A         1.8 kgf   <- MIN_PWM_DOWNWARD
 *      15%     38       0.54 A         4.5 kgf   <- estimated load
 *      20%     50       0.71 A         6.4 kgf   <- MIN_PWM_UPWARD
 *      63%    160       2.28 A        23.4 kgf   <- duty ceiling, AND the only
 *                                                   current limit there is
 *     100%    255       3.64 A        38.2 kgf   <- measured stall, NOT reachable
 *
 *   Nothing chops any of this off any more. With ITRIP disabled the table runs
 *   to its arithmetic end, and the ceiling of 160 is the sole thing keeping the
 *   motor off the 3.64 A row. It was chosen for SPEED - 0.035 m/s under load
 *   needs ~160 counts once back-EMF is subtracted (Ke ~= 0.00117 V/RPM, 5250 rpm
 *   at that speed) - and it now doubles as the current limit. Raising it toward
 *   255 walks into the DRV8876's 3.5 A OCP band.
 *
 *   The belt-force column is arithmetic, not a promise: it assumes the derived
 *   GEAR_RATIO and the inferred 4.5 kgf load, neither measured. What it does say
 *   is that force is no longer capped at 13.1 kgf by ITRIP, so a jam is now
 *   pushed against harder. The stall detector is what has to catch that, and it
 *   is the ONLY thing that will - see handleMotorSafety().
 *
 *   Thermally, a 2.28 A stall is ~3.6 W in the DRV8876's ~0.7 ohm and ~17 W in
 *   the winding. Inside the part's 44 C/W rating; hard on the motor if held.
 */
#define VELOCITY_PID_KP         2500.0  ///< Velocity proportional gain (PWM counts per m/s error)
#define VELOCITY_PID_KI         3000.0  ///< Velocity integral gain - supplies the gravity-holding bias
#define VELOCITY_PID_KD         0.0     ///< Velocity derivative gain - disabled, velocity signal too noisy
// Raised 160 -> 220 to hit the 350 mm transition inside 8 s. Set by OCP MARGIN,
// which is the criterion that actually matters here:
//
//   duty 220 -> dead-stall current 220/255 x 3.64 = 3.14 A, 10% under the
//               DRV8876's 3.5 A OCP floor. A jam is still caught first by the
//               firmware stall detector (250 ms, latches and classifies).
//   duty 245 -> 3.50 A, exactly at the OCP floor. Past this, OCP fires first at
//               2 ms auto-retry and the bridge chatters until the firmware
//               notices. Not damaging - IMODE level 2 auto-retries and TSD
//               backs it up - but it is the wrong protection responding, so
//               245 is the hard ceiling on the ceiling.
//
// 10% is thinner than the 18% at duty 200 and buys 9.0 s -> 7.8 s on a full
// stroke. Judged worth it; drop back to 200 if a stall ever chatters.
//
// THERMALS ARE A NON-ISSUE, which earlier revisions of this comment got badly
// wrong. Running current is ~0.90 A (from the voltage-speed intercept, see
// MAX_ACTUATOR_VELOCITY), not the 2+ A the uncalibrated scale implied. That is
// 0.90^2 x 0.70 = 0.57 W in the driver against a 44.3 C/W package - about 25 C
// of rise. The long thermal analysis that used to live here was computed from a
// speed figure that was 1.5x too slow, which made the motor look near-stall.
//
// The 3.3-5.9 ohm winding spread still makes the STALL case uncertain, but a
// stall is bounded by the detector at 250 ms, so it no longer drives the choice.
//
// Worth one measurement anyway: meter in series with a motor lead during a full
// lift. Expect ~0.9 A. Anything approaching 2 A means the load has changed and
// this whole derivation needs revisiting.
#define VELOCITY_PID_OUTPUT_LIMIT   220     ///< Duty ceiling (86%) - set by OCP margin, see above
#define VELOCITY_PID_INTEGRAL_LIMIT 0.045   ///< Integral windup limit (meters); KI x this = 135 counts

/**
 * @brief PWM Limits and Deadband
 *
 * These floors are applied to ANY non-zero PID output (see applyVelocityPID), so
 * they set the current the motor draws every time the controller nudges.
 *
 * Sized from the load at 90:1, where each amp buys 10.81 kgf at the belt. The
 * bench put the real load nearer 4-5 kgf than the 3 kgf the mass suggests - the
 * 30:1 sagged at 2.6 kgf and barely moved at 4.5 kgf - so size against 4.5:
 *
 *     4.5 kgf -> I = 0.12 + 4.5/10.81 = 0.54 A -> 38 counts at stall
 *
 * UPWARD is 100, SET FROM MEASUREMENT, superseding the 50 that was derived from
 * the force model. Duty sweep on hardware, four 300 ms bursts per level, chopping
 * disabled, counts moved per burst:
 *
 *     duty    mean   worst      duty    mean   worst
 *       60    +139     -26       120    +400    +294
 *       70    +230    +182       130    +464    +433
 *       80    +208     +56       140    +516    +492
 *       90    +203     +39       150    +581    +565
 *      100    +275    +209       160    +679    +608
 *
 * Linear above 100 at ~6.7 counts per duty count, extrapolating to zero at duty
 * 59 - the true break-away. But break-away is the wrong thing to set a floor
 * from: between 60 and 90 the WORST burst collapses to +26..+56 counts, i.e.
 * barely distinguishable from not moving, because break-away is where stiction
 * is won or lost at random. 100 is the lowest duty where every burst cleared
 * +200 counts. That is what a floor has to guarantee.
 *
 * The old 50 predicted 0.71 A / 6.4 kgf and "42% over the load". The actuator
 * does not reliably move at 50. Treat the force model as refuted, not merely
 * uncalibrated - see GEAR_RATIO and the belt-load note.
 *
 * Cost: floor 100 against OUTPUT_LIMIT 160 leaves only 60 counts of proportional
 * range, so control is coarse. Raising OUTPUT_LIMIT would widen it at the price
 * of current - and note the floor is applied AFTER the ceiling clamp, so a
 * MIN_PWM_UPWARD above OUTPUT_LIMIT silently defeats the ceiling. With IPROPI
 * grounded there is no way to see current in firmware; use a meter on a lead.
 *
 * DOWNWARD drops to 20: gravity assists, and 90:1 back-drives far less readily
 * than the 30:1 did, so both the runaway-descent risk and the holding problem
 * improve. 20 counts is 0.29 A / 1.8 kgf, enough to control the descent.
 *
 * NOTE: these floors plus the 2 mm position tolerance are what produce the
 * sag -> nudge -> sag limit cycle on a back-driveable gravity load. The real fix
 * is a proper dead-band in applyVelocityPID (force 0 when inside tolerance and
 * target velocity is 0), which is a code change, not a constant.
 */
#define MIN_PWM_UPWARD          100     ///< Minimum PWM for upward motion - MEASURED, lowest duty that moves reliably
#define MIN_PWM_DOWNWARD        20      ///< Minimum PWM for downward motion (gravity assists)

/**
 * @brief Position Tolerance
 * Promoted out of updateMotorControl() so stall recovery can use the same value
 * to decide whether a new command is genuinely asking for a different position.
 */
#define POSITION_TOLERANCE_M    0.002   ///< Position deadband (m)

/**
 * @brief Control Loop Timing
 */
#define CONTROL_LOOP_PERIOD_MS  2       ///< Control loop period in milliseconds (500 Hz)
#define CONTROL_LOOP_FREQ_HZ    (1000 / CONTROL_LOOP_PERIOD_MS)  ///< Control frequency



// =============================================================================
// SAFETY AND LIMITS
// =============================================================================

/**
 * @brief Safety Parameters
 */
#define WATCHDOG_TIMEOUT_MS     500     ///< Watchdog timeout for safety stop
#define EMERGENCY_STOP_DECEL    5.0     ///< Emergency stop deceleration (m/s²)
#define MIN_BATTERY_VOLTAGE     10.5    ///< Minimum battery voltage (V)
#define MAX_MOTOR_TEMP_C        80      ///< Maximum motor temperature (°C)

/**
 * @brief Stall Detection
 *
 * This is the ONLY overload detector in the system. The DRV8876 carrier does not
 * break IPROPI out, so there is no current measurement to threshold on, and a
 * stall has to be inferred from "we are commanding real duty but the encoder is
 * not moving". That was the condition that destroyed the TB6612.
 *
 * Resolution is not a problem: ENCODER_COUNTS_PER_METER is ~61600, so 0.005 m/s
 * is ~308 counts/s and a genuine stall reads exactly zero counts. The 250 ms
 * window is long enough to ride out direction reversals and the initial break
 * away from stiction, which legitimately show near-zero velocity at high duty.
 */
#define STALL_PWM_THRESHOLD     40      ///< |PWM| at or above which the motor counts as driven
#define STALL_VELOCITY_MPS      0.005   ///< |velocity| below which the motor counts as not moving
#define STALL_TIMEOUT_MS        250     ///< How long both must hold before latching a stall

/**
 * @brief Stall Recovery
 *
 * A stall latch is soft at first: it releases itself after STALL_RECOVERY_MS so
 * a transient jam does not need operator intervention. That bounds the duty
 * cycle of stall attempts to STALL_TIMEOUT_MS / STALL_RECOVERY_MS (~8%), which
 * is thermally harmless even at full stall current.
 *
 * After STALL_MAX_RETRIES consecutive stalls the latch goes hard and only a
 * command for a materially different position releases it, so the actuator
 * stops grinding against a real obstruction. The escalation also protects the
 * latch from handleWatchdog(), which parks actuator_cmd at the current position
 * on command timeout and would otherwise look like a fresh operator command.
 */
#define STALL_RECOVERY_MS       3000    ///< Soft latch auto-clears after this long
#define STALL_MAX_RETRIES       3       ///< Consecutive stalls before the latch goes hard

/**
 * @brief Distinguishing an Obstruction from an Overload
 *
 * A hardware block and an over-capacity lift produce the identical symptom -
 * commanding duty with no encoder motion - so the detector needs a second piece
 * of evidence. The discriminator is whether real motion was achieved earlier in
 * THIS move attempt:
 *
 *   moved, then stopped   -> ERROR_MOTOR_BLOCKED. Something got in the way.
 *   never moved at all    -> ERROR_LOAD_EXCEEDED. Could not break away.
 *
 * This is a best guess, not a proof. A block that happens to sit right at the
 * start position reads as an overload, and a load that only becomes too heavy
 * partway through travel reads as a block. Consumers should treat error_code as
 * a strong hint and use current_position (already published) to spot the giveaway
 * pattern: a real obstruction stalls at the SAME position across retries, an
 * overload stalls wherever it happens to start.
 *
 * The classification changes behaviour, not just reporting. A block hard-latches
 * immediately and skips the soft-retry cycle, because whatever stopped a moving
 * actuator might be a hand, and grinding at it three more times over nine seconds
 * is the wrong instinct. An overload keeps the soft retries - nothing is in the
 * way, the load is simply heavy, and retrying is harmless. Erring toward "block"
 * is the safe direction, so an ambiguous case that latches hard is acceptable.
 *
 * Escape from a hard latch is unchanged: command a materially different position.
 * Commanding the back DOWN moves it away from the obstruction and releases it.
 */
#define MOVED_CONFIRM_MS        50      ///< Sustained motion needed to count the attempt as "moving"

/**
 * @brief Runaway Detection
 *
 * The stall detector catches a motor that will not move. This catches the
 * opposite and more dangerous failure: one moving far faster than commanded, or
 * against the command. That is the signature of every sign-inversion fault, and
 * on a gravity-loaded actuator each turns the velocity loop into POSITIVE
 * feedback - the PID's correction makes the error worse and it runs to the stop.
 *
 *   PMODE left floating   independent half-bridge inverts the DOWN duty, so a
 *                         commanded 30 applies ~225. Caught by overspeed.
 *   Motor leads swapped   commanded up drives down. Caught by reversal.
 *   MOTOR_PH_UP flipped   same as swapped leads. Caught by reversal.
 *   Encoder A/B swapped   PID reads the wrong sign. Caught by reversal.
 *
 * These are exactly the mistakes available during first bring-up, and PMODE
 * being GPIO-driven rather than hardwired keeps the first one live indefinitely.
 *
 * Threshold choice. 2x MAX_ACTUATOR_VELOCITY = 0.12 m/s sits in a genuine gap:
 *   0.060 m/s  commanded maximum
 *   0.063 m/s  terminal velocity of a 3 kg load free-falling against the short
 *              brake (brake force balances gravity at ~0.99 A), so an
 *              uncommanded descent does NOT reach the threshold
 *   0.120 m/s  runaway threshold
 *   0.222 m/s  no-load speed - what an inverted-duty descent actually reaches
 * So it discriminates powered runaway from every legitimate motion.
 *
 * Recovery is deliberately absent: a runaway means the hardware is not what the
 * firmware believes it to be, and no amount of retrying fixes miswiring. The
 * latch clears only on reset.
 */
// DECOUPLED from MAX_ACTUATOR_VELOCITY - was (MAX_ACTUATOR_VELOCITY * 2.0).
//
// That derivation is a trap once MAX_ACTUATOR_VELOCITY is corrected downward to
// something achievable. At the honest 0.012 m/s it would put the overspeed
// threshold at 0.024 m/s - BELOW the 0.030-0.033 m/s peaks measured during
// ordinary gravity-assisted DOWN bursts. Every normal descent would latch a
// runaway fault that only a reset clears.
//
// The asymmetry is the point: ascent is limited by what the motor can produce,
// descent by what gravity does, and they are nowhere near each other. One
// constant cannot be derived from the other.
//
// Absolute, NOT derived from MAX_ACTUATOR_VELOCITY. Deriving it was a trap:
// ascent is limited by what the motor can produce, descent by gravity, and the
// two are nowhere near each other. The old (MAX_ACTUATOR_VELOCITY * 2.0) gave
// 0.070, below the descent peaks actually measured - every normal descent would
// have latched an unrecoverable runaway.
//
// On the corrected 99,100 counts/m scale: steady descent ~0.027 m/s, with peaks
// to ~0.090 m/s (transients around 3x steady - the descent is jerky). 0.150
// clears the observed peak by ~67% and sits well below any speed implying a real
// sign inversion on a gravity load.
//
// Re-check after the decoder fix: some of that 3x transient was the old decoder
// injecting spurious counts, so the true peaks may now be lower.
#define RUNAWAY_VELOCITY_MPS    0.150   ///< Overspeed threshold (m/s) - absolute, see above
#define RUNAWAY_REVERSE_MPS     0.015   ///< Wrong-direction speed that counts as a reversal (m/s)
#define RUNAWAY_CONFIRM_MS      150     ///< How long either condition must hold before latching

/**
 * @brief NO EARLY WARNING FOR OVERLOAD
 *
 * Worth stating plainly, because the firmware used to have one. Chopping at
 * ITRIP fired BEFORE motion stopped, so rising friction, a partial obstruction
 * or an over-weight payload showed up as an amber LED while the actuator was
 * still moving. Disabling current regulation removed that signal outright.
 *
 * What is left only fires AFTER motion has already stopped: the stall detector
 * needs STALL_TIMEOUT_MS of commanded-duty-but-no-encoder-motion. There is no
 * longer any way to notice a mechanism that is working too hard but still
 * winning. Restoring one means reconnecting IPROPI to an ADC - see the PIN MAP
 * note in the pin section for what that costs.
 */

/**
 * @brief DRV8876 nFAULT Handling
 *
 * With current regulation disabled (IPROPI to GND), nFAULT has exactly one
 * meaning: a real device fault - overcurrent, thermal shutdown, or undervoltage
 * lockout. It no longer pulses for current chopping.
 *
 * This removed a whole class of logic. The firmware used to disambiguate the two
 * by whether the bridge was commanded to drive, and to count chop pulses over a
 * window to raise a "current limiting" flag. Both are gone: any debounced nFAULT
 * low, driving or idle, is now simply a fault.
 *
 * Debouncing still matters. nFAULT is open-drain held up only by the RP2040's
 * weak internal pull-up (~50-80k), so it is noise-prone; see the pin notes.
 */
#define NFAULT_DEBOUNCE_SAMPLES 5       ///< Consecutive low reads before latching a driver fault

/**
 * @brief Calibration descent guards
 *
 * calibrateActuator() drives the bridge directly with normal motor control
 * suspended, so motor.pwm_output is stale and NEITHER the stall detector nor the
 * runaway detector can fire on that path - runaway is skipped explicitly, stall
 * silently never triggers because pwm_mag reads 0. Until these constants existed
 * the only bound on a failed homing run was CALIBRATION_TIMEOUT_MS, i.e. up to
 * 20 s of driving into the bottom stop at the full duty ceiling.
 *
 * MIN_COUNTS is deliberately a very low bar: at 99 counts/mm a healthy descent
 * covers ~28 mm (2800 counts) in the stall window, so 20 counts distinguishes
 * "genuinely not moving" from "moving slowly" with a wide margin.
 *
 * REVERSE_COUNTS guards the direction. This is the only motion path with no
 * runaway detection, and it drives a gravity-loaded axis downward - the exact
 * situation a swapped motor lead or flipped PH polarity turns dangerous. 200
 * counts is ~2 mm of travel the wrong way.
 */
#define CALIBRATION_STALL_MS        400     ///< No-progress window before aborting the homing descent
#define CALIBRATION_MIN_COUNTS      20      ///< Counts of progress needed within that window
#define CALIBRATION_REVERSE_COUNTS  200     ///< Upward counts while commanded down that abort homing
#define DRIVER_FAULT_RETRY_MS   1000    ///< Minimum spacing between nSLEEP re-arm attempts

/**
 * @brief Error Codes
 */
#define ERROR_NONE              0x00    ///< No error
#define ERROR_INVALID_COMMAND   0x01    ///< Invalid command received
#define ERROR_MOTOR_FAULT       0x02    ///< Motor driver fault
#define ERROR_ENCODER_FAULT     0x03    ///< Encoder fault
#define ERROR_LOW_BATTERY       0x04    ///< Low battery voltage
#define ERROR_OVERTEMPERATURE   0x05    ///< Motor overtemperature
#define ERROR_COMMUNICATION     0x06    ///< Communication timeout
#define ERROR_MOTOR_STALL       0x07    ///< Stall of undetermined cause (umbrella; firmware emits 0x09/0x0A)
#define ERROR_DRIVER_FAULT      0x08    ///< DRV8876 nFAULT asserted while bridge commanded idle
#define ERROR_MOTOR_BLOCKED     0x09    ///< Was moving, then abruptly stopped - obstruction
#define ERROR_LOAD_EXCEEDED     0x0A    ///< Never broke away from rest - load beyond lift capacity
#define ERROR_MOTOR_RUNAWAY     0x0B    ///< Moving far faster than commanded, or against the command



// =============================================================================
// MATHEMATICAL CONSTANTS
// =============================================================================

#ifndef PI
#define PI                      3.14159265358979323846  ///< Pi constant
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG              (180.0 / PI)            ///< Radians to degrees conversion
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD              (PI / 180.0)            ///< Degrees to radians conversion
#endif

#define MM_TO_M                 0.001                   ///< Millimeters to meters conversion
#define M_TO_MM                 1000.0                  ///< Meters to millimeters conversion



// =============================================================================
// DEBUG AND DEVELOPMENT
// =============================================================================

/**
 * @brief Debug Configuration
 */
#define DEBUG_ENABLED           1       ///< Enable debug output (0=off, 1=on)
#define DEBUG_MOTOR_CONTROL     1       ///< Debug motor control (0=off, 1=on)
#define DEBUG_ODOMETRY          1       ///< Debug odometry calculations (0=off, 1=on)
#define DEBUG_SERIAL_COMM       1       ///< Debug serial communication (0=off, 1=on)

/**
 * @brief Development Flags
 */
#define SIMULATE_ENCODERS       0       ///< Simulate encoder input for testing (0=off, 1=on)
#define ENABLE_MOTOR_SAFETY     1       ///< Enable motor safety checks (0=off, 1=on)
#define ENABLE_BATTERY_MONITOR  1       ///< Enable battery monitoring (0=off, 1=on)



// =============================================================================
// ROS STATE MACHINE CONFIGURATION
// =============================================================================

/**
 * @brief ROS Agent Connection States
 */
typedef enum {
    WAITING_AGENT = 0,      ///< Waiting for micro-ROS agent connection
    AGENT_AVAILABLE,        ///< Agent is available, attempting to create entities
    AGENT_CONNECTED,        ///< Agent connected and entities created successfully
    AGENT_DISCONNECTED      ///< Agent disconnected, cleanup needed
} RosAgentState_t;

/**
 * @brief ROS State Machine Timing Configuration
 */
#define ROS_TASK_FREQUENCY_HZ   50      ///< ROS task frequency (Hz)
#define ROS_TASK_PERIOD_MS      20      ///< ROS task period (ms) = 1000/FREQUENCY
#define AGENT_PING_INTERVAL_MS  100     ///< Interval to ping agent when waiting (ms)
#define AGENT_HEALTH_CHECK_MS   200     ///< Interval to check connection health (ms)
#define AGENT_PING_TIMEOUT_MS   50      ///< Timeout for agent ping when waiting (ms)

// Health check while CONNECTED: rmw_uros_ping_agent BLOCKS the whole ROS loop
// (including the BackState publisher) for up to timeout*attempts, so keep a single
// short attempt and tolerate consecutive misses instead. Old config (100ms x 10)
// stalled the loop up to 1s per check whenever the agent replied slowly, which
// collapsed the published state rate (observed on the arms: ~15-20 Hz).
#define AGENT_HEALTH_TIMEOUT_MS 25      ///< Timeout for health check ping (ms)
#define AGENT_PING_ATTEMPTS     1       ///< Number of ping attempts when waiting
#define AGENT_HEALTH_ATTEMPTS   1       ///< Single attempt; misses tolerated below
// Consecutive failed health pings before declaring AGENT_DISCONNECTED
// (10 x 200ms cycle = ~2s of unresponsive agent, same effective latency as before).
#define AGENT_HEALTH_MAX_MISSES 10

// If the agent only shows up after the board has been waiting this long, the
// USB-CDC FIFOs / XRCE framing on both ends are full of hours of stale ping
// traffic and sessions establish degraded (observed: state rate stuck at
// ~15-20 Hz until a power cycle). Reboot instead of connecting: re-enumeration
// recreates the known-good fresh-boot condition, and the agent (respawn=True)
// reconnects to a seconds-old board. Normal bringup never waits this long.
#define AGENT_LONG_WAIT_REBOOT_MS 120000

/**
 * @brief Utility macro for executing code at specific intervals
 * @param interval_ms Interval in milliseconds
 * @param code Code to execute
 */
#define EXECUTE_EVERY_N_MS(interval_ms, code) \
    do { \
        static uint32_t last_execution = 0; \
        uint32_t current_time = millis(); \
        if (current_time - last_execution >= interval_ms) { \
            last_execution = current_time; \
            code; \
        } \
    } while(0)

/**
 * @brief Run @p code on a fixed cadence of @p interval_ms (drift-free).
 *
 * Unlike EXECUTE_EVERY_N_MS, which resets its deadline to "now" (so the task's
 * own run-time stretches the true period), this advances the deadline by exactly
 * @p interval_ms, keeping the long-run average rate exact. If the loop falls more
 * than one interval behind (e.g. after an agent stall) the deadline is clamped
 * forward so it never fires a catch-up burst. Signed comparison tolerates the
 * millis() wraparound.
 */
#define EXECUTE_AT_RATE_MS(interval_ms, code) \
    do { \
        static uint32_t next_deadline_ms = 0; \
        uint32_t now_ms = millis(); \
        if ((int32_t)(now_ms - next_deadline_ms) >= 0) { \
            next_deadline_ms += (uint32_t)(interval_ms); \
            if ((int32_t)(now_ms - next_deadline_ms) >= 0) { \
                next_deadline_ms = now_ms + (uint32_t)(interval_ms); \
            } \
            code; \
        } \
    } while(0)
