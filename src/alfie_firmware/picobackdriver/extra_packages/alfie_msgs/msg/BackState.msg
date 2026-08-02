uint8 board_temp
bool limit_switch_triggered

# Linear actuator state feedback
# in meters
float32 command_position

# in meters per second
float32 command_velocity

# in meters per second per second
float32 command_acceleration

bool is_moving
float32 current_position
float32 current_velocity
float32 current_acceleration
int64 pulse_count
uint8 pwm_output

bool is_calibrated

# Calibration state constants
# CALIBRATION_IDLE = 0
# CALIBRATION_LOWERING = 1
# CALIBRATION_RAISING = 2
# CALIBRATION_COMPLETE = 3
# CALIBRATION_ERROR = 4

# Current calibration state (see constants above)
uint8 calibration_status

# Fault state
# error_code mirrors the firmware's ERROR_* codes in config.h. It latches until
# the fault clears; see handleMotorSafety() for the recovery rules (a stall
# self-clears for the first few attempts, then requires a new target position).
uint8 ERROR_NONE=0
uint8 ERROR_INVALID_COMMAND=1
uint8 ERROR_MOTOR_FAULT=2
uint8 ERROR_ENCODER_FAULT=3
uint8 ERROR_LOW_BATTERY=4
uint8 ERROR_OVERTEMPERATURE=5
uint8 ERROR_COMMUNICATION=6
uint8 ERROR_MOTOR_STALL=7
uint8 ERROR_DRIVER_FAULT=8
# A block and an overload look identical to the detector (duty commanded, no
# encoder motion), so they are separated by whether motion was achieved earlier
# in the same move attempt. Treat these as a strong hint, not a proof: a block
# sitting at the start position reads as an overload, and a load that only
# becomes too heavy partway through reads as a block. The giveaway is position -
# a real obstruction stalls at the SAME current_position across retries, an
# overload stalls wherever it started.
#
# BLOCKED hard-latches immediately and will NOT retry on its own; whatever
# stopped a moving actuator might be a hand. LOAD_EXCEEDED retries up to three
# times, three seconds apart, then hard-latches. Either releases when a
# materially different position is commanded - sending the back DOWN is the
# escape from an obstruction.
uint8 ERROR_MOTOR_BLOCKED=9
uint8 ERROR_LOAD_EXCEEDED=10
# Moving far faster than commanded, or against the command. The signature of a
# sign inversion - swapped motor leads, swapped encoder channels, flipped PH
# polarity, or a floating PMODE inverting the down direction - each of which
# turns the velocity loop into positive feedback on a gravity-loaded actuator.
#
# This NEVER clears on its own; only a reset does. Retrying cannot fix miswiring,
# and the failure it guards against is the back driving itself into a hard stop.
# It also outranks every other code, because once the hardware is not what the
# firmware believes, any other diagnosis derived from that belief is suspect.
uint8 ERROR_MOTOR_RUNAWAY=11
uint8 error_code

# NOTE: current_limiting was removed here. The DRV8876's internal current
# regulation is disabled by design (IPROPI strapped to GND), so there is no
# chopping to report and no current measurement anywhere in the system. Overload
# now surfaces only through error_code / stall_count / stall_position.

# Position at which the last stall latched, in meters. Holds its value after the
# fault clears, so it can be compared against the next one.
#
# This is the discriminator that error_code can only guess at. A real obstruction
# stalls at the SAME stall_position across retries; an overload stalls wherever
# it happened to start. Two stalls at the same value is strong evidence of
# something physically in the way, whichever code was reported.
float32 stall_position

# Consecutive stall attempts. Cleared by any real motion, so a non-zero value
# means the actuator has been trying and failing. Compare against the firmware's
# STALL_MAX_RETRIES (3) to know how much patience is left.
uint8 stall_count

# True when the fault will NOT clear on its own and needs a materially different
# target position to release. False means it is still in the self-clearing retry
# window (~3 s per attempt) - useful for deciding whether to surface a prompt to
# a human or just wait. ERROR_MOTOR_BLOCKED latches hard on the first occurrence;
# ERROR_LOAD_EXCEEDED only after it has used up its retries.
bool fault_latched

# BNO085 IMU telemetry (torso/base orientation)
# quaternion orientation, angular velocity (rad/s), linear acceleration (m/s^2 incl. gravity)
ImuState imu
