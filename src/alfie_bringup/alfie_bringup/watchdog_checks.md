# Watchdog Health Checks

The watchdog health checks are integrated into the Master Status Node (`master_status.py`) to monitor the health of critical subsystems. The health check classes are defined in `watchdog_checks.py` and use the existing subscriptions in `master_status.py` to minimize load on microcontrollers and micro-ROS.

## Architecture

The watchdog functionality is split across two files:

- **`watchdog_checks.py`** - Contains all health check classes and configuration
- **`master_status.py`** - Integrates health checks using its existing topic subscriptions

This design allows the Master Status Node to:
1. Subscribe to GDB and Jetson state topics once (not duplicated)
2. Update health monitors in the same callbacks that process state data
3. Run health checks at 1Hz via a separate timer

## Watched Properties and Allowable Values

### GDB (Servo Driver Board) Monitoring

| Property | Check Type | Expected/Threshold | Notes |
|----------|------------|-------------------|-------|
| **GDB0 Rate** | Rate | 100Hz ± 10Hz | Right arm + Head driver |
| **GDB1 Rate** | Rate | 100Hz ± 10Hz | Left arm driver |
| **GDB0 Voltage** | Voltage | 12.0V ± 0.2V | All servos monitored |
| **GDB1 Voltage** | Voltage | 12.0V ± 0.2V | All servos monitored |
| **GDB0 Timing** | Timing | ≤ 10ms total | Sum of servo diagnostic durations |
| **GDB1 Timing** | Timing | ≤ 10ms total | Sum of servo diagnostic durations |
| **GDB0 IMU Timing** | Timing | ≤ 10ms | IMU update duration (separate) |
| **GDB1 IMU Timing** | Timing | ≤ 10ms | IMU update duration (separate) |
| **GDB0 Board Temp** | Temperature | < 45°C | Driver board temperature |
| **GDB1 Board Temp** | Temperature | < 45°C | Driver board temperature |

### Servo Health Monitoring

Each servo on both GDBs is monitored for the following conditions:

| Property | Warning Threshold | Notes |
|----------|-------------------|-------|
| **Temperature** | ≥ 50°C | Per-servo temperature |
| **Load** | ≥ 100% | Max torque percentage |
| **Status Code** | ≠ 0 | Any non-zero status is an error |
| **Current Draw** | ≥ 3000mA (3A) | Current consumption |

#### GDB0 Servos (Right Arm + Head)
| Index | Servo Name |
|-------|------------|
| 0 | R Shoulder Yaw |
| 1 | R Shoulder1 Pitch |
| 2 | R Shoulder2 Pitch (derived) |
| 3 | R Elbow Pitch |
| 4 | R Wrist Pitch |
| 5 | R Wrist Roll |
| 6 | R Hand |
| 7 | Head Yaw |
| 8 | Head Pitch |
| 9 | Head Roll |

#### GDB1 Servos (Left Arm)
| Index | Servo Name |
|-------|------------|
| 0 | L Shoulder Yaw |
| 1 | L Shoulder1 Pitch |
| 2 | L Shoulder2 Pitch (derived) |
| 3 | L Elbow Pitch |
| 4 | L Wrist Pitch |
| 5 | L Wrist Roll |
| 6 | L Hand |

### Jetson Health Monitoring

#### Temperature Monitors

| Property | Warning | Critical | Notes |
|----------|---------|----------|-------|
| **CPU Temp** | ≥ 70°C | ≥ 85°C | Jetson CPU temperature |
| **GPU Temp** | ≥ 70°C | ≥ 85°C | Jetson GPU temperature |
| **Thermal Junction** | ≥ 85°C | ≥ 95°C | Hottest thermal zone |

#### Resource Monitors

| Property | Warning | Critical | Notes |
|----------|---------|----------|-------|
| **RAM Usage** | ≥ 85% | ≥ 95% | System memory usage |
| **Swap Usage** | ≥ 50% | ≥ 80% | Swap memory usage |
| **Disk Usage** | ≥ 80% | ≥ 95% | Root filesystem usage |
| **CPU Load** | ≥ 90% | ≥ 98% | Overall CPU utilization |

#### WiFi Monitor

| Property | Threshold | Notes |
|----------|-----------|-------|
| **Connection** | Must be connected | Reports CRITICAL if disconnected |
| **Signal Strength** | ≥ -75 dBm | Reports WARNING if signal is weaker |

## Subscribed Topics

All topics are subscribed by the Master Status Node (`master_status.py`), which shares the data with the watchdog health checks:

| Topic | Message Type | QoS | Used By |
|-------|--------------|-----|---------|
| `low/gdb0state` | `alfie_msgs/GDBState` | BEST_EFFORT | Status + Watchdog |
| `low/gdb1state` | `alfie_msgs/GDBState` | BEST_EFFORT | Status + Watchdog |
| `low/jetsonstate` | `alfie_msgs/JetsonState` | RELIABLE | Watchdog only |
| `low/backstate` | `alfie_msgs/BackState` | BEST_EFFORT | Status only |
| `oak/imu/data` | `sensor_msgs/Imu` | BEST_EFFORT | Status only |

## Check Rate

The watchdog performs health checks at **1 Hz** (once per second), while the Master Status Node publishes robot state at **100 Hz**.

## Diagnostic Timing Fields

The following GDB diagnostic timing fields are summed and compared against the maximum threshold:

- `pollservostatusduration` - Time to poll servo status
- `updateservoidleduration` - Time to update servos in idle mode
- `updateservoactiveduration` - Time to update servos in active mode
- `imuupdateduration` - Time to read IMU data

All timing values are reported in **milliseconds**.

## Health Check Types

### RateMonitor
Monitors message reception rate for a topic. Reports error if rate deviates from expected by more than the tolerance.

### ServoVoltageMonitor
Monitors voltage levels for ALL servos on a GDB (reported in 0.1V units, scaled to volts). Reports error if any servo's voltage deviates from expected by more than the tolerance.

### DiagnosticTimingMonitor
Monitors GDB diagnostic timing values. Reports error if servo timing total exceeds the maximum threshold, or if IMU update duration exceeds 10ms (checked separately).

### TemperatureMonitor
Monitors temperature values. Reports WARNING if above warn threshold, CRITICAL if above critical threshold.

### ThresholdMonitor
Monitors percentage values (0-100%). Reports WARNING if above warn threshold, CRITICAL if above critical threshold.

### WifiMonitor
Monitors WiFi connection status and signal strength. Reports CRITICAL if disconnected, WARNING if signal strength is below minimum.

### ServoMonitor
Monitors individual servo health including temperature, status codes, and current draw. Reports warnings for any servos exceeding thresholds.

## Module Structure

### `watchdog_checks.py`

Contains:
- `HealthCheck` - Abstract base class for all health checks
- `RateMonitor` - Message rate monitoring
- `ServoVoltageMonitor` - Per-servo voltage monitoring
- `DiagnosticTimingMonitor` - GDB timing diagnostics
- `TemperatureMonitor` - Temperature threshold monitoring
- `ThresholdMonitor` - Percentage threshold monitoring
- `WifiMonitor` - WiFi connection and signal monitoring
- `ServoMonitor` - Per-servo health monitoring
- `GDB0_SERVO_NAMES` / `GDB1_SERVO_NAMES` - Servo name configurations
- `create_health_checks()` - Factory function to create all configured health checks

### `master_status.py`

Integrates watchdog by:
1. Importing `create_health_checks()` and `HealthCheck` from `watchdog_checks.py`
2. Initializing health checks in `__init__`
3. Updating health checks in GDB and Jetson callbacks
4. Running `run_health_checks()` at 1Hz via timer

## Usage

The watchdog is now integrated into the Master Status Node:

```bash
ros2 run alfie_bringup master_status
```

All health check failures are logged as errors to the ROS2 logging system.

## Deprecated

The standalone `master_watchdog.py` node is deprecated. Its functionality has been merged into `master_status.py` to reduce duplicate topic subscriptions and system load.
