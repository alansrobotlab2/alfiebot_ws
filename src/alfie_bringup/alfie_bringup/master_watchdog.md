# Master Watchdog Node

The Master Watchdog Node monitors the health of critical subsystems by subscribing to state topics and tracking message reception timing, rates, and hardware values.

## Watched Properties and Allowable Values

### GDB (Servo Driver Board) Monitoring

| Property | Check Type | Expected/Threshold | Notes |
|----------|------------|-------------------|-------|
| **GDB0 Rate** | Rate | 100Hz ± 10Hz | Right arm + Head driver |
| **GDB1 Rate** | Rate | 100Hz ± 10Hz | Left arm driver |
| **GDB0 Voltage** | Voltage | 12.0V ± 0.2V | From first servo |
| **GDB1 Voltage** | Voltage | 12.0V ± 0.2V | From first servo |
| **GDB0 Timing** | Timing | ≤ 10ms total | Sum of all diagnostic durations |
| **GDB1 Timing** | Timing | ≤ 10ms total | Sum of all diagnostic durations |
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

| Topic | Message Type | QoS |
|-------|--------------|-----|
| `/alfie/low/gdb0state` | `alfie_msgs/GDBState` | BEST_EFFORT |
| `/alfie/low/gdb1state` | `alfie_msgs/GDBState` | BEST_EFFORT |
| `/alfie/low/jetsonstate` | `alfie_msgs/JetsonState` | RELIABLE |

## Check Rate

The watchdog performs health checks at **1 Hz** (once per second).

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

### VoltageMonitor
Monitors servo voltage levels (reported in 0.1V units, scaled to volts). Reports error if voltage deviates from expected by more than the tolerance.

### DiagnosticTimingMonitor
Monitors GDB diagnostic timing values. Reports error if total timing exceeds the maximum threshold.

### TemperatureMonitor
Monitors temperature values. Reports WARNING if above warn threshold, CRITICAL if above critical threshold.

### ThresholdMonitor
Monitors percentage values (0-100%). Reports WARNING if above warn threshold, CRITICAL if above critical threshold.

### WifiMonitor
Monitors WiFi connection status and signal strength. Reports CRITICAL if disconnected, WARNING if signal strength is below minimum.

### ServoMonitor
Monitors individual servo health including temperature, load, status codes, and current draw. Reports warnings for any servos exceeding thresholds.

## Usage

The node is launched as part of the bringup system:

```bash
ros2 run alfie_bringup master_watchdog
```

All health check failures are logged as errors to the ROS2 logging system.
