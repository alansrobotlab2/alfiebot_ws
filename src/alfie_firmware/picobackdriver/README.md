# Pico Back Driver Board

This project implements a Raspberry Pi Pico-based driver board for a linear actuator control system using micro-ROS for ROS2 communication.

## Hardware Overview

### Main Components
- **Microcontroller**: Raspberry Pi Pico (RP2040)
- **Motor Driver**: TB6612FNG dual H-bridge (using one channel)
- **Motor**: Linear actuator with encoder feedback
- **Communication**: micro-ROS over USB Serial
- **Status Indication**: 
  - Built-in LED (GPIO 25)
  - WS2812B RGB LED (GPIO 16)

### Pin Assignments

#### TB6612FNG Motor Driver Control
- `GPIO 2`: Motor PWM (PWMA - speed control)
- `GPIO 3`: Motor Direction 1 (AIN1)
- `GPIO 4`: Motor Direction 2 (AIN2)
- `GPIO 5`: Standby Pin (STBY - HIGH=active, LOW=standby)
- `GPIO 15`: VCC Power (logic power supply - must be HIGH)

#### Encoder Feedback
- `GPIO 28`: Encoder A
- `GPIO 29`: Encoder B

#### Limit Switches
- `GPIO 10`: Lower Limit Switch (active high, goes LOW when triggered at base position)

#### Status LEDs
- `GPIO 25`: Built-in LED (status blink pattern)
- `GPIO 16`: WS2812B RGB LED (visual state feedback)

### TB6612FNG Motor Driver Operation

The TB6612FNG is a dual H-bridge motor driver. This board uses one channel (Motor A) with the following control logic:

| AIN1 | AIN2 | PWM | Function |
|------|------|-----|----------|
| HIGH | LOW  | PWM | Forward (CW) |
| LOW  | HIGH | PWM | Reverse (CCW) |
| LOW  | LOW  | 0   | Brake (short brake) |
| HIGH | HIGH | 0   | Brake (short brake) |

**Important Notes:**
- **STBY pin** must be HIGH for normal operation. Setting it LOW puts the driver in standby mode (all outputs disabled)
- **VCC pin** (GPIO 15) must be set HIGH to power the driver's logic
- Brake mode (both direction pins same state) provides active braking

## WS2812B RGB LED Status Codes

The RGB LED provides visual feedback about the system state:

| Color | State | Description |
|-------|-------|-------------|
| **Blue** (dim) | Waiting for ROS Agent | System is initialized, waiting for micro-ROS agent connection |
| **Yellow** | Creating ROS Entities | ROS agent connected, creating publishers/subscribers |
| **Green** (breathing) | Operational & Idle | Fully operational, connected to ROS, actuator idle |
| **Cyan** | Moving | Actuator is actively moving |
| **Orange** | Agent Disconnected | ROS agent connection lost |
| **Red** (pulsing) | Error/Fault | Motor fault or system error detected |
| **White** (dim) | Unknown State | Undefined system state |

## Development Environment

This project is developed using:
- **VS Code** with PlatformIO extension
- **Framework**: Arduino framework for RP2040
- **Communication**: micro-ROS for ROS2 integration

## Features

### Dual-Core Architecture
- **Core 0**: Peripheral management (motor control, encoders, sensors, LED updates)
- **Core 1**: ROS2 communications and high-level control

### Motor Control
- Velocity PID controller
- Acceleration limiting
- Position/velocity/acceleration control modes
- Encoder feedback for closed-loop control

### Safety Features
- Motor fault detection
- Emergency stop capability
- Timeout-based safety checks
- Visual status indication via RGB LED

## Building and Flashing

### Prerequisites
1. Install PlatformIO: https://docs.platformio.org/en/latest/core/installation/
2. Configure udev rules (Linux): https://docs.platformio.org/en/latest/core/installation/udev-rules.html

### Build Commands
```bash
# Build the project
pio run

# Upload to Pico
pio run --target upload

# Monitor serial output
pio device monitor
```

## ROS2 Integration

This firmware uses micro-ROS to communicate with ROS2. The following custom messages are used:

- `alfie_msgs/BackCmd`: Actuator command message (position, velocity, acceleration)
- `alfie_msgs/BackState`: Actuator state message (current position, velocity, pulses, etc.)

### Topics
- Subscriber: `/back_cmd` - Receives actuator commands
- Publisher: `/back_state` - Publishes actuator state at 100Hz

## Configuration

All configuration parameters are defined in `include/config.h`:
- Serial baud rate: 1,500,000 bps
- Control loop frequency: 100Hz (10ms period)
- PWM frequency: 512Hz
- Motor/encoder pins
- PID gains
- Safety limits

## Code Structure

```
├── include/
│   ├── config.h          # Hardware and control configuration
│   └── driverboard.h     # Main driver board class definition
├── lib/
│   ├── motor_control/    # Motor control and peripheral management
│   ├── ros_interface/    # micro-ROS communication
│   └── ws2812/           # WS2812B RGB LED driver (PIO-based)
├── src/
│   └── main.cpp          # Main application entry point
└── extra_packages/
    └── alfie_msgs/       # Custom ROS2 messages
```

## Coding Standards

1. Use descriptive variable and function names
2. Include comments to explain complex logic
3. Follow consistent indentation and formatting
4. Adhere to Arduino framework conventions for RP2040
5. Modularize code into functions for better readability
6. Use constants and macros for fixed values
7. **No serial print statements** (serial port is used for micro-ROS)
8. Test on actual hardware
9. Keep README up to date with code changes

## Additional Resources

- Arduino-Pico Documentation: https://arduino-pico.readthedocs.io/en/latest/platformio.html
- PlatformIO udev Rules: https://docs.platformio.org/en/latest/core/installation/udev-rules.html

## License

[Add your license information here]

## Authors

Alfie Bot Project
