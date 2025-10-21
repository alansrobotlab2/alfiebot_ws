# Alfie Firmware

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-orange)](https://platformio.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)

Dual ESP32-based firmware for Alfiebot's driver boards, providing real-time control of servos, motors, IMU sensors, and ROS2 integration via micro-ROS.

## 📋 Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Hardware Configuration](#hardware-configuration)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Building and Flashing](#building-and-flashing)
- [ROS2 Integration](#ros2-integration)
- [Configuration](#configuration)
- [Development](#development)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## 🎯 Overview

Alfiebot uses two modified Waveshare General Driver Boards (ESP32-WROVER-based) to manage distributed hardware control:

- **Driver Board 0 (Top)**: Right arm servos, head servos, and wheel motor drivers
- **Driver Board 1 (Bottom)**: Left arm servos, eye LED drivers, and shoulder limit switches

Each board runs identical firmware with board-specific functionality determined by compile-time configuration. The firmware leverages FreeRTOS for dual-core task distribution and micro-ROS for seamless ROS2 communication.

### Key Features

- ⚡ **100Hz Control Loop**: Precise real-time servo and motor control
- 🔄 **Dual-Core Architecture**: Hardware tasks on Core 0, ROS2 on Core 1
- 🤖 **micro-ROS Integration**: Native ROS2 communication over serial
- 📊 **IMU Support**: QMI8658 accelerometer/gyroscope + AK09918 magnetometer
- 🔧 **Smart Servo Protocol**: Feetech SCS/SMS serial bus servos
- 🛡️ **Watchdog Protection**: Automatic safety shutdowns on communication loss
- 🔌 **Hot-Reconnect**: Automatic ROS agent reconnection

## 🏗️ Architecture

### FreeRTOS Task Distribution

```
Core 0 (Hardware Interface Task) - 100Hz
├── Servo status polling
├── Servo command execution
├── Motor encoder processing
├── IMU data acquisition
└── Shoulder limit switch monitoring

Core 1 (ROS Task) - 100Hz
├── ROS agent connection management
├── State message publishing
├── Command message subscription
└── Service request handling
```

### ROS2 Communication

**Driver Board 0 (Top Board - Right Arm, Head, Wheels):**
- Published: `/alfie/gdb0state` - Board status (100Hz)
- Subscribed: `/alfie/gdb0cmd` - Control commands
- Service: `/alfie/gdb0servoservice` - Servo memory read/write operations

**Driver Board 1 (Bottom Board - Left Arm, Eyes, Shoulder):**
- Published: `/alfie/gdb1state` - Board status (100Hz)
- Subscribed: `/alfie/gdb1cmd` - Control commands
- Service: `/alfie/gdb1servoservice` - Servo memory read/write operations

## 🔧 Hardware Configuration

### Driver Board 0 (Top Board)
- **Device**: `/dev/ttyUSB0`
- **Servos**: 10 units (right arm + head)
- **Motors**: 2x TB6612 wheel drivers with encoders
- **PWM Frequency**: 512 Hz
- **Special Features**: Wheel encoder interrupts

### Driver Board 1 (Bottom Board)
- **Device**: `/dev/ttyUSB1`
- **Servos**: 7 units (left arm)
- **Lighting**: Eye LED drivers
- **PWM Frequency**: 8268 Hz
- **Special Features**: Shoulder limit switch monitoring

### Pin Assignments

#### Driver Board 0 (Top Board)

**Servo UART:**
- RX: GPIO 18
- TX: GPIO 19
- Baudrate: 1000000

**Motor A (TB6612):**
- PWMA: GPIO 25
- AIN1: GPIO 21
- AIN2: GPIO 17

**Motor B (TB6612):**
- PWMB: GPIO 26
- BIN1: GPIO 22
- BIN2: GPIO 23

**Encoder A:**
- AENCA: GPIO 35
- AENCB: GPIO 34

**Encoder B:**
- BENCA: GPIO 27
- BENCB: GPIO 16

#### Driver Board 1 (Bottom Board)

**Servo UART:**
- RX: GPIO 18
- TX: GPIO 19
- Baudrate: 1000000

**Eye Light Driver Left (TB6612):**
- Eye A PWM: GPIO 25
- Eye A IN1: GPIO 21
- Eye A IN2: GPIO 17

**Eye Light Driver Right (TB6612):**
- Eye B PWM: GPIO 26
- Eye B IN1: GPIO 22
- Eye B IN2: GPIO 23

**Shoulder Limit Switch:**
- GPIO 34 (input only)

## 📦 Prerequisites

### Required Software

- **Visual Studio Code** with PlatformIO extension
- **ROS2 Humble** (on host computer)
- **micro-ROS Agent** (for ROS2 communication)
- **Python 3.8+**

### System Requirements

- Ubuntu 20.04/22.04 (recommended for ROS2)
- USB ports for ESP32 programming
- Stable power supply for driver boards

## 🚀 Installation

### 1. Clone the Repository

```bash
cd ~/alfiebot_ws/src
git clone <repository-url> alfie_firmware
```

### 2. Install PlatformIO

**Via VS Code:**
1. Open VS Code
2. Go to Extensions (Ctrl+Shift+X)
3. Search for "PlatformIO IDE"
4. Click Install

**Via Command Line:**
```bash
pip install platformio
```

### 3. Install ROS2 Dependencies

```bash
cd ~/alfiebot_ws
source /opt/ros/humble/setup.bash

# Install custom message package
cd src/alfie_firmware/extra_packages
colcon build --packages-select alfie_msgs
source install/setup.bash
```

### 4. Install micro-ROS Agent

```bash
# Install from snap (recommended)
sudo snap install micro-ros-agent

# Or build from source
cd ~/alfiebot_ws/src
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
cd ~/alfiebot_ws
colcon build --packages-select micro_ros_agent
```

## 🔨 Building and Flashing

### Compile and Use the Firmware

#### 1. Open the Project in VS Code

To compile and use the firmware, you need to open the project folder in VS Code with PlatformIO:

```bash
cd ~/alfiebot_ws/src/alfie_firmware
code .
```

**Important:** Make sure to open the folder `alfiebot_ws/src/alfie_firmware` directly (not the parent workspace folder). This folder contains the `platformio.ini` file which PlatformIO needs to identify this as a PlatformIO project.

#### 2. PlatformIO Initialization

When you first open the folder:
- PlatformIO will automatically detect the `platformio.ini` configuration file
- The PlatformIO extension will initialize the project (this may take a minute)
- You'll see the PlatformIO icon appear in the left sidebar
- PlatformIO will download the ESP32 platform, toolchain, and dependencies automatically

**First-Time Setup:**
- The first initialization may take 5-10 minutes as PlatformIO downloads:
  - ESP32 platform and build tools
  - Arduino framework for ESP32
  - micro-ROS libraries and dependencies
  - All required libraries specified in `platformio.ini`
- You can monitor progress in the PlatformIO terminal at the bottom of VS Code

Once initialization is complete, you're ready to build and flash the firmware!

### Configure for Target Board

1. Open `src/config.h`
2. Set the `DRIVERBOARD` define:
   ```cpp
   #define DRIVERBOARD 0  // For top board (/dev/ttyUSB0)
   // or
   #define DRIVERBOARD 1  // For bottom board (/dev/ttyUSB1)
   ```

### Build Firmware

**Via VS Code:**
- Click PlatformIO icon in sidebar
- Project Tasks → esp-wrover-kit → Build

**Via Command Line:**
```bash
pio run
```

### Upload to Board

**Via VS Code:**
- Click PlatformIO icon in sidebar
- Project Tasks → esp-wrover-kit → Upload

**Via Command Line:**
```bash
# Make sure the correct USB port is connected
pio run --target upload
```

### Complete Setup for Both Boards

```bash
# Flash Driver Board 0
# 1. Connect /dev/ttyUSB0
# 2. Edit config.h: #define DRIVERBOARD 0
pio run --target upload

# Flash Driver Board 1
# 1. Connect /dev/ttyUSB1
# 2. Edit config.h: #define DRIVERBOARD 1
pio run --target upload
```

## 🌐 ROS2 Integration

### Start micro-ROS Agent

The agent bridges serial communication to ROS2 DDS network.

**Single board (for testing):**
```bash
ros2 run micro-ros-agent serial --dev /dev/ttyUSB0 -b 921600
```

**Both boards (production):**
```bash
# Terminal 1
ros2 run micro-ros-agent serial --dev /dev/ttyUSB0 -b 921600

# Terminal 2
ros2 run micro-ros-agent serial --dev /dev/ttyUSB1 -b 921600
```

**Using tmux (recommended):**
```bash
# Start both agents in split terminal
tmux new-session -d -s alfie_agents
tmux send-keys -t ros2 run alfie_agents "micro-ros-agent serial --dev /dev/ttyUSB0 -b 921600" Enter
tmux split-window -t alfie_agents -v
tmux send-keys -t ros2 run alfie_agents "micro-ros-agent serial --dev /dev/ttyUSB1 -b 921600" Enter
tmux attach -t alfie_agents
```

### Verify ROS2 Communication

```bash
# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source ~/alfiebot_ws/install/setup.bash

# List topics
ros2 topic list | grep alfie

# Monitor board state
ros2 topic echo /alfie/gdb0state
ros2 topic echo /alfie/gdb1state

# Check service availability
ros2 service list | grep alfie
```

### Send Commands

```bash
# Example: Command a servo (requires custom message structure)
ros2 topic pub /alfie/gdb0cmd alfie_msgs/msg/GDBCmd "..."

# Call servo service
ros2 service call /alfie/gdb0servoservice alfie_msgs/srv/GDBServoService "..."
```

## ⚙️ Configuration

### Adjusting Parameters

Edit `src/config.h`:

```cpp
#define NUMSERVOS 10     // Number of servos per board
```

### Changing Serial Baudrate

Both in `platformio.ini` and `src/config.h`:

```ini
; platformio.ini
monitor_speed = 921600
```

```cpp
// src/config.h
#define BAUDRATE 921600
```

### Watchdog Timeout

```cpp
#define WATCHDOG_TIMEOUT_MS 100  // Safety shutdown after 100ms no commands
```

## 🛠️ Development

### Clean micro-ROS Build

When you modify ROS message definitions:

**Via VS Code:**
1. Click PlatformIO icon
2. Miscellaneous → PlatformIO Core CLI
3. Run: `pio run --target clean_microros`

**Via Command Line:**
```bash
pio run --target clean_microros
pio run
```

### Access PlatformIO Home

```bash
# Forward port 8008 if using remote development
pio home
```

### Project Structure

```
alfie_firmware/
├── src/                    # Main source code
│   ├── main.cpp           # Entry point & FreeRTOS tasks
│   ├── config.h           # Board configuration
│   ├── ros_control.cpp    # ROS2 integration
│   ├── servo_control.cpp  # Servo management
│   ├── motor_control.cpp  # Motor driver control
│   ├── imu_control.cpp    # IMU sensor interface
│   └── imu/               # IMU driver libraries
│       ├── QMI8658.cpp    # Accelerometer/gyroscope
│       └── AK09918.cpp    # Magnetometer
├── extra_packages/
│   └── alfie_msgs/        # Custom ROS2 messages
├── platformio.ini         # PlatformIO configuration
└── README.md
```

### Adding Custom Functionality

1. **New Peripherals**: Add header/source in `src/`
2. **ROS Messages**: Modify `extra_packages/alfie_msgs/msg/`
3. **Board-Specific Code**: Use `#if DRIVERBOARD == X` directives
4. **Task Modifications**: Edit `vHardwareInterfaceTask()` or `vROSTask()` in `main.cpp`

## 🐛 Troubleshooting

### Boards Not Detected

```bash
# Check USB connections
ls -l /dev/ttyUSB*

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in

# Check permissions
sudo chmod 666 /dev/ttyUSB*
```

### Compilation Errors

```bash
# Clean and rebuild
pio run --target clean
pio run --target clean_microros
pio run
```

### ROS Agent Connection Issues

1. Verify baudrate matches (921600)
2. Check correct USB port
3. Ensure no other process is using the serial port:
   ```bash
   sudo lsof | grep ttyUSB
   ```
4. Monitor board output:
   ```bash
   pio device monitor -b 921600
   ```

### Servo Communication Failures

- Check servo power supply
- Verify TX/RX wiring
- Test servo baudrate (default: 1000000)
- Use servo debugging tools in `scservo/` library

### Reset Driver Boards

```bash
# Python script
python3 reset_driver_boards.py

# Or shell script
./reset_driver_boards.sh
```

### Common Error Messages

| Error | Solution |
|-------|----------|
| `Agent not available` | Start micro-ros-agent on correct port |
| `Watchdog timeout` | Check command publishing frequency (>10Hz) |
| `Servo read failed` | Verify servo power and ID configuration |
| `IMU init failed` | Check I2C connections and pull-up resistors |

## 📚 Additional Resources

- [PlatformIO Documentation](https://docs.platformio.org/)
- [micro-ROS Documentation](https://micro.ros.org/docs/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ESP32 Arduino Core](https://docs.espressif.com/projects/arduino-esp32/)
- [FreeRTOS Documentation](https://www.freertos.org/documentation/)

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

---

**Maintainer:** Alan's Robot Lab (alansrobotlab@gmail.com)  
**Last Updated:** October 2025