# 🤖 Alfiebot Workspace

<div align="center">

**A ROS2 Jazzy workspace for an intelligent, interactive humanoid robot**

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-Apache%202.0-brightgreen.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Jetson-76B900.svg)](https://developer.nvidia.com/embedded/jetson)

</div>

---

## 📋 Overview

Alfiebot is a conversational humanoid robot built on ROS2 Jazzy, featuring voice interaction, LLM-powered intelligence, and expressive servo-controlled movements. The platform integrates automatic speech recognition (ASR), large language model (LLM) processing, text-to-speech (TTS), and sophisticated motor control through micro-ROS firmware.

### Key Features

- 🎤 **Voice Interaction** - Real-time speech recognition and natural language understanding
- 🧠 **LLM Integration** - Powered by modern language models for intelligent conversations
- 🔊 **Text-to-Speech** - Natural voice synthesis for robot responses
- ⚙️ **Motor Control** - Servo-based articulation with Waveshare serial bus servos
- 📡 **Micro-ROS** - Efficient real-time communication with embedded firmware
- 🌐 **Network Management** - Built-in WiFi hotspot capabilities via Alfienet

---

## 🗂️ Package Structure

```
alfiebot_ws/
├── src/
│   ├── alfie_agent/       # High-level behavioral control and orchestration
│   ├── alfie_asr/         # Automatic speech recognition node
│   ├── alfie_back/        # Backend services and utilities
│   ├── alfie_bringup/     # Launch files and system startup configuration
│   ├── alfie_firmware/    # Micro-ROS firmware for embedded systems
│   ├── alfie_llm/         # Large language model integration
│   ├── alfie_mic/         # Microphone interface and audio capture
│   ├── alfie_msgs/        # Custom ROS2 message and service definitions
│   ├── alfie_tools/       # Development and debugging tools (servotool, etc.)
│   └── alfie_tts/         # Text-to-speech synthesis node
└── alfienet/              # WiFi hotspot and network management scripts
```

---

## 🚀 Quick Start

### Prerequisites

- NVIDIA Jetson device (tested on Jetson Orin series)
- ROS2 Jazzy
- Ubuntu 24.04 (JetPack 7)

### Setup

For detailed environment setup and dependencies, see **[SETUP.md](SETUP.md)**.

```bash
# Clone the repository
git clone https://github.com/alansrobotlab2/alfiebot_ws.git
cd alfiebot_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.zsh
```

### Running Alfiebot

```bash
# Launch the complete system
ros2 launch alfie_bringup alfiebot.launch.py

# Or launch individual components as needed
ros2 run alfie_asr asr_node
ros2 run alfie_llm llm_node
ros2 run alfie_tts tts_node
```

---

## 🛠️ Development Status

### Active Tasks

- [ ] Troubleshoot micro ros agent locking out usb when quitting launchfile
- [ ] Evaluate BEST_EFFORT vs RELIABLE QoS for core topics
- [ ] Debug message/video lag after overnight operation
- [ ] Investigate `alfie_mic` crash issue (related to reset step?)
- [ ] Benchmark Foxglove Bridge performance (on-device vs off-device)
- [ ] Fix ServoTool2 memory offset bug affecting EEPROM writes
- [ ] Add rate limiting to ServoTool slider (target: 20Hz)
- [ ] Implement min/max value constraints in ServoTool UI
- [ ] Add raw/degrees/radians unit selector to ServoTool I/O

### Modeling & Simulation

- [ ] Create simplified OnShape CAD model
- [ ] Export URDF for Foxglove Studio visualization
- [ ] Export model to NVIDIA Omniverse

### Hardware Development

- [ ] Build teleoperation rig with Teensy 4.2
- [ ] Implement "high five" gesture interaction

#### Upcoming Hardware Improvements

- [ ] Install dual antennas on head with body extensions
- [ ] Replace 6.6ft USB-C cable with 5ft version
- [ ] Upgrade shoulder bearings

#### Completed Hardware Updates ✅

- ✅ Installed smaller diameter middle wheel inserts (reduced turn jiggle)
- ✅ Replaced with quieter case fans in base
- ✅ Added wiring harness slack for 180° head rotation
- ✅ Centered Jetson for improved airflow

---

## 🔮 Future Roadmap

### Hardware Evolution

- **Base Redesign** - Lower-mounted drive wheels or omniwheel configuration
- **Autonomous Charging** - Docking station with automatic charge detection
- **Cable Management** - Cleaner internal wiring layout

### Software Enhancements

- Advanced behavioral planning and decision-making
- Multi-modal interaction (vision + voice)
- Cloud integration for knowledge updates
- Improved power management and battery monitoring

---

## 📚 Documentation

- **[SETUP.md](SETUP.md)** - Detailed installation and configuration guide
- **[Firmware README](src/alfie_firmware/README.md)** - Micro-ROS firmware documentation
- **[MKS-SERVO42C_1.0.md](MKS-SERVO42C_1.0.md)** - Servo motor documentation
- **[alfienet/](alfienet/)** - Network configuration documentation

---

## 🤝 Contributing

Contributions are welcome! This is an active research and development project.

---

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

---

## 👤 Maintainer

**Alan's Robot Lab**  
📧 alansrobotlab@gmail.com  
🔗 [GitHub](https://github.com/alansrobotlab2)

---

<div align="center">
  <sub>Built with ❤️ and ROS2</sub>
</div>


