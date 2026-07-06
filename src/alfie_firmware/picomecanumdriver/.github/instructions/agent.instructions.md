---
applyTo: '**'
---

# Project Overview

This project allows a Waveshare RP2040-Zero to act as a device connected to a host computer using microros ros2.

The four mecanum wheels are driven by a Hiwonder 4-channel encoder motor driver connected over I2C (GP12 = SDA, GP13 = SCL). The Hiwonder controller runs its own per-channel closed-loop velocity PID, so the RP2040 only sends target speeds and reads back accumulated encoder counts over I2C. The board subscribes to a geometry_msgs/Twist (`/mecanumdrive`), performs inverse mecanum kinematics on-chip, and publishes nav_msgs/Odometry (`/odom`).

The onboard status LED is a single WS2812 RGB on GP16 (driven via Adafruit NeoPixel) — there is no plain digital LED.

## Development Environment

This solution is developed under VSCode using the PlatformIO extension.
The code is built using the Arduino framework for the RP2040 (earlephilhower core).

## Solution Tips

1. we're using the serial port for microros, so no serial print statements can be used for debugging.


## Coding Standards

1. Use descriptive variable and function names.
2. Include comments to explain complex logic.
3. Follow consistent indentation and formatting.
4. Adhere to the Arduino framework conventions for RP2040.
5. Modularize code into functions for better readability and maintenance.
6. Use constants and macros for fixed values (e.g., pin numbers, serial baud rate).
7. Implement error handling for serial communication.
8. Test code on actual hardware to ensure functionality.
9. Always check the README to make sure it's up to date with the code.
10. Always check and update the triple quoted comments to make sure that hover help is accurate.