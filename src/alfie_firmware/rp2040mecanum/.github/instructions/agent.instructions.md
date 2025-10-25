---
applyTo: '**'
---

# Project Overview

This project allows a Raspberry Pi Pico, to act as a device connected to a host computer using microros ros2

There are a total of 4 h bridges connected via gpio.  each h bridge has two pins for direction and one pin for pwm speed control.

## Development Environment

This solution is developed under VSCode using the PlatformIO extension.
The code is built using the Arduino framework for the RP2040.

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