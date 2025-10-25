---
applyTo: '**'
---

# Project Overview

This project allows a Waveshare RP2040-Zero, which is a RP2040 derivative board to act as a device connected to a Waveshare General Driver Board over I2C.

The Pico will be attached to the hosts I2C bus as a slave at address 0x58

The WS2812B LED is connected to GP16


## Development Environment

This solution is developed under VSCode using the PlatformIO extension.
The code is built using the Arduino framework for the RP2040.

## Coding Standards

1. Use descriptive variable and function names.
2. Include comments to explain complex logic.
3. Follow consistent indentation and formatting.
4. Adhere to the Arduino framework conventions for RP2040.
5. Modularize code into functions for better readability and maintenance.
6. Use constants and macros for fixed values (e.g., pin numbers, I2C addresses).
7. Implement error handling for I2C communication.
8. Test code on actual hardware to ensure functionality.
9. Always check the README to make sure it's up to date with the code.
10. Always check and update the triple quoted comments to make sure that hover help is accurate.