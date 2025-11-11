# Waveshare Driver Board Programmer

## Overview
This is a simple ESP32 programmer utility that writes configuration data to the board's simulated EEPROM (NVS - Non-Volatile Storage). It uses the Arduino Preferences library to store a unique board ID for the two Waveshare general driver boards on the robot.

## What It Does
The programmer stores a **Board ID** value in the ESP32's non-volatile storage under the namespace `boardconfig` with the key `boardid`.

**Valid Board IDs:**
- **0** - First Waveshare driver board
- **1** - Second Waveshare driver board

## How to Use

### 1. Set the Board ID Value
Open `src/main.cpp` and modify the board ID value on line 16:

```cpp
boardconfig.putUInt("boardid", 0);  // Change to 0 or 1
```

Set the value to:
- **0** for the first driver board
- **1** for the second driver board

### 2. Upload the Program
Connect your ESP32 board via USB and run:

```bash
pio run --target upload
```

Or use the PlatformIO upload button in VS Code.

### 3. Monitor the Programming Process
After uploading, open the serial monitor to verify the programming:

```bash
pio device monitor
```

Or use the PlatformIO serial monitor in VS Code.

You should see output like:
```
ESP32 EEPROM Programming - Board Configuration
Board ID written to EEPROM: 0
Board ID read from EEPROM: 0
Programming complete!
```

### 4. Verify the Board ID
The program automatically reads back the stored value and displays it on the serial monitor to confirm successful programming.

## Important Notes

- **Valid Values**: Only use **0** or **1** as board IDs (one for each Waveshare driver board on the robot).
- **Persistent Storage**: The board ID is stored in NVS and will persist across power cycles and firmware updates (unless explicitly erased).
- **One-Time Programming**: Once programmed, you typically don't need to run this again unless you want to change the board ID.
- **Namespace**: The data is stored under the namespace `"boardconfig"` which can be accessed by other programs on the same ESP32.
- **Data Type**: The board ID is stored as an unsigned integer (`uint`).

## Changing Board IDs

To program both driver boards:
1. Set the value to `0` in `main.cpp`
2. Upload to the first board
3. Verify via serial monitor
4. Set the value to `1` in `main.cpp`
5. Upload to the second board
6. Verify via serial monitor

## Erasing the EEPROM (Optional)

If you need to completely erase the NVS storage:

```bash
pio run --target erase
```

Then re-upload the programmer.

## Technical Details

- **Framework**: Arduino for ESP32
- **Library**: Preferences (built-in to ESP32 Arduino core)
- **Storage**: NVS partition in flash memory
- **Baud Rate**: 115200
- **Board**: ESP-WROVER-KIT (or compatible ESP32 board)
