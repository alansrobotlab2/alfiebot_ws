# Waveshare General Driverboard Firmware for Alfiebot

Alfiebot uses two (slightly) modified driverboards to handle some hardware functions.  

This code is maintained in the PlatformIO extension through Visual Studio Code.

The codebase uses freertos to distribute two main tasks across the two available cores.
- core 0 focuses on the hardware tasks, including polling the serial bus servos at 100hz as well broadcasting commands for the same.
- core 1 focuses on the ros2 stack, including broadcasting 100hz status messages and accepting commands for the same.

In order to work with the project, open the ~/alfiebot_ws/src/alfiebot_firmware folder directly and platformio will recognize it as a platformio project.

Each board has different peripherals.  by changing the DRIVERBOARD define in config.h and selecting the appropriate ttyUSB port you can compile and deploy the firmware to the correct board.

On my system:
 - DRIVERBOARD 0 corresponds to /dev/ttyUSB0
 - DRIVERBOARD 1 corresponds to /dev/ttyUSB1


## hardware key

### driver0 - top board
/dev/ttyUSB0
right arm servos
head servos
right arm servos
wheel motor drivers

### driver1 - bottom board
/dev/ttyUSB1
left arm servos
eye light drivers
shoulder limit switch





What's currently broken:




What is left to implement:






to clean micro_ros_platformio
like when you change msg formats etc
click on icon on sidebar -> select Miscellaneous -> PlatformIO Core CLI
pio run --target clean_microros

to get to the platformio homepage
add 8008 to forwarded ports
click on icon on sidebar -> select Miscellaneous -> PlatformIO Core CLI
pio home
select open in editor