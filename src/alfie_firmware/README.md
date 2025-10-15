What's currently broken:
 - servo low position msg feedback needs to account for 12bit int
 - memory map population from servo is not working; mostly 0s
 - update service should update local memory map as well as servo
 - unable to upload firmware after running alfie_bringup.  annoying



What is left to implement:
 - is there a way to reposition the jetson to exit air from both sides?


driver0 - top board
/dev/ttyUSB0
right arm servos
head servos
left arm servos
wheel motor drivers

driver1 - bottom board
/dev/ttyUSB1

eye light drivers
shoulder limit switch


to clean micro_ros_platformio
like when you change msg formats etc
click on icon on sidebar -> select Miscellaneous -> PlatformIO Core CLI
pio run --target clean_microros

to get to the platformio homepage
add 8008 to forwarded ports
click on icon on sidebar -> select Miscellaneous -> PlatformIO Core CLI
pio home
select open in editor