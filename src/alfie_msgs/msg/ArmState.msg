# State for one arm: 6 logical joints (SI units, see ServoState).
# Same joint order as ArmCmd. The derived shoulder-pitch servo is not reported;
# the shoulder-pitch joint carries the primary servo's feedback.
std_msgs/Header header

ServoState[6] joint_state

# Pico unique board id (8-byte QSPI flash RUID). Used to map this board to the
# left or right arm; echoed here so the id can be read over ROS for provisioning.
uint8[8] board_serial

# Resolved side: 0 = left, 1 = right, 255 = unknown (serial not in the table).
uint8 arm_side
