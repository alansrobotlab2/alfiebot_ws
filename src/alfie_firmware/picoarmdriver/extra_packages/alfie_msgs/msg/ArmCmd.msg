# Command for one arm: 6 logical joints (SI units, see ServoCmd).
# The firmware expands the shoulder-pitch joint into a mirrored 2-servo coupled
# pair, so 6 joints drive 7 physical bus servos.
#
# Joint order:
#   0 shoulder_yaw
#   1 shoulder_pitch
#   2 elbow_pitch
#   3 wrist_pitch
#   4 wrist_roll
#   5 gripper
ServoCmd[6] joint_cmd
