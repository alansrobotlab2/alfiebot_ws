# State of the mecanum drive board: fused wheel odometry + board health.
# Kept compact (no covariance blocks) so it fits one XRCE serial frame and can be
# published BEST_EFFORT like the other pico state topics. master_low_status
# republishes the odometry as nav_msgs/Odometry on low/odom.
# header.stamp is the board's millis() uptime, not synced wall time.
std_msgs/Header header

# Pose in the odom frame
float32 x                 # m
float32 y                 # m
float32 yaw               # rad

# Body-frame velocity (base_link)
float32 vx                # m/s
float32 vy                # m/s
float32 wz                # rad/s

# RP2040 die temperature (deg C, internal sensor), refreshed ~1 Hz. 0 until the
# first read.
uint8 board_temp
