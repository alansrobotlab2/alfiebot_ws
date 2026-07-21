# Orientation (quaternion)
float64 orientation_x
float64 orientation_y
float64 orientation_z
float64 orientation_w
#float64[9] orientation_covariance

# True when the quaternion above is the mag-referenced rotation vector, i.e. the
# absolute heading is valid. False when the compass has been decoupled (the
# quaternion is the game rotation vector: gyro+accel only, yaw is relative/free)
# because nearby motor current would corrupt the magnetometer. Consumers that
# fuse absolute yaw should ignore this message's orientation when false.
# NB: false is the safe default (legacy/zero-init messages read as unreliable).
bool orientation_reliable

# Angular velocity
float64 angular_velocity_x
float64 angular_velocity_y
float64 angular_velocity_z
#float64[9] angular_velocity_covariance

# Linear acceleration
float64 linear_acceleration_x
float64 linear_acceleration_y
float64 linear_acceleration_z
#float64[9] linear_acceleration_covariance