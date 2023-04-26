Header header

# Transformation from sensor (IMU) to camera.
# Transforms any point from IMU kinematic reference frame to the camera reference frame.
geometry_msgs/Transform T_SC

# Offset between the camera and IMU timestamps in microseconds (t_correct = t_imu - offset)
int64 timeOffsetMicros

# Variance of the offset of the angular velocity [rad/s]
float32 omegaOffsetVar

# Variance of the offset of the acceleration [m/s^2]
float32 accOffsetVar

# Noise density of the gyroscope [rad/s^s/sqrt(Hz)]
float32 omegaNoiseDensity

# Noise density of the accelerometer [m/s^2/sqrt(Hz)]
float32 accNoiseDensity

# Noise random walk of the gyroscope [rad/s^s/sqrt(Hz)]
float32 omegaNoiseRandomWalk

# Noise random walk of the accelerometer [m/s^2/sqrt(Hz)]
float32 accNoiseRandomWalk
