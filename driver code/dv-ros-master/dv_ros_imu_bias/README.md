# DV ROS IMU biases estimation

DV ROS IMU biases estimation provides a node for estimating the IMU biases and store them into a calibration file
provided by the user.
The node takes as input the stream of the IMU data coming from the camera capture node and perform the biases guess. The
camera must be placed on a stable and level surface with the gravity vector in the same direction of the Y axis of the
camera frame.
