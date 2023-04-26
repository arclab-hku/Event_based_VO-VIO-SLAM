#pragma once

#include <ros/ros.h>

#include <filesystem>

namespace dv_ros_node {
/**
 * Struct containing the parameters
 */
struct Params {
	int64_t timeIncrement = 1000;
	bool frames           = true;
	bool events           = true;
	bool imu              = true;
	bool triggers         = true;
	std::string cameraName;
	std::filesystem::path aedat4FilePath;
	std::filesystem::path cameraCalibrationFilePath;
	std::string cameraFrameName    = "camera";
	std::string imuFrameName       = "imu";
	bool transformImuToCameraFrame = true;
	bool unbiasedImuData           = true;
	bool noiseFiltering            = false;
	int64_t noiseBATime            = 2000;

	std::vector<std::string> syncDeviceList;
	bool waitForSync = false;
};

/**
 * Read the parameters from configuration file.
 */
class ParametersLoader {
public:
	/**
	 * Load parameters from configuration file in config/settings_{setup_type}.yaml
	 *
	 * @param nodeHandle ros::NodeHandle
	 *
	 */
	explicit ParametersLoader(const ros::NodeHandle &nodeHandle);

	/**
	 * Return the parameters read from the configuration file.
	 *
	 * @return dv_capture_node::Params
	 */
	Params getParams();

	/**
	 * Log read data with ros::console::levels::Info
	 */
	void printConfiguration();

private:
	Params params_;
};

} // namespace dv_ros_node
