#pragma once

#include <ros/ros.h>

#include <filesystem>

namespace dv_ros_node {
/**
 * Struct containing the parameters
 */
struct Params {
	int64_t timeIncrement = 1000;
	bool frames           = true;//是否有图像
	bool events           = true;//是否有事件
	bool imu              = true;//是否有IMU
	bool triggers         = true;
	std::string cameraName;//相机名称（带序列号的）
	std::filesystem::path aedat4FilePath;
	std::filesystem::path cameraCalibrationFilePath;
	std::string cameraFrameName    = "camera";
	std::string imuFrameName       = "imu";
	bool transformImuToCameraFrame = false;//是否将IMU数据转换到相机坐标系下 (原本为true)
	bool unbiasedImuData           = true;
	bool noiseFiltering            = true;//是否进行噪声滤波（原本为false）
	int64_t noiseBATime            = 2000;//默认的去除噪声的参数

// A list of other cameras connected with synchronization cable to this camera If this list is empty, the camera node will not properly synchronize them 
// Master camera node will wait for other cameras to be online and will send hardware synchronization signal 
// while other camera with “waitForSync = true” These cameras will report their status to the master node and will not publish any data until synchronization signal is received
	std::vector<std::string> syncDeviceList;//同步设备列表（由此相机向这些相机发布同步）
	bool waitForSync = false;//是否等待同步（）
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
