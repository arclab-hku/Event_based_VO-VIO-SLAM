#pragma once

#include <ros/ros.h>

#include <filesystem>

namespace dv_ros_node {
/**
 * Struct containing the parameters
 */
struct Params {
	int64_t timeIncrement = 1000;//事件的触发频率(Increment of the timestamp at each iteration of the thread. The thread sleeps for timeIncrement micros) (原本为1000，即1KHZ)
	bool frames           = true;//是否有图像
	bool events           = true;//是否有事件
	bool imu              = true;//是否有IMU
	bool triggers         = true;//If true, Trigger msg containing Trigger data is published
	std::string cameraName;//相机名称（带序列号的）
	std::filesystem::path aedat4FilePath;//Path to the aedat4 file, if not specified the first available camera will be read
	std::filesystem::path cameraCalibrationFilePath;//camera矫正文件的路径
	std::string cameraFrameName    = "dvs";//(原本为camera)Camera reference frame name for publishing IMU extrinsic calibration
	std::string imuFrameName       = "imu";// IMU reference frame name for publishing IMU extrinsic calibration
	bool transformImuToCameraFrame = false;//是否将IMU数据转换到相机坐标系下 (原本为true)
	bool unbiasedImuData           = false;//(原本为true)If true, publish unbiased IMU data, that is IMU bias values that are read from calibration file will be subtracted out of the raw IMU stream
	bool noiseFiltering            = true;//是否进行噪声滤波（原本为false）
	int64_t noiseBATime            = 2000;//默认的去除噪声的参数(Noise filter background activity time threshold in microseconds)

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
