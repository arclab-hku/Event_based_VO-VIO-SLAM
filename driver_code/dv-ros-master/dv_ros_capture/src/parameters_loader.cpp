#include <dv_ros_capture/parameters_loader.hpp>

#include <fmt/core.h>
#include <fmt/format.h>

#include <iostream>
#include <string>

using namespace dv_ros_node;

ParametersLoader::ParametersLoader(const ros::NodeHandle &nodeHandle) {
	nodeHandle.getParam("frames", params_.frames);//是否有图像
	nodeHandle.getParam("events", params_.events);//是否有事件
	nodeHandle.getParam("imu", params_.imu);//是否有IMU
	nodeHandle.getParam("triggers", params_.triggers);//If true, Trigger msg containing Trigger data is published
	nodeHandle.getParam("cameraName", params_.cameraName);
	int timeIncrement = static_cast<int>(params_.timeIncrement);//确定时间间隔，也就是频率
	nodeHandle.getParam("timeIncrement", timeIncrement);
	params_.timeIncrement = static_cast<int64_t>(timeIncrement);

	nodeHandle.param<std::string>("cameraFrameName", params_.cameraFrameName, "camera");//参数文件中改为“dvs”
	nodeHandle.param<std::string>("imuFrameName", params_.imuFrameName, "imu");//imu frame的名字
	nodeHandle.param<bool>("transformImuToCameraFrame", params_.transformImuToCameraFrame, false);//是否将IMU数据转换到相机坐标系下 (原本为true)
	nodeHandle.param<bool>("unbiasedImuData", params_.unbiasedImuData, false);//是否进行校准IMU，默认不校准（原本为true）

	nodeHandle.param<bool>("noiseFiltering", params_.noiseFiltering, true);//是否进行噪声滤波（原本为false）
	int value;
	nodeHandle.param<int>("noiseBackgroundActivityTime", value, 2000);//默认的去除噪声的参数
	params_.noiseBATime = static_cast<int64_t>(value);

	std::string tmp;
	nodeHandle.getParam("cameraCalibrationFilePath", tmp);
	params_.cameraCalibrationFilePath = static_cast<std::filesystem::path>(tmp);

	if (nodeHandle.getParam("aedat4FilePath", tmp)) {
		params_.aedat4FilePath = tmp;
		if (!params_.aedat4FilePath.empty() && !std::filesystem::exists(params_.aedat4FilePath)) {
			throw std::invalid_argument(
				fmt::format("File {0} not found. Please provide a correct path in the configuration file.",
					params_.aedat4FilePath.string()));
		}
	}

	nodeHandle.param<std::vector<std::string>>("syncDevices", params_.syncDeviceList, {});
	nodeHandle.param<bool>("waitForSync", params_.waitForSync, false);
}

Params ParametersLoader::getParams() {
	return params_;
}

void ParametersLoader::printConfiguration() {
	ROS_INFO(">>>>>> Capture node parameter settings: <<<<<<");
	// Always print the config, it's easier to understand the behavior
	ROS_INFO_STREAM("Frames enabled: " << (params_.frames ? "yes" : "no"));
	ROS_INFO_STREAM("Events enabled: " << (params_.events ? "yes" : "no"));
	ROS_INFO_STREAM("Imu enabled: " << (params_.imu ? "yes" : "no"));
	ROS_INFO_STREAM("Convert IMU to camera frame: " << (params_.transformImuToCameraFrame ? "yes" : "no"));
	ROS_INFO_STREAM("Triggers enabled: " << (params_.triggers ? "yes" : "no"));
	if (!params_.aedat4FilePath.empty()) {
		ROS_INFO_STREAM("aedat4FilePath: " << params_.aedat4FilePath);
	}
	if (!params_.cameraName.empty()) {
		ROS_INFO_STREAM("cameraName: " << params_.cameraName);
	}
	ROS_INFO_STREAM("cameraCalibrationFilePath: " << params_.cameraCalibrationFilePath);
	ROS_INFO_STREAM("noiseFiltering: " << (params_.noiseFiltering ? "yes" : "no"));
	ROS_INFO_STREAM("noiseBackgroundActivityTime: " << params_.noiseBATime);
	ROS_INFO_STREAM("syncDevices: " << fmt::format("[{}]", fmt::join(params_.syncDeviceList, ", ")));
	ROS_INFO_STREAM("waitForSync: " << (params_.waitForSync ? "yes" : "no"));
	ROS_INFO(">>>>>> End of parameters <<<<<<");
}
