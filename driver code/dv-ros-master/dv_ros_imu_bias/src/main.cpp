#include <dv_ros_messaging/messaging.hpp>

#include <dv_ros_imu_bias/BiasesEstimationConfig.h>
#include <dv_ros_imu_bias/SetImuBiasesService.h>

#include <Eigen/Core>

#include <ros/ros.h>

#include <chrono>
#include <dynamic_reconfigure/server.h>
#include <tf2_msgs/TFMessage.h>

using namespace std::chrono_literals;
using TransformsMessage = DV_ROS_MSGS(tf2_msgs::TFMessage);

ros::Subscriber imuSubscriber;
int64_t startTimestamp = -1, collectionDuration = 0;
float varianceThreshold, gravityRange;
std::vector<std::vector<float>> accValues(3), gyroValues(3);
Eigen::Vector3f accBiases, gyroBiases;

bool startCollecting = false;
ros::ServiceClient client;

void storeImuBiases() {
	dv_ros_capture::SetImuBiases srv;

	srv.request.accBiases.x = accBiases.x();
	srv.request.accBiases.y = accBiases.y();
	srv.request.accBiases.z = accBiases.z();

	srv.request.gyroBiases.x = gyroBiases.x();
	srv.request.gyroBiases.y = gyroBiases.y();
	srv.request.gyroBiases.z = gyroBiases.z();

	client.call(srv);
	auto rsp = srv.response;
	ROS_INFO_STREAM(rsp.status_message);
	if (rsp.success) {
		imuSubscriber.shutdown();
		ROS_INFO("Shutting down IMU biases estimation node...");
		ros::shutdown();
	}
	else {
		ROS_ERROR("Setting IMU biases failed, please investigate any issues in the log and retry.");
	}
}

void estimateBias() {
	const Eigen::Vector3f earthG(0.f, 9.81007f, 0.f);

	// Validate standard deviation to make sure the data device was stable during
	// collection
	std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf>> values;

	// Mapping the data
	for (const auto &acc : accValues) {
		Eigen::VectorXf data = Eigen::VectorXf::Map(acc.data(), static_cast<Eigen::Index>(acc.size()));
		values.push_back(data);
	}
	for (const auto &gyro : gyroValues) {
		Eigen::VectorXf data = Eigen::VectorXf::Map(gyro.data(), static_cast<Eigen::Index>(gyro.size()));
		values.push_back(data);
	}

	std::vector<float> biases;
	// Sanity checking and mean estimation
	for (auto &sequence : values) {
		const float mean = sequence.mean();
		const float variance
			= std::sqrt((sequence.array() - mean).square().sum() / static_cast<float>(sequence.size() - 1));

		if (variance > varianceThreshold) {
			throw dv::exceptions::RuntimeError("Some motion detected in IMU data, please keep the device steady "
											   "while collecting and repeat.");
		}

		biases.push_back(mean);
	}

	if (biases[1] < -(earthG[1] + gravityRange) || biases[1] > -(earthG[1] - gravityRange)) {
		throw dv::exceptions::RuntimeError(
			"Gravity vector is not aligned, please make sure to place the camera horizontally on a level surface.");
	}

	accBiases  = Eigen::Vector3f(biases[0], biases[1], biases[2]) + earthG;
	gyroBiases = Eigen::Vector3f(biases[3], biases[4], biases[5]);

	// Result printing
	ROS_INFO("Bias estimation is successful!");
	ROS_INFO_STREAM("Accelerometer biases [x, y, z] in m/s^2: "
					<< "[" << accBiases.x() << ", " << accBiases.y() << ", " << accBiases.z() << "]");
	ROS_INFO_STREAM("Gyroscope biases [x, y, z] in rad/s: "
					<< "[" << gyroBiases.x() << ", " << gyroBiases.y() << ", " << gyroBiases.z() << "]");
	ROS_INFO_STREAM("Earth gravity vector: [ " << earthG.x() << ", " << earthG.y() << ", " << earthG.z() << " ]");
}

void imuCallback(const dv_ros_msgs::ImuMessage::ConstPtr &msgPtr) {
	if (!startCollecting) {
		return;
	}
	if (startTimestamp < 0) {
		startTimestamp = dv_ros_msgs::toDvTime(msgPtr->header.stamp);
	}
	accValues[0].push_back(static_cast<float>(msgPtr->linear_acceleration.x));
	accValues[1].push_back(static_cast<float>(msgPtr->linear_acceleration.y));
	accValues[2].push_back(static_cast<float>(msgPtr->linear_acceleration.z));

	gyroValues[0].push_back(static_cast<float>(msgPtr->angular_velocity.x));
	gyroValues[1].push_back(static_cast<float>(msgPtr->angular_velocity.y));
	gyroValues[2].push_back(static_cast<float>(msgPtr->angular_velocity.z));

	if (dv_ros_msgs::toDvTime(msgPtr->header.stamp) - startTimestamp > collectionDuration) {
		startCollecting = false;
		// Print some info
		const size_t sampleSize = accValues[0].size();
		ROS_INFO_STREAM(
			"Collected " << sampleSize << " samples over "
						 << static_cast<double>(dv_ros_msgs::toDvTime(msgPtr->header.stamp) - startTimestamp) * 1e-6
						 << " seconds");

		estimateBias();
		storeImuBiases();
	}
}

void reconfigureBias(dv_ros_imu_bias::BiasesEstimationConfig &config, uint32_t l) {
	varianceThreshold  = static_cast<float>(config.variance_threshold);
	gravityRange       = static_cast<float>(config.gravity_range);
	collectionDuration = static_cast<int64_t>(1e+6 * config.collection_duration);

	if (config.estimate_biases) {
		startCollecting        = !startCollecting;
		config.estimate_biases = false;
		ROS_INFO("Start collecting IMU data.");
	}
}

int main(int argc, char **argv) {
	ROS_INFO("Keep the camera steady on a level surface and press start-collecting in the rqt dynamic reconfiguration "
			 "setup...");
	ros::init(argc, argv, "bias_estimation_node");

	ros::NodeHandle nodeHandler("~");
	auto camNamespace = nodeHandler.param<std::string>("/set_calibration/camera_namespace", "/capture_node/");
	client            = nodeHandler.serviceClient<dv_ros_capture::SetImuBiases>(camNamespace + "set_imu_biases");

	boost::recursive_mutex mReaderMutex;
	dynamic_reconfigure::Server<dv_ros_imu_bias::BiasesEstimationConfig> dynamicReconfigure(nodeHandler);

	dv_ros_imu_bias::BiasesEstimationConfig initialSettings = dv_ros_imu_bias::BiasesEstimationConfig::__getDefault__();
	initialSettings.variance_threshold                      = 0.1;
	initialSettings.gravity_range                           = 0.25;
	initialSettings.collection_duration                     = 1.0;

	dynamic_reconfigure::Server<dv_ros_imu_bias::BiasesEstimationConfig>::CallbackType f;
	f = boost::bind(&reconfigureBias, _1, _2);
	dynamicReconfigure.setCallback(f);

	imuSubscriber = nodeHandler.subscribe("imu", 10, &imuCallback);

	ros::spin();
	return EXIT_SUCCESS;
}
