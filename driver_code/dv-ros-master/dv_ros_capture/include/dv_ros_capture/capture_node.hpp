#pragma once

#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/camera/calibrations/camera_calibration.hpp>
#include <dv-processing/core/core.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include <dv-processing/noise/background_activity_noise_filter.hpp>

#include <dv_ros_messaging/messaging.hpp>

#include <dv_ros_capture/CameraDiscovery.h>
#include <dv_ros_capture/DAVISConfig.h>
#include <dv_ros_capture/DVXplorerConfig.h>
#include <dv_ros_capture/PlaybackConfig.h>

#include "SetImuBiasesService.h"
#include "SetImuInfoService.h"
#include "SynchronizeCameraService.h"
#include "parameters_loader.hpp"
#include "reader.hpp"

#include <boost/lockfree/spsc_queue.hpp>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <tf2_msgs/TFMessage.h>
#include <thread>

namespace dv_capture_node {
using TimestampQueue = boost::lockfree::spsc_queue<int64_t, boost::lockfree::capacity<1000>>;

using TransformMessage  = DV_ROS_MSGS(geometry_msgs::TransformStamped);
using TransformsMessage = DV_ROS_MSGS(tf2_msgs::TFMessage);
using DiscoveryMessage  = DV_ROS_MSGS(dv_ros_capture::CameraDiscovery);

namespace fs = std::filesystem;

/**
 * Initialize ros publishers to stream data from a DV camera or a aedat4 file.
 */
class CaptureNode {
public:
	/**
	 * Initialize the publisher according to the params.
	 * @param nodeHandle ros::NodeHandle.
	 * @param params dv_ros_node::Params read from a configuration file specified in the launch file.
	 */
	CaptureNode(std::shared_ptr<ros::NodeHandle> nodeHandle, const dv_ros_node::Params &params);

	/**
	 * Stop the running threads.
	 */
	~CaptureNode();

	/**
	 * Start the threads for reading the data.
	 */
	void startCapture();

	/**
	 * Stop the running threads.
	 */
	void stop();

	/**
	 * Check whether read threads are still running
	 * @return True if capture threads are still running, false otherwise.
	 */
	[[nodiscard]] bool isRunning() const;

private:
	// Publishers declaration.
	ros::Publisher mFramePublisher;
	ros::Publisher mCameraInfoPublisher;
	ros::Publisher mImuPublisher;
	ros::Publisher mEventArrayPublisher;
	ros::Publisher mTriggerPublisher;
	ros::Publisher mTransformPublisher;
	ros::Publisher mDiscoveryPublisher;
	ros::ServiceServer mCameraService;
	ros::ServiceServer mImuInfoService;
	ros::ServiceServer mImuBiasesService;
	std::unique_ptr<ros::ServiceServer> mSyncServerService = nullptr;
	std::shared_ptr<ros::NodeHandle> mNodeHandle           = nullptr;

	// Declare the Reader to read from the device or from a recording.
	dv_ros_node::Reader mReader;

	// Parameters read from the configuration file. Set to true the type of data that needs to be streamed.
	dv_ros_node::Params mParams;

	// Camera info message buffer
	dv_ros_msgs::CameraInfoMessage mCameraInfoMsg;
	std::unique_ptr<std::thread> mCameraInfoThread = nullptr;

	// threads related
	std::thread mFrameThread;
	TimestampQueue mFrameQueue;
	std::thread mImuThread;
	TimestampQueue mImuQueue;
	std::thread mEventsThread;
	TimestampQueue mEventsQueue;
	std::thread mTriggerThread;
	TimestampQueue mTriggerQueue;
	std::atomic<bool> mSpinThread = true;
	std::thread mClock;
	std::thread mSyncThread;
	std::unique_ptr<std::thread> mDiscoveryThread = nullptr;
	boost::recursive_mutex mReaderMutex;

	std::atomic<bool> mSynchronized;

	std::unique_ptr<dv::noise::BackgroundActivityNoiseFilter<>> mNoiseFilter = nullptr;
	void updateNoiseFilter(const bool enable, const int64_t backgroundActivityTime);

	dv::camera::CalibrationSet mCalibration;

	int64_t mImuTimeOffset      = 0;
	Eigen::Vector3f mAccBiases  = Eigen::Vector3f::Zero();
	Eigen::Vector3f mGyroBiases = Eigen::Vector3f::Zero();
	ros::Time startupTime;//开启时间
	std::atomic<int64_t> mCurrentSeek;
	std::optional<TransformsMessage> mImuToCamTransforms = std::nullopt;
	dv::kinematics::Transformationf mImuToCamTransform
		= dv::kinematics::Transformationf(0, Eigen::Vector3f::Zero(), Eigen::Quaternion<float>::Identity());

	std::unique_ptr<dynamic_reconfigure::Server<dv_ros_capture::DAVISConfig>> davisColorServer    = nullptr;
	std::unique_ptr<dynamic_reconfigure::Server<dv_ros_capture::DVXplorerConfig>> dvxplorerServer = nullptr;
	std::unique_ptr<dynamic_reconfigure::Server<dv_ros_capture::PlaybackConfig>> playbackServer   = nullptr;

	/**
	 * Publish the images and camera info if mFrameBool is True
	 */
	void framePublisher();
	bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp);

	/**
	 * Publish the imu message if mImuBool is True
	 */
	void imuPublisher();
	bool setImuInfo(dv_ros_capture::SetImuInfo::Request &req, dv_ros_capture::SetImuInfo::Response &rsp);
	bool setImuBiases(dv_ros_capture::SetImuBiases::Request &req, dv_ros_capture::SetImuBiases::Response &rsp);

	/**
	 * Publish the events if mEventsBool is True
	 */
	void eventsPublisher();

	/**
	 * Publish the triggers if mTriggerBool is True
	 */
	void triggerPublisher();

	/**
	 * Start a clock thread that gives time synchronization to all the other threads.
	 * @param start Start time of a recording file. (-1 if capturing from camera)
	 * @param end End time of a recording file. (-1 if capturing from camera)
	 * @param timeIncrement Increment of the timestamp at each iteration of the thread. The thread sleeps for
	 * timeIncrement micros.
	 */
	void clock(int64_t start, int64_t end, int64_t timeIncrement);

	/**
	 * Create a camera info message.
	 * @param cameraCalib camera parameters for the camera info message.
	 */
	void populateInfoMsg(const dv::camera::CameraGeometry &cameraGeometry);

	/**
	 * Convert the imu message frame into the camera frame if the transformation exists.
	 * @param imu
	 * @return ROS Imu message in camera reference frame
	 */
	[[nodiscard]] inline dv_ros_msgs::ImuMessage transformImuFrame(dv_ros_msgs::ImuMessage &&imu);

	/**
	 * Generate the CalibrationSet with the data from the Set Camera Info and the set IMU services.
	 * @return dv::camera::CalibrationSet
	 */
	void updateCalibrationSet();

	/**
	 * Stores the calibration data into a new file.
	 * @return path to the new file.
	 */
	[[nodiscard]] fs::path saveCalibration();

	/**
	 * Write current capture node calibration parameters into an active calibration file.
	 */
	void generateActiveCalibrationFile();

	/**
	 * Get the path to the active calibration file.
	 * @return Filesystem path to the currently opened camera active calibration file.
	 */
	[[nodiscard]] fs::path getActiveCalibrationPath() const;

	/**
	 * Get camera calibration directory for the currently opened camera, it uses
	 * @param createDirectories If true, the method will create the directory if it's not existing in the filesystem.
	 * @return Path to the calibration
	 */
	fs::path getCameraCalibrationDirectory(bool createDirectories = true) const;

	/**
	 * Creates and runs a thread that publishes discovery messages about the type of the camera.
	 * @param syncServiceName   Provide a synchronization service name in the discovery message if the camera
	 *                          is connected with a synchronization cable.
	 */
	void runDiscovery(const std::string &syncServiceName);

	/**
	 * Handler for the camera synchronization service.
	 * @param req       Synchronization request.
	 * @param rsp       Synchronization response.
	 * @return          Whether synchronization succeeded.
	 */
	[[nodiscar]] bool synchronizeCamera(
		dv_ros_capture::SynchronizeCamera::Request &req, dv_ros_capture::SynchronizeCamera::Response &rsp);

	/**
	 * A blocking call that waits until all devices in `mParams.syncDeviceList` is online and discovered over
	 * the /dvs/discovery topic.
	 * @return      A map of devices, where key is the camera name and value is the name of synchronization service.
	 */
	[[nodiscard]] std::map<std::string, std::string> discoverSyncDevices() const;

	/**
	 * Send synchronization calls to the list of devices. This function distributes the timestamp offset to the
	 * given devices over synchronization service calls. The list of devices should be retrieved by
	 * `discoverSyncDevices` call.
	 * @param serviceNames  List of devices to synchronize.
	 * @sa CaptureNode::discoverSyncDevices
	 */
	void sendSyncCalls(const std::map<std::string, std::string> &serviceNames) const;

	/**
	 * Run and maintain a synchronization service online for the master node to synchronize.
	 */
	void synchronizationThread();
};

} // namespace dv_capture_node
