#pragma once
#include <dv_ros_messaging/messaging.hpp>

#include <dv_ros_msgs/Event.h>
#include <dv_ros_msgs/EventArray.h>

#include <boost/circular_buffer.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>

#include <dv-sdk/module.hpp>
#include <regex>
#include <sensor_msgs/CameraInfo.h>

class RosEventBridge : public dv::ModuleBase {
public:
	typedef dv::InputVectorDataWrapper<dv::EventPacket, dv::Event> DvEvents;
	typedef std::shared_ptr<DvEvents> DvEventsPtr;
	typedef boost::circular_buffer<DvEventsPtr> EventsBuffer;

private:
	static inline const std::regex filenameCleanupRegex{"[^a-zA-Z-_\\d]"};

	std::unique_ptr<ros::NodeHandle> nh              = nullptr;
	std::unique_ptr<ros::Publisher> pub              = nullptr;
	std::unique_ptr<ros::Publisher> camera_info_pub_ = nullptr;

	uint16_t width  = 0;
	uint16_t height = 0;

	cv::Mat distortionCoeffs;
	cv::Mat cameraMat;

	EventsBuffer eventsBuffer;

	DV_ROS_MSGS(sensor_msgs::CameraInfo) cameraInfo;
	std::string cameraID;

	bool loadCalibrationFile(const std::string &filename);

	static bool cvExists(const cv::FileNode &fn);

	void setCameraID(const std::string &originDescription);

public:
	static void initInputs(dv::InputDefinitionList &in);

	static const char *initDescription();

	static void initConfigOptions(dv::RuntimeConfig &config);

	void run() override;

	void configUpdate() override;

	EventsBuffer::const_iterator findClosest(int64_t timestamp) const;

	void publishEventsMsg(const dv_ros_msgs::EventArrayMessage &msg);
};
