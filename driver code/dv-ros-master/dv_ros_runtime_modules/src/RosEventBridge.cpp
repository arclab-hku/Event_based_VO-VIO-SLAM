#include "RosEventBridge.h"

#include <regex>

void RosEventBridge::initInputs(dv::InputDefinitionList &in) {
	in.addEventInput("events");
	in.addTriggerInput("sync", true);
}

const char *RosEventBridge::initDescription() {
	return "Publishes event stream into ROS.";
}

bool RosEventBridge::loadCalibrationFile(const std::string &filename) {
	// loads single camera calibration file
	if (filename.empty()) {
		log.error << "No camera calibration file specified." << std::endl;
		return (false);
	}

	cv::FileStorage fs(filename, cv::FileStorage::READ);

	if (!fs.isOpened()) {
		log.error << "Impossible to load the camera calibration file: " << filename << std::endl;
		return (false);
	}
	auto typeNode = fs["type"];

	if (!cvExists(typeNode) || !typeNode.isString()) {
		log.info << "Old-style camera calibration file found." << std::endl;

		if (!cvExists(fs["camera_matrix"]) || !cvExists(fs["distortion_coefficients"])) {
			log.error.format("Calibration data not present in file: %s", filename);
			return (false);
		}

		fs["camera_matrix"] >> cameraMat;
		fs["distortion_coefficients"] >> distortionCoeffs;

		log.info.format("Loaded camera matrix and distortion coefficients from file: %s", filename);
	}
	else {
		log.info << "New-style camera calibration file found." << std::endl;

		auto cameraNode = fs[cameraID];

		if (!cvExists(cameraNode) || !cameraNode.isMap()) {
			log.error.format("Calibration data for camera %s not present in file: %s", cameraID, filename);
			return (false);
		}

		if (!cvExists(cameraNode["camera_matrix"]) || !cvExists(cameraNode["distortion_coefficients"])) {
			log.error.format("Calibration data for camera %s not present in file: %s", cameraID, filename);
			return (false);
		}

		cameraNode["camera_matrix"] >> cameraMat;
		cameraNode["distortion_coefficients"] >> distortionCoeffs;

		log.info.format(
			"Loaded camera matrix and distortion coefficients for camera %s from file: %s", cameraID, filename);
	}

	// Only fisheye is different, let's keep this by default for now
	cameraInfo.distortion_model = "plumb_bob";

	// Copy cameraMat by value
	for (int i = 0; i < cameraMat.rows; i++) {
		for (int j = 0; j < cameraMat.cols; j++) {
			cameraInfo.K.at(i * cameraMat.rows + j)       = cameraMat.at<double>(i, j);
			cameraInfo.P.at(i * (cameraMat.rows + 1) + j) = cameraMat.at<double>(i, j);
		}
	}

	cameraInfo.P.at(11) = 1.0;

	for (int i = 0; i < distortionCoeffs.rows; i++) {
		for (int j = 0; j < distortionCoeffs.cols; j++) {
			cameraInfo.D.push_back(distortionCoeffs.at<double>(i, j));
		}
	}

	// Identity matrix
	cameraInfo.R.at(0) = 1.0;
	cameraInfo.R.at(4) = 1.0;
	cameraInfo.R.at(8) = 1.0;

	return true;
}

void RosEventBridge::publishEventsMsg(const dv_ros_msgs::EventArrayMessage &msg) {
	pub->publish(msg);
	if (camera_info_pub_) {
		cameraInfo.header.stamp = msg.header.stamp;
		camera_info_pub_->publish(cameraInfo);
	}
	ros::spinOnce();
}

void RosEventBridge::run() {
	auto events = inputs.getEventInput("events");

	if (!pub) {
		// Read all configuration, initialize the ROS connection.
		char **argv = {nullptr};
		int argc    = 0;
		ros::init(argc, argv, "DVS_Event_Publisher");

		nh                  = std::make_unique<ros::NodeHandle>();
		std::string topicNS = config.getString("topicNamespace");
		pub = std::make_unique<ros::Publisher>(nh->advertise<dv_ros_msgs::EventArray>(topicNS + "/events", 10));

		// Load calibration from file.
		setCameraID(events.getOriginDescription());
		if (loadCalibrationFile(config.getString("calibrationFile"))) {
			camera_info_pub_
				= std::make_unique<ros::Publisher>(nh->advertise<sensor_msgs::CameraInfo>(topicNS + "/camera_info", 1));
			cameraInfo.width  = static_cast<unsigned int>(events.sizeX());
			cameraInfo.height = static_cast<unsigned int>(events.sizeY());
		}
	}

	auto sync = inputs.getTriggerInput("sync");
	if (sync.isConnected()) {
		if (auto e = events.events()) {
			eventsBuffer.push_back(std::make_shared<DvEvents>(e));
			width  = static_cast<uint16_t>(events.sizeX());
			height = static_cast<uint16_t>(events.sizeY());
		}
		if (sync.data() && !eventsBuffer.empty()) {
			int64_t timestamp  = sync.data().front().timestamp;
			auto closestEvents = findClosest(timestamp);
			dv::EventStore store(**closestEvents);

			auto eventsArray = dv_ros_msgs::toRosEventsMessage(store, cv::Size(width, height));
			publishEventsMsg(eventsArray);
		}
	}
	else {
		dv::EventStore store(events.events().getBasePointer());
		auto eventsArray = dv_ros_msgs::toRosEventsMessage(store, cv::Size(width, height));

		publishEventsMsg(eventsArray);
	}
}

void RosEventBridge::configUpdate() {
	uint32_t bufferSize = static_cast<uint32_t>(config.getInt("bufferSize"));
	eventsBuffer        = boost::circular_buffer<DvEventsPtr>(bufferSize);
	ModuleBase::configUpdate();
}

void RosEventBridge::initConfigOptions(dv::RuntimeConfig &config) {
	config.add("topicNamespace", dv::ConfigOption::stringOption("ROS Topic name", "/dvxplorer"));
	config.add("calibrationFile",
		dv::ConfigOption::fileOpenOption(
			"The name of the file from which to load the calibration settings for undistortion.", "xml"));
	config.add("bufferSize", dv::ConfigOption::intOption("Size of the circular buffer", 10, 1, 1000));

	config.setPriorityOptions({"calibrationFile", "topicNamespace"});
}

bool RosEventBridge::cvExists(const cv::FileNode &fn) {
	return fn.type() != cv::FileNode::NONE;
}

void RosEventBridge::setCameraID(const std::string &originDescription) {
	// Camera origin descriptions are fine, never start with a digit or have spaces
	// or other special characters in them that fail with OpenCV's FileStorage.
	// So we just clean/escape the string for possible other sources.
	auto str = std::regex_replace(originDescription, filenameCleanupRegex, "_");

	// FileStorage node names can't start with - or 0-9.
	// Prefix with an underscore in that case.
	if ((str[0] == '-') || (std::isdigit(str[0]) != 0)) {
		str = "_" + str;
	}

	cameraID = str;
}

RosEventBridge::EventsBuffer::const_iterator RosEventBridge::findClosest(int64_t timestamp) const {
	int64_t minDistance = INT64_MAX;
	auto iter           = eventsBuffer.begin();
	auto minIter        = eventsBuffer.end();
	while (iter != eventsBuffer.end()) {
		int64_t timedistance = std::abs((*iter)->front().timestamp() - timestamp);
		if (timedistance < minDistance) {
			minIter     = iter;
			minDistance = timedistance;
		}
		iter++;
	}
	return minIter;
}

registerModuleClass(RosEventBridge)
