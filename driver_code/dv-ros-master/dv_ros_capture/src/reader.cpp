#include <dv_ros_capture/reader.hpp>

using namespace dv_ros_node;

Reader::Reader(const std::filesystem::path &aedat4FilePath, const std::string &cameraName) {
	monoCameraRecordingPtr = std::make_unique<dv::io::MonoCameraRecording>(aedat4FilePath, cameraName);
	mCameraCapture         = false;
}

Reader::Reader(const std::string &cameraName) {
	cameraCapturePtr = std::make_unique<dv::io::CameraCapture>(cameraName);
	mCameraCapture   = true;
}

std::optional<cv::Size> Reader::getFrameResolution() const {
	if (mCameraCapture) {
		return cameraCapturePtr->getFrameResolution();
	}
	else {
		return monoCameraRecordingPtr->getFrameResolution();
	}
}

std::optional<cv::Size> Reader::getEventResolution() const {
	if (mCameraCapture) {
		return cameraCapturePtr->getEventResolution();
	}
	else {
		return monoCameraRecordingPtr->getEventResolution();
	}
}

std::optional<dv::EventStore> Reader::getNextEventBatch() {
	if (mCameraCapture) {
		return cameraCapturePtr->getNextEventBatch();
	}
	else {
		return monoCameraRecordingPtr->getNextEventBatch();
	}
}

std::optional<dv::cvector<dv::IMU>> Reader::getNextImuBatch() {
	if (mCameraCapture) {
		return cameraCapturePtr->getNextImuBatch();
	}
	else {
		return monoCameraRecordingPtr->getNextImuBatch();
	}
}

std::optional<dv::Frame> Reader::getNextFrame() {
	if (mCameraCapture) {
		return cameraCapturePtr->getNextFrame();
	}
	else {
		return monoCameraRecordingPtr->getNextFrame();
	}
}

std::optional<dv::cvector<dv::Trigger>> Reader::getNextTriggerBatch() {
	if (mCameraCapture) {
		return cameraCapturePtr->getNextTriggerBatch();
	}
	else {
		return monoCameraRecordingPtr->getNextTriggerBatch();
	}
}

bool Reader::isEventStreamAvailable() const {
	if (mCameraCapture) {
		return true;
	}
	else {
		return monoCameraRecordingPtr->isEventStreamAvailable();
	}
}

bool Reader::isFrameStreamAvailable() const {
	if (mCameraCapture) {
		return cameraCapturePtr->isFrameStreamAvailable();
	}
	else {
		return monoCameraRecordingPtr->isFrameStreamAvailable();
	}
}

bool Reader::isImuStreamAvailable() const {
	if (mCameraCapture) {
		return true;
	}
	else {
		return monoCameraRecordingPtr->isImuStreamAvailable();
	}
}

bool Reader::isTriggerStreamAvailable() const {
	if (mCameraCapture) {
		return true;
	}
	else {
		return monoCameraRecordingPtr->isTriggerStreamAvailable();
	}
}

std::optional<std::pair<int64_t, int64_t>> Reader::getTimeRange() const {
	if (mCameraCapture) {
		return std::nullopt;
	}
	else {
		return monoCameraRecordingPtr->getTimeRange();
	}
}

bool Reader::isConnected() const {
	if (mCameraCapture) {
		return cameraCapturePtr->isConnected();
	}
	else {
		return true;
	}
}
const std::unique_ptr<dv::io::CameraCapture> &Reader::getCameraCapturePtr() const {
	return cameraCapturePtr;
}

const std::unique_ptr<dv::io::MonoCameraRecording> &Reader::getMonoCameraRecordingPtr() const {
	return monoCameraRecordingPtr;
}

std::string Reader::getCameraName() const {
	if (mCameraCapture) {
		return cameraCapturePtr->getCameraName();
	}
	else {
		return monoCameraRecordingPtr->getCameraName();
	}
}
