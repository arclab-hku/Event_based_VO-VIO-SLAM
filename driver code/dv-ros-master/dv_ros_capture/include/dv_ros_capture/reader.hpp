#pragma once

#include <dv-processing/core/core.hpp>
#include <dv-processing/io/camera_capture.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>

#include <opencv2/core/types.hpp>

namespace dv_ros_node {
/**
 * Wrap the dv::io::CameraCapture and the dv::io::MonoCameraRecording in a single interface.
 */
class Reader {
public:
	Reader() = default;

	/**
	 * Construct a dv::io::MonoCameraRecording equivalent.
	 * @param aedat4FilePath path to the recording aedat4 file.
	 * @param cameraName name of the camera to stram. Can be an empty string.
	 */
	explicit Reader(const std::filesystem::path &aedat4FilePath, const std::string &cameraName);

	/**
	 * Construct a dv::io::CameraCapture equivalent.
	 * @param cameraName name of the camera to stram. Can be an empty string.
	 */
	explicit Reader(const std::string &cameraName);

	/**
	 * Retrieve frame stream resolution.
	 * @return Frame stream resolution or `std::nullopt` if the frame stream is not available.
	 */
	[[nodiscard]] std::optional<cv::Size> getFrameResolution() const;

	/**
	 * Get event stream resolution.
	 * @return 		Event stream resolution.
	 */
	[[nodiscard]] std::optional<cv::Size> getEventResolution() const;

	/**
	 * Parse and retrieve next event batch.
	 * @return 		Event batch or `std::nullopt` if no events were received since last read.
	 */
	[[nodiscard]] std::optional<dv::EventStore> getNextEventBatch();

	/**
	 * Parse and retrieve next IMU data batch.
	 * @return 		IMU data batch or `std::nullopt` if no IMU data was received since last read.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::IMU>> getNextImuBatch();

	/**
	 * Parse and retrieve next frame.
	 * @return 		Frame or `std::nullopt` if no frames were received since last read.
	 */
	[[nodiscard]] std::optional<dv::Frame> getNextFrame();

	/**
	 * Parse and retrieve next trigger data batch.
	 * @return 		Trigger data batch or `std::nullopt` if no triggers were received since last read.
	 */
	[[nodiscard]] std::optional<dv::cvector<dv::Trigger>> getNextTriggerBatch();

	/**
	 * Checks whether frame data stream is present in the file.
	 * @return 		True if the frames are available, false otherwise.
	 */
	[[nodiscard]] bool isFrameStreamAvailable() const;

	/**
	 * Checks whether event data stream is present in the file.
	 * @return 		True if the events are available, false otherwise. It is always true for dv::io::CameraCapture.
	 */
	[[nodiscard]] bool isEventStreamAvailable() const;

	/**
	 * Checks whether event data stream is present in the file.
	 * @return 		True if the imu data is available, false otherwise. It is always true for dv::io::CameraCapture.
	 */
	[[nodiscard]] bool isImuStreamAvailable() const;

	/**
	 * Checks whether event data stream is present in the file.
	 * @return 		True if the triggers are available, false otherwise. It is always true for dv::io::CameraCapture.
	 */
	[[nodiscard]] bool isTriggerStreamAvailable() const;

	/**
	 * Return a pair containing start (first) and end (second) time of the recording file.
	 * @return 		A pair containing start and end timestamps for the recording.
	 */
	[[nodiscard]] std::optional<std::pair<int64_t, int64_t>> getTimeRange() const;

	[[nodiscard]] bool isConnected() const;
	[[nodiscard]] const std::unique_ptr<dv::io::CameraCapture> &getCameraCapturePtr() const;
	[[nodiscard]] const std::unique_ptr<dv::io::MonoCameraRecording> &getMonoCameraRecordingPtr() const;
	[[nodiscard]] std::string getCameraName() const;

private:
	// construct unique pointers at construction times
	std::unique_ptr<dv::io::CameraCapture> cameraCapturePtr;
	bool mCameraCapture = false;
	std::unique_ptr<dv::io::MonoCameraRecording> monoCameraRecordingPtr;
};
} // namespace dv_ros_node
