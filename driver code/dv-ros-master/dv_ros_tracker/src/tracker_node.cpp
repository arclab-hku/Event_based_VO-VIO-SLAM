#include <dv-processing/data/utilities.hpp>
#include <dv-processing/features/event_feature_lk_tracker.hpp>
#include <dv-processing/visualization/pose_visualizer.hpp>

#include <dv_ros_tracker/tracker_node.h>

#include <iostream>
#include <memory>
#include <sensor_msgs/distortion_models.h>

using namespace dv_tracker_node;
using namespace std::chrono_literals;

TrackerNode::TrackerNode(ros::NodeHandle &nodeHandle) : mNodeHandle(nodeHandle) {
	// Publishers
	mTimedKeypointArrayPublisher = mNodeHandle.advertise<TimedKeypointArrayMessage>("keypoints", 100);
	mTimedKeypointUndistortedArrayPublisher
		= mNodeHandle.advertise<TimedKeypointArrayMessage>("keypointsUndistorted", 100);

	// Read the parameters
	mLucasKanadeConfig.maskedFeatureDetect
		= mNodeHandle.param("maskedFeatureDetect", mLucasKanadeConfig.maskedFeatureDetect);
	mLucasKanadeConfig.numPyrLayers = mNodeHandle.param("numPyrLayers", mLucasKanadeConfig.numPyrLayers);
	int windowSize                  = mNodeHandle.param("searchWindowSize", mLucasKanadeConfig.searchWindowSize.width);
	mLucasKanadeConfig.searchWindowSize = cv::Size(windowSize, windowSize);
	mLucasKanadeConfig.terminationEpsilon
		= mNodeHandle.param("terminationEpsilon", mLucasKanadeConfig.terminationEpsilon);
	mTrackingConfig.numIntermediateFrames
		= mNodeHandle.param("numIntermediateFrames", mTrackingConfig.numIntermediateFrames);
	mTrackingConfig.accumulationFramerate
		= mNodeHandle.param("accumulationFramerate", mTrackingConfig.accumulationFramerate);
	mTrackingConfig.fastThreshold     = mNodeHandle.param("fastThreshold", mTrackingConfig.fastThreshold);
	mTrackingConfig.lookbackRejection = mNodeHandle.param("lookbackRejection", mTrackingConfig.lookbackRejection);
	mTrackingConfig.redetectionThreshold
		= mNodeHandle.param("redetectionThreshold", mTrackingConfig.redetectionThreshold);
	mTrackingConfig.maxTracks = mNodeHandle.param("maxTracks", mTrackingConfig.maxTracks);
	mTrackingConfig.numEvents = mNodeHandle.param("numEvents", mTrackingConfig.numEvents);

	bool useEvents = mNodeHandle.param("useEvents", true);
	bool useFrames = mNodeHandle.param("useFrames", true);

	// Define the operation mode
	if (useEvents && useFrames) {
		mode                         = OperationMode::Combined;
		mTracksPreviewPublisher      = mNodeHandle.advertise<dv_ros_msgs::ImageMessage>("preview/image", 10);
		mTracksEventsFramesPublisher = mNodeHandle.advertise<dv_ros_msgs::ImageMessage>("events_preview/image", 10);
	}
	else if (useEvents) {
		mode                         = OperationMode::EventsOnly;
		mTracksEventsFramesPublisher = mNodeHandle.advertise<dv_ros_msgs::ImageMessage>("events_preview/image", 10);
	}
	else if (useFrames) {
		mode                    = OperationMode::FramesOnly;
		mTracksPreviewPublisher = mNodeHandle.advertise<dv_ros_msgs::ImageMessage>("preview/image", 10);
	}
	else {
		throw dv::exceptions::RuntimeError(
			"Neither events nor frames are enabled as input, at least one has to be enabled for the tracker!");
	}

	bool motionCompensation = mNodeHandle.param("useMotionCompensation", false);
	if (motionCompensation) {
		mode = static_cast<OperationMode>(static_cast<int>(mode) + 1);
	}

	// Subscriber
	mFrameInfoSubscriber = mNodeHandle.subscribe("camera_info", 10, &TrackerNode::cameraInfoCallback, this);

	frameTracks.setTrackTimeout(10ms);
}

TrackerNode::~TrackerNode() {
	stop();
}

void TrackerNode::eventsArrayCallback(const dv_ros_msgs::EventArrayMessage::ConstPtr &msgPtr) {
	if (msgPtr == nullptr) {
		return;
	}
	auto events = dv_ros_msgs::toEventStore(*msgPtr);
	mDataQueue.push(std::move(events));
}

void TrackerNode::frameCallback(const dv_ros_msgs::ImageMessage::ConstPtr &msgPtr) {
	if (msgPtr == nullptr) {
		return;
	}
	mDataQueue.push(dv_ros_msgs::FrameMap(msgPtr));
}

void TrackerNode::poseCallback(const PoseStampedMsg::ConstPtr &msgPtr) {
	// if the tracker is not initialized don't care about the messages.
	if (msgPtr == nullptr) {
		return;
	}

	// if the timestamp is not monotonically increasing return.
	auto timestamp = dv_ros_msgs::toDvTime(msgPtr->header.stamp);
	if (lastTransformTime >= timestamp) {
		return;
	}
	lastTransformTime = timestamp;

	Eigen::Vector3f t(msgPtr->pose.position.x, msgPtr->pose.position.y, msgPtr->pose.position.z);

	Eigen::Quaternionf q(
		msgPtr->pose.orientation.w, msgPtr->pose.orientation.x, msgPtr->pose.orientation.y, msgPtr->pose.orientation.z);

	dv::kinematics::Transformationf T_WC = dv::kinematics::Transformationf(timestamp, t, q);
	mDataQueue.push(T_WC);
}

void TrackerNode::depthEstimationCallback(const dv_ros_tracker::Depth::ConstPtr &msgPtr) {
	if (msgPtr == nullptr) {
		return;
	}

	mDepthEstimation = msgPtr->depth;
	auto timestamp   = dv_ros_msgs::toDvTime(msgPtr->timestamp);
	switch (mode) {
		case OperationMode::EventsOnlyCompensated:
			dynamic_cast<dvf::EventFeatureLKTracker<dv::kinematics::MotionCompensator<>> *>(tracker.get())
				->accept(dv::measurements::Depth(timestamp, mDepthEstimation));
			break;
		case OperationMode::FramesOnlyCompensated:
			dynamic_cast<dvf::ImageFeatureLKTracker *>(tracker.get())
				->accept(dv::measurements::Depth(timestamp, mDepthEstimation));
			break;
		case OperationMode::CombinedCompensated:
			dynamic_cast<dvf::EventCombinedLKTracker<dv::kinematics::MotionCompensator<>> *>(tracker.get())
				->accept(dv::measurements::Depth(timestamp, mDepthEstimation));
			break;
		default:
			break;
	}
}

void TrackerNode::cameraInfoCallback(const dv_ros_msgs::CameraInfoMessage::ConstPtr &msgPtr) {
	if (mCameraInitialized) {
		return;
	}
	// read the camera info
	mCameraCalibration            = dv::camera::calibrations::CameraCalibration();
	mCameraCalibration.resolution = cv::Size(static_cast<int>(msgPtr->width), static_cast<int>(msgPtr->height));
	for (const auto &d : msgPtr->D) {
		mCameraCalibration.distortion.push_back(static_cast<float>(d));
	}

	if (static_cast<std::string>(msgPtr->distortion_model) == sensor_msgs::distortion_models::EQUIDISTANT) {
		mCameraCalibration.distortionModel = dv::camera::DistortionModel::Equidistant;
	}
	else if (static_cast<std::string>(msgPtr->distortion_model) == sensor_msgs::distortion_models::PLUMB_BOB) {
		mCameraCalibration.distortionModel = dv::camera::DistortionModel::RadTan;
	}
	else {
		throw dv::exceptions::InvalidArgument<dv_ros_msgs::CameraInfoMessage::_distortion_model_type>(
			"Unknown camera model.", msgPtr->distortion_model);
	}
	mCameraCalibration.focalLength    = cv::Point2f(static_cast<float>(msgPtr->K[0]), static_cast<float>(msgPtr->K[4]));
	mCameraCalibration.principalPoint = cv::Point2f(static_cast<float>(msgPtr->K[2]), static_cast<float>(msgPtr->K[5]));
	mCameraInitialized                = true;

	// crate the tracker according to the config file and the camera info.
	createTracker();

	// Subscribers
	if (mode == OperationMode::FramesOnly || mode == OperationMode::Combined
		|| mode == OperationMode::FramesOnlyCompensated || mode == OperationMode::CombinedCompensated) {
		mFrameSubscriber = mNodeHandle.subscribe("image", 10, &TrackerNode::frameCallback, this);
		ROS_INFO("Subscribing to image stream..");
	}
	if (mode == OperationMode::EventsOnly || mode == OperationMode::Combined
		|| mode == OperationMode::EventsOnlyCompensated || mode == OperationMode::CombinedCompensated) {
		mEventsArraySubscriber = mNodeHandle.subscribe("events", 10, &TrackerNode::eventsArrayCallback, this);
		ROS_INFO("Subscribing to event stream..");
	}
	if (mode == OperationMode::FramesOnlyCompensated || mode == OperationMode::EventsOnlyCompensated
		|| mode == OperationMode::CombinedCompensated) {
		ROS_INFO_STREAM("Subscribe to pose and depth messages..");
		mDepthEstimationSubscriber
			= mNodeHandle.subscribe("depthEstimation", 10, &TrackerNode::depthEstimationCallback, this);
		mTfSubscriber = mNodeHandle.subscribe("pose", 10, &TrackerNode::poseCallback, this);
		ROS_INFO("Tracker with motion compensation..");
	}

	// start the tracking thread
	startTracking();
}

bool TrackerNode::isRunning() const {
	return mSpinThread.load(std::memory_order_relaxed);
}

void TrackerNode::startTracking() {
	ROS_INFO("Spinning tracking node.");
	mSpinThread      = true;
	mKeypointsThread = std::thread(&TrackerNode::assembleTrack, this);
}

void TrackerNode::createTracker() {
	auto detector = std::make_unique<dvf::ImagePyrFeatureDetector>(
		mCameraCalibration.resolution, cv::FastFeatureDetector::create(mTrackingConfig.fastThreshold));

	switch (mode) {
		case OperationMode::EventsOnly: {
			ROS_INFO("Constructing Events Only Tracker..");
			auto eventTracker = dvf::EventFeatureLKTracker<dv::PixelAccumulator>::RegularTracker(
				mCameraCalibration.resolution, mLucasKanadeConfig, nullptr, std::move(detector),
				std::make_unique<dvf::FeatureCountRedetection>(mTrackingConfig.redetectionThreshold));
			eventTracker->setMaxTracks(mTrackingConfig.maxTracks);
			eventTracker->setFramerate(mTrackingConfig.accumulationFramerate);
			eventTracker->setNumberOfEvents(mTrackingConfig.numEvents);
			eventTracker->setLookbackRejection(mTrackingConfig.lookbackRejection);
			tracker = static_unique_ptr_cast<dvf::TrackerBase>(std::move(eventTracker));
			break;
		}
		case OperationMode::EventsOnlyCompensated: {
			ROS_INFO("Constructing Events with Motion Compensation Tracker..");
			auto eventTracker = dvf::EventFeatureLKTracker<dv::kinematics::MotionCompensator<>>::MotionAwareTracker(
				std::make_shared<dv::camera::CameraGeometry>(mCameraCalibration.getCameraGeometry()),
				mLucasKanadeConfig, nullptr, nullptr, std::move(detector),
				std::make_unique<dvf::FeatureCountRedetection>(mTrackingConfig.redetectionThreshold));

			eventTracker->setMaxTracks(mTrackingConfig.maxTracks);
			eventTracker->setFramerate(mTrackingConfig.accumulationFramerate);
			eventTracker->setNumberOfEvents(mTrackingConfig.numEvents);
			eventTracker->setLookbackRejection(mTrackingConfig.lookbackRejection);
			tracker = static_unique_ptr_cast<dvf::TrackerBase>(std::move(eventTracker));
			break;
		}
		case OperationMode::FramesOnly: {
			ROS_INFO("Constructing Frames Only Tracker..");
			auto frameTracker = dvf::ImageFeatureLKTracker::RegularTracker(mCameraCalibration.resolution,
				mLucasKanadeConfig, std::move(detector),
				std::make_unique<dvf::FeatureCountRedetection>(mTrackingConfig.redetectionThreshold));
			frameTracker->setMaxTracks(mTrackingConfig.maxTracks);
			frameTracker->setLookbackRejection(mTrackingConfig.lookbackRejection);
			tracker = static_unique_ptr_cast<dvf::TrackerBase>(std::move(frameTracker));
			break;
		}
		case OperationMode::FramesOnlyCompensated: {
			ROS_INFO("Constructing Frames with Motion Compensation Tracker..");
			auto frameTracker = dvf::ImageFeatureLKTracker::MotionAwareTracker(
				std::make_shared<dv::camera::CameraGeometry>(mCameraCalibration.getCameraGeometry()),
				mLucasKanadeConfig, nullptr, std::move(detector),
				std::make_unique<dvf::FeatureCountRedetection>(mTrackingConfig.redetectionThreshold));
			frameTracker->setMaxTracks(mTrackingConfig.maxTracks);
			frameTracker->setLookbackRejection(mTrackingConfig.lookbackRejection);
			tracker = static_unique_ptr_cast<dvf::TrackerBase>(std::move(frameTracker));
			break;
		}

		case OperationMode::Combined: {
			ROS_INFO("Constructing Combined Tracker..");
			auto combinedTracker = dvf::EventCombinedLKTracker<dv::PixelAccumulator>::RegularTracker(
				mCameraCalibration.resolution, mLucasKanadeConfig, nullptr, std::move(detector),
				std::make_unique<dvf::FeatureCountRedetection>(mTrackingConfig.redetectionThreshold));
			combinedTracker->setMaxTracks(mTrackingConfig.maxTracks);
			combinedTracker->setNumberOfEvents(mTrackingConfig.numEvents);
			combinedTracker->setNumIntermediateFrames(mTrackingConfig.numIntermediateFrames);
			combinedTracker->setLookbackRejection(mTrackingConfig.lookbackRejection);
			tracker = static_unique_ptr_cast<dvf::TrackerBase>(std::move(combinedTracker));
			break;
		}
		case OperationMode::CombinedCompensated: {
			ROS_INFO("Constructing Combined with Motion Compensation Tracker..");
			auto combinedTracker = dvf::EventCombinedLKTracker<dv::kinematics::MotionCompensator<>>::MotionAwareTracker(
				std::make_shared<dv::camera::CameraGeometry>(mCameraCalibration.getCameraGeometry()),
				mLucasKanadeConfig, nullptr, nullptr, std::move(detector),
				std::make_unique<dvf::FeatureCountRedetection>(mTrackingConfig.redetectionThreshold));
			combinedTracker->setMaxTracks(mTrackingConfig.maxTracks);
			combinedTracker->setNumberOfEvents(mTrackingConfig.numEvents);
			combinedTracker->setNumIntermediateFrames(mTrackingConfig.numIntermediateFrames);
			combinedTracker->setLookbackRejection(mTrackingConfig.lookbackRejection);
			tracker = static_unique_ptr_cast<dvf::TrackerBase>(std::move(combinedTracker));
			break;
		}
	}
}

void TrackerNode::pushEventToTracker(const dv::EventStore &events) {
	switch (mode) {
		case OperationMode::EventsOnly:
			dynamic_cast<dvf::EventFeatureLKTracker<dv::PixelAccumulator> *>(tracker.get())->accept(events);
			break;
		case OperationMode::EventsOnlyCompensated:
			dynamic_cast<dvf::EventFeatureLKTracker<dv::kinematics::MotionCompensator<>> *>(tracker.get())
				->accept(events);
			break;
		case OperationMode::Combined:
			dynamic_cast<dvf::EventCombinedLKTracker<dv::PixelAccumulator> *>(tracker.get())->accept(events);
			break;
		case OperationMode::CombinedCompensated:
			dynamic_cast<dvf::EventCombinedLKTracker<dv::kinematics::MotionCompensator<>> *>(tracker.get())
				->accept(events);
			break;
		default:
			// Noop,
			break;
	}
}

void TrackerNode::pushFrameToTracker(const dv::Frame &frame) {
	switch (mode) {
		case OperationMode::FramesOnly:
			dynamic_cast<dvf::ImageFeatureLKTracker *>(tracker.get())->accept(frame);
			break;
		case OperationMode::FramesOnlyCompensated:
			dynamic_cast<dvf::ImageFeatureLKTracker *>(tracker.get())->accept(frame);
			break;
		case OperationMode::Combined:
			dynamic_cast<dvf::EventCombinedLKTracker<dv::PixelAccumulator> *>(tracker.get())->accept(frame);
			break;
		case OperationMode::CombinedCompensated:
			dynamic_cast<dvf::EventCombinedLKTracker<dv::kinematics::MotionCompensator<>> *>(tracker.get())
				->accept(frame);
			break;
		default:
			// Noop
			break;
	}
}

void TrackerNode::pushTransformToTracker(const dv::kinematics::Transformationf &transform) {
	switch (mode) {
		case OperationMode::EventsOnlyCompensated:
			dynamic_cast<dvf::EventFeatureLKTracker<dv::kinematics::MotionCompensator<>> *>(tracker.get())
				->accept(transform);
			break;
		case OperationMode::FramesOnlyCompensated:
			dynamic_cast<dvf::ImageFeatureLKTracker *>(tracker.get())->accept(transform);
			break;
		case OperationMode::CombinedCompensated:
			dynamic_cast<dvf::EventCombinedLKTracker<dv::kinematics::MotionCompensator<>> *>(tracker.get())
				->accept(transform);
			break;
		default:
			break;
	}
}

dv::cvector<dv::TimedKeyPoint> TrackerNode::undistortKeypoints(const dv::cvector<dv::TimedKeyPoint> &keypoints) {
	static const auto mCamGeom                          = mCameraCalibration.getCameraGeometry();
	dv::cvector<dv::TimedKeyPoint> undistortedKeypoints = keypoints;
	for (auto &keypoint : undistortedKeypoints) {
		keypoint.pt = mCamGeom.undistort<dv::Point2f>(keypoint.pt);
	}
	return undistortedKeypoints;
}

bool TrackerNode::runTracking() {
	if (auto tracks = tracker->runTracking(); tracks != nullptr) {
		frameTracks.accept(tracks);
		mTimedKeypointArrayPublisher.publish(toRosTimedKeypointArrayMessage(tracks->timestamp, tracks->keypoints));

		mTimedKeypointUndistortedArrayPublisher.publish(
			toRosTimedKeypointArrayMessage(tracks->timestamp, undistortKeypoints(tracks->keypoints)));
		return true;
	}
	return false;
}

void TrackerNode::publishEventsPreview(const cv::Mat &background) {
	if (mTracksEventsFramesPublisher.getNumSubscribers() > 0 && !background.empty()) {
		mTracksEventsFramesPublisher.publish(dv_ros_msgs::toRosImageMessage(frameTracks.visualize(background)));
	}
}

void TrackerNode::publishPreview(const cv::Mat &background) {
	if (mTracksPreviewPublisher.getNumSubscribers() > 0 && !background.empty()) {
		mTracksPreviewPublisher.publish(dv_ros_msgs::toRosImageMessage(frameTracks.visualize(background)));
	}
}

void TrackerNode::manageEventsQueue(const dv::EventStore &events) {
	switch (mode) {
		case OperationMode::EventsOnly: {
			pushEventToTracker(events);
			while (runTracking()) {
				cv::Mat accumulatedImage
					= dynamic_cast<dvf::EventFeatureLKTracker<dv::PixelAccumulator> *>(tracker.get())
						  ->getAccumulatedFrame();
				// publish accumulated image
				publishEventsPreview(accumulatedImage);
			}
			break;
		}
		case OperationMode::Combined: {
			pushEventToTracker(events);
			mLastEventsTimestamp = events.getHighestTime();
			// synchronize frames and events
			while (!queueFrame.empty() && queueFrame.front().frame.timestamp < mLastEventsTimestamp) {
				pushFrameToTracker(queueFrame.front().frame);
				runTracking();
				auto frames = dynamic_cast<dvf::EventCombinedLKTracker<dv::PixelAccumulator> *>(tracker.get())
								  ->getAccumulatedFrames();
				if (!frames.empty()) {
					cv::Mat accumulatedImage = frames.back().pyramid.front();
					// publish accumulated image
					publishEventsPreview(accumulatedImage);
				}
				// publish the tracks on the frame
				publishPreview(queueFrame.front().frame.image);

				// remove the used data from the queue
				queueFrame.pop();
			}
			break;
		}
			// same behaviour for the two cases
		case OperationMode::EventsOnlyCompensated:
		case OperationMode::CombinedCompensated:
			// Store events batch in the queue for synchronization
			queueEventStore.push(events);
			break;
		default:
			break;
	}
}

void TrackerNode::manageFramesQueue(const dv_ros_msgs::FrameMap &map) {
	if (mode == OperationMode::FramesOnly) {
		// Perform tracking and publish the results
		pushFrameToTracker(map.frame);
		runTracking();
		publishPreview(map.frame.image);
	}
	else {
		// store frame in the queue for synchronization
		queueFrame.push(map);
	}
}

void TrackerNode::manageTransformsQueue(const dv::kinematics::Transformationf &transform) {
	pushTransformToTracker(transform);
	switch (mode) {
		case OperationMode::FramesOnlyCompensated: {
			// Synchronize frames and transforms
			while (!queueFrame.empty() && queueFrame.front().frame.timestamp < lastTransformTime) {
				pushFrameToTracker(queueFrame.front().frame);
				runTracking();
				publishPreview(queueFrame.front().frame.image);

				queueFrame.pop();
			}
			break;
		}
		case OperationMode::EventsOnlyCompensated: {
			// Synchronize events and transforms
			while (!queueEventStore.empty() && queueEventStore.front().getHighestTime() < lastTransformTime) {
				pushEventToTracker(queueEventStore.front());
				mLastEventsTimestamp = queueEventStore.front().getHighestTime();
				queueEventStore.pop();
				while (runTracking()) {
					cv::Mat accumulatedImage
						= dynamic_cast<dvf::EventFeatureLKTracker<dv::kinematics::MotionCompensator<>> *>(tracker.get())
							  ->getAccumulatedFrame();
					publishEventsPreview(accumulatedImage);
				}
			}
			break;
		}
		case OperationMode::CombinedCompensated: {
			// Synchronize events and transforms
			while (!queueEventStore.empty() && queueEventStore.front().getHighestTime() < lastTransformTime) {
				pushEventToTracker(queueEventStore.front());
				mLastEventsTimestamp = queueEventStore.front().getHighestTime();
				queueEventStore.pop();
				// Synchronize frames and transforms
				while (!queueFrame.empty() && queueFrame.front().frame.timestamp < mLastEventsTimestamp) {
					pushFrameToTracker(queueFrame.front().frame);
					// perform tracking and publish the results
					runTracking();
					auto tmpTracker = dynamic_cast<dvf::EventCombinedLKTracker<dv::kinematics::MotionCompensator<>> *>(
						tracker.get());
					auto frames = tmpTracker->getAccumulatedFrames();
					if (!frames.empty()) {
						cv::Mat accumulatedImage = frames.back().pyramid.front();
						publishEventsPreview(accumulatedImage);
					}
					publishPreview(queueFrame.front().frame.image);

					queueFrame.pop();
				}
			}
			break;
		}
		default:
			break;
	}
}

void TrackerNode::assembleTrack() {
	while (mSpinThread) {
		mDataQueue.consume_all([&](const auto &data) {
			// read events.
			if (const dv::EventStore *events = std::get_if<dv::EventStore>(&data); events != nullptr) {
				manageEventsQueue(*events);
			}
			// read frames
			else if (const dv_ros_msgs::FrameMap *map = std::get_if<dv_ros_msgs::FrameMap>(&data); map != nullptr) {
				manageFramesQueue(*map);
			}
			// read transform
			else if (const dv::kinematics::Transformationf *transform
					 = std::get_if<dv::kinematics::Transformationf>(&data);
					 transform != nullptr) {
				manageTransformsQueue(*transform);
			}
			else {
				throw std::runtime_error("Wrong type in queue.");
			}
		});
		std::this_thread::sleep_for(100us);
	}
}

void TrackerNode::stop() {
	ROS_INFO("Stopping the tracking node.");

	mSpinThread = false;
	mKeypointsThread.join();
}
