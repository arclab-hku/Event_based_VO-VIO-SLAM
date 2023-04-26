#include <dv-processing/core/frame.hpp>

#include <dv_ros_messaging/messaging.hpp>

#include <dv_ros_accumulation/EdgeMapConfig.h>

#include <ros/ros.h>

#include <chrono>
#include <dynamic_reconfigure/server.h>
#include <thread>

int main(int argc, char **argv) {
	using namespace std::chrono_literals;

	// Initialize the node
	ros::init(argc, argv, "edge_map");

	// Start node
	ros::NodeHandle nh("~");

	dv::EventStreamSlicer slicer;
	std::unique_ptr<dv::PixelAccumulator> accumulator = nullptr;

	dynamic_reconfigure::Server<dv_ros_accumulation::EdgeMapConfig> server;
	auto framePublisher = nh.advertise<dv_ros_msgs::ImageMessage>("image", 10);

	const auto slicerCallback = [&framePublisher, &accumulator](const dv::EventStore &events) {
		if (accumulator != nullptr) {
			accumulator->accept(events);
			framePublisher.publish(dv_ros_msgs::frameToRosImageMessage(accumulator->generateFrame()));
		}
	};

	std::optional<int> jobId;
	const auto reconfigureCallback = [&accumulator, &slicer, &jobId, &slicerCallback](
										 const dv_ros_accumulation::EdgeMapConfig &config, uint32_t level) {
		if (jobId.has_value()) {
			slicer.removeJob(*jobId);
		}

		if (config.enable_decay) {
			accumulator->setDecay(static_cast<float>(config.decay));
		}
		else {
			accumulator->setDecay(-1.f);
		}
		accumulator->setIgnorePolarity(config.rectify_polarity);
		accumulator->setContribution(static_cast<float>(config.event_contribution));
		accumulator->setNeutralValue(static_cast<float>(config.neutral_potential));

		switch (config.slice_method) {
			case dv_ros_accumulation::EdgeMap_TIME: {
				jobId = slicer.doEveryTimeInterval(dv::Duration(config.accumulation_time * 1000LL), slicerCallback);
				break;
			}
			case dv_ros_accumulation::EdgeMap_NUMBER: {
				jobId = slicer.doEveryNumberOfEvents(config.accumulation_number, slicerCallback);
				break;
			}
			default:
				throw dv::exceptions::InvalidArgument<int>("Unknown slicing method id", config.slice_method);
		}
	};

	auto eventSubscriber = nh.subscribe<dv_ros_msgs::EventArrayMessage>(
		"events", 200, [&slicer, &accumulator, &server, &reconfigureCallback](const auto &events) {
			if (accumulator == nullptr) {
				accumulator = std::make_unique<dv::PixelAccumulator>(cv::Size(events->width, events->height));
				server.setCallback(reconfigureCallback);
			}

			try {
				slicer.accept(dv_ros_msgs::toEventStore(*events));
			}
			catch (std::out_of_range &exception) {
				ROS_WARN("%s", exception.what());
			}
		});

	// Sping ros
	while (ros::ok()) {
		ros::spinOnce();
		std::this_thread::sleep_for(1ms);
	}

	return 0;
}
