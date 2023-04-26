#include <dv-processing/core/frame.hpp>

#include <dv_ros_messaging/messaging.hpp>

#include <dv_ros_accumulation/AccumulatorConfig.h>

#include <boost/lockfree/spsc_queue.hpp>

#include <ros/ros.h>

#include <chrono>
#include <dynamic_reconfigure/server.h>
#include <thread>

int main(int argc, char **argv) {
	using namespace std::chrono_literals;

	// Initialize the node
	ros::init(argc, argv, "accumulator");

	// Start node
	ros::NodeHandle nh("~");

	dv::EventStreamSlicer slicer;
	std::unique_ptr<dv::Accumulator> accumulator = nullptr;

	dynamic_reconfigure::Server<dv_ros_accumulation::AccumulatorConfig> server;
	auto framePublisher = nh.advertise<dv_ros_msgs::ImageMessage>("image", 10);

	boost::lockfree::spsc_queue<dv::EventStore> eventQueue(100);

	std::atomic<bool> continueThread = true;

	std::thread accumulationThread([&framePublisher, &accumulator, &eventQueue, &continueThread] {
		while (continueThread) {
			if (accumulator != nullptr) {
				eventQueue.consume_all([&framePublisher, &accumulator](const dv::EventStore &events) {
					accumulator->accept(events);
					dv::Frame frame               = accumulator->generateFrame();
					dv_ros_msgs::ImageMessage msg = dv_ros_msgs::toRosImageMessage(frame.image);
					msg.header.stamp              = dv_ros_msgs::toRosTime(frame.timestamp);
					framePublisher.publish(msg);
				});
			}
			std::this_thread::sleep_for(100us);
		}
	});

	const auto slicerCallback = [&eventQueue](const dv::EventStore &events) {
		eventQueue.push(events);
	};

	std::optional<int> jobId;
	const auto reconfigureCallback = [&accumulator, &slicer, &jobId, &slicerCallback](
										 const dv_ros_accumulation::AccumulatorConfig &config, uint32_t level) {
		if (jobId.has_value()) {
			slicer.removeJob(*jobId);
		}

		accumulator->setEventContribution(static_cast<float>(config.event_contribution));
		accumulator->setMaxPotential(static_cast<float>(config.max_potential));
		accumulator->setMinPotential(static_cast<float>(config.min_potential));
		accumulator->setNeutralPotential(static_cast<float>(config.neutral_potential));
		accumulator->setDecayParam(static_cast<float>(config.decay_param));
		accumulator->setRectifyPolarity(config.rectify_polarity);
		accumulator->setSynchronousDecay(config.synchronous_decay);
		accumulator->setDecayFunction(static_cast<dv::Accumulator::Decay>(config.decay_function));

		switch (config.slice_method) {
			case dv_ros_accumulation::Accumulator_TIME: {
				jobId = slicer.doEveryTimeInterval(dv::Duration(config.accumulation_time * 1000LL), slicerCallback);
				break;
			}
			case dv_ros_accumulation::Accumulator_NUMBER: {
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
				accumulator = std::make_unique<dv::Accumulator>(cv::Size(events->width, events->height));
				server.setCallback(reconfigureCallback);
			}
			auto store = dv_ros_msgs::toEventStore(*events);

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
		std::this_thread::sleep_for(100us);
	}

	continueThread = false;
	accumulationThread.join();

	return 0;
}
