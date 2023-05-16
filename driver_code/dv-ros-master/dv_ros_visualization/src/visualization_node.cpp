#include <dv-processing/processing.hpp>

#include <dv_ros_messaging/messaging.hpp>

#include <ros/ros.h>

#include <chrono>
#include <thread>

int main(int argc, char **argv) {
	using namespace std::chrono_literals;

	// Initialize the node
	ros::init(argc, argv, "visualization_node");

	// Start node
	ros::NodeHandle nh("~");

	dv::EventStreamSlicer slicer;
	std::unique_ptr<dv::visualization::EventVisualizer> visualizer = nullptr;

	//订阅事件
	auto eventSubscriber
		= nh.subscribe<dv_ros_msgs::EventArrayMessage>("events", 200, [&slicer, &visualizer](const auto &events) {
			  if (visualizer == nullptr) {
				  visualizer
					//   = std::make_unique<dv::visualization::EventVisualizer>(cv::Size(events->width, events->height));
					= std::make_unique<dv::visualization::EventVisualizer>(cv::Size(events->width, events->height), 
					cv::Scalar(0,0,0),//背景为黑
					cv::Scalar(255,0,0),//正极为蓝
					cv::Scalar (0,0,255)//负极为红
					);//按原来的背景为黑，正极为蓝，负极为灰
			  }

			  try {
				  slicer.accept(dv_ros_msgs::toEventStore(*events));
			  }
			  catch (std::out_of_range &exception) {
				  ROS_WARN("%s", exception.what());
			  }
		  });

	//发布图像
	auto framePublisher = nh.advertise<dv_ros_msgs::ImageMessage>("image", 10);

	slicer.doEveryTimeInterval(33ms, [&framePublisher, &visualizer](const dv::EventStore &events) {
		if (visualizer != nullptr) {
			cv::Mat image                 = visualizer->generateImage(events);
			dv_ros_msgs::ImageMessage msg = dv_ros_msgs::toRosImageMessage(image);
			msg.header.stamp              = dv_ros_msgs::toRosTime(events.getLowestTime());
			framePublisher.publish(msg);
		}
	});

	// Sping ros
	while (ros::ok()) {
		ros::spinOnce();
		std::this_thread::sleep_for(1ms);
	}

	return 0;
}
