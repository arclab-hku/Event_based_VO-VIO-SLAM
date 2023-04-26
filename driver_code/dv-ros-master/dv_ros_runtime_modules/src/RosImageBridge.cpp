#include <dv_ros_messaging/messaging.hpp>

#include <ros/ros.h>

#include <dv-sdk/module.hpp>

class RosImageBridge : public dv::ModuleBase {
private:
	std::unique_ptr<ros::NodeHandle> nh = nullptr;
	ros::Publisher pub;

public:
	static void initInputs(dv::InputDefinitionList &in) {
		in.addFrameInput("image");
	}

	static const char *initDescription() {
		return ("Publishes image frames into ROS.");
	}

	static void initConfigOptions(dv::RuntimeConfig &config) {
		config.add("topicName", dv::ConfigOption::stringOption("ROS Topic name", "/camera/image"));
		config.setPriorityOptions({"topicName"});
	}

	void configUpdate() override {
		ModuleBase::configUpdate();
	}

	void run() override {
		if (!pub) {
			char **argv = {nullptr};
			int argc    = 0;
			ros::init(argc, argv, "DVS_Image_Publisher");

			nh  = std::make_unique<ros::NodeHandle>();
			pub = nh->advertise<dv_ros_msgs::ImageMessage>(config.getString("topicName"), 10);
		}

		auto frame = inputs.getFrameInput("image");
		pub.publish(dv_ros_msgs::frameToRosImageMessage(*frame.frame().getBasePointer()));
		ros::spinOnce();
	}
};

registerModuleClass(RosImageBridge)
