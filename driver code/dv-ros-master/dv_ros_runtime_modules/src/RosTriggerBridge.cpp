#include <dv_ros_messaging/messaging.hpp>

#include <ros/ros.h>

#include <dv-sdk/module.hpp>

class RosImageBridge : public dv::ModuleBase {
private:
	std::unique_ptr<ros::NodeHandle> nh = nullptr;
	std::unique_ptr<ros::Publisher> pub = nullptr;

public:
	static void initInputs(dv::InputDefinitionList &in) {
		in.addTriggerInput("sync");
	}

	static const char *initDescription() {
		return ("Publishes trigger data as ROS publisher.");
	}

	static void initConfigOptions(dv::RuntimeConfig &config) {
		config.add("topicName", dv::ConfigOption::stringOption("ROS Topic name", "/sync"));
		config.setPriorityOptions({"topicName"});
	}

	void configUpdate() override {
		ModuleBase::configUpdate();
	}

	void run() override {
		if (!pub) {
			char **argv = {nullptr};
			int argc    = 0;
			ros::init(argc, argv, "DVS_Trigger_Publisher");

			nh  = std::make_unique<ros::NodeHandle>();
			pub = std::make_unique<ros::Publisher>(
				nh->advertise<dv_ros_msgs::TriggerMessage>(config.getString("topicName"), 1));
		}

		auto triggers = inputs.getTriggerInput("sync");
		if (auto input = triggers.data()) {
			for (const auto &trigger : input) {
				dv_ros_msgs::TriggerMessage msg;
				pub->publish(dv_ros_msgs::toRosTriggerMessage(trigger));
			}
		}

		ros::spinOnce();
	}
};

registerModuleClass(RosImageBridge)
