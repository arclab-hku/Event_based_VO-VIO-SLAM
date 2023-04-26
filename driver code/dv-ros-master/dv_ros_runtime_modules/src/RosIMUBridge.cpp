#include <dv_ros_messaging/messaging.hpp>

#include <ros/ros.h>

#include <dv-sdk/module.hpp>

class RosIMUBridge : public dv::ModuleBase {
private:
	std::unique_ptr<ros::NodeHandle> nh = nullptr;
	std::unique_ptr<ros::Publisher> pub = nullptr;

public:
	static void initInputs(dv::InputDefinitionList &in) {
		in.addIMUInput("imu");
	}

	static const char *initDescription() {
		return ("Publishes IMU measurements into ROS.");
	}

	static void initConfigOptions(dv::RuntimeConfig &config) {
		config.add("topicName", dv::ConfigOption::stringOption("ROS Topic name", "/camera/imu"));
		config.setPriorityOptions({"topicName"});
	}

	void configUpdate() override {
		ModuleBase::configUpdate();
	}

	void run() override {
		if (!pub) {
			char **argv = {nullptr};
			int argc    = 0;
			ros::init(argc, argv, "DVS_IMU_Publisher");

			nh  = std::make_unique<ros::NodeHandle>();
			pub = std::make_unique<ros::Publisher>(
				nh->advertise<dv_ros_msgs::ImuMessage>(config.getString("topicName"), 1));
		}

		auto imuInput = inputs.getIMUInput("imu");
		for (const auto &sample : imuInput.data()) {
			pub->publish(dv_ros_msgs::toRosImuMessage(sample));
		}
		ros::spinOnce();
	}
};

registerModuleClass(RosIMUBridge)
