#include <dv_ros_tracker/tracker_node.h>

using namespace dv_tracker_node;

int main(int argc, char **argv) {
	// Initialize the node
	ros::init(argc, argv, "tracker_node");

	// Start node
	ros::NodeHandle nh("~");
	auto node = TrackerNode(nh);

	// Sping ros
	ros::spin();

	return 0;
}
