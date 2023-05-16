#include <dv_ros_capture/capture_node.hpp>
#include <dv_ros_capture/parameters_loader.hpp>

#include <thread>

using namespace dv_capture_node;

int main(int argc, char **argv) {
	using namespace std::chrono_literals;

	// Initialize the node
	ros::init(argc, argv, "capture_node");

	// Start node
	auto nh = std::make_shared<ros::NodeHandle>("~");

	auto loadParams = dv_ros_node::ParametersLoader(*nh);//读入参数
	loadParams.printConfiguration();//打印参数

	auto node = CaptureNode(nh, loadParams.getParams());//初始化节点,同时传入参数

	// Spin the node
	node.startCapture();//开始读取数据
	// Sping ros
	while (ros::ok() && node.isRunning()) {
		ros::spinOnce();
		std::this_thread::sleep_for(1ms);
	}

	return 0;
}
