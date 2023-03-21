#include <utility.hpp>

#define TS_DIFF_THLD  0.01  // 99% - 101%
#define EVENT_SKIP_TS 5.0   // [s]

// ------------------------------------------------------------------------- //

bool receive_imu                    = false;
bool receive_event_left             = false;
bool receive_event_right            = false;
bool receive_camera_left            = false;
bool receive_camera_right           = false;
bool receive_kinect_color           = false;
bool receive_kinect_depth           = false;
bool receive_lidar                  = false;
bool receive_gt                     = false;
bool receive_event_left_depth       = false;
bool receive_event_right_depth      = false;
bool receive_camera_left_depth      = false;
bool receive_camera_right_depth     = false;
bool receive_camera_left_undistort  = false;
bool receive_camera_right_undistort = false;

double   prev_imu_ts                    = 0;
uint64_t prev_event_left_ts             = 0;
double   prev_event_left_array_ts       = 0;
uint64_t prev_event_right_ts            = 0;
double   prev_event_right_array_ts      = 0;
double   prev_camera_left_ts            = 0;
double   prev_camera_right_ts           = 0;
double   prev_kinect_color_ts           = 0;
double   prev_kinect_depth_ts           = 0;
double   prev_lidar_ts                  = 0;
double   prev_gt_ts                     = 0;
double   prev_event_left_depth_ts       = 0;
double   prev_event_right_depth_ts      = 0;
double   prev_camera_left_depth_ts      = 0;
double   prev_camera_right_depth_ts     = 0;
double   prev_camera_left_undistort_ts  = 0;
double   prev_camera_right_undistort_ts = 0;

std::vector<double> event_left_ts;
std::vector<size_t> event_left_size;
std::vector<double> event_right_ts;
std::vector<size_t> event_right_size;

// ------------------------------------------------------------------------- //

// For every sensor handler, it contains the following features:
//   (1) Check whether the bag contains this type of sensor message
//   (2) Check whether there is an unusual frequency on the stream (i.e. some message is lost)
//   (3) Check whether there is a chronological error caused by synchronization
//   (4*) For event handler, count the number of events for future calculation of the Mean Event Rate.

void ImuHandler(const IMU::ConstPtr & imu_message) {
  if (!receive_imu) {
    receive_imu = true;
    prev_imu_ts = imu_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives data from the IMU!").c_str());
  } else {
    double imu_ts_diff = imu_message->header.stamp.toSec() - prev_imu_ts;
    if (imu_ts_diff < (1 - TS_DIFF_THLD) * IMU_PERIOD || imu_ts_diff > (1 + TS_DIFF_THLD) * IMU_PERIOD)
      ROS_WARN("%s", colorful_char::warning("IMU stream detects an unusual frequency!").c_str());
    if (imu_ts_diff < 0) ROS_WARN("%s", colorful_char::warning("IMU stream detects a chronological error!").c_str());
    prev_imu_ts = imu_message->header.stamp.toSec();
  }
}

void EventLeftHandler(const EventArray::ConstPtr & event_array_message) {
  if (!receive_event_left) {
    receive_event_left       = true;
    prev_event_left_array_ts = event_array_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives data from the Left Event Camera!").c_str());
  } else {
    double left_event_array_ts_diff = event_array_message->header.stamp.toSec() - prev_event_left_array_ts;
    if (left_event_array_ts_diff < 0)
      ROS_WARN("%s", colorful_char::warning("Left Event Array stream detects a chronological error!").c_str());
    prev_event_left_array_ts = event_array_message->header.stamp.toSec();
  }
  for (auto & event : event_array_message->events) {
    if (event.ts.toNSec() >= prev_event_left_ts) prev_event_left_ts = event.ts.toNSec();
    else
      ROS_WARN_ONCE("%s", colorful_char::warning("Left Event stream detects a chronological error!").c_str());
  }
  event_left_ts.emplace_back(event_array_message->header.stamp.toSec());
  event_left_size.emplace_back(event_array_message->events.size());
}

void EventRightHandler(const EventArray::ConstPtr & event_array_message) {
  if (!receive_event_right) {
    receive_event_right       = true;
    prev_event_right_array_ts = event_array_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives data from the Right Event Camera!").c_str());
  } else {
    double right_event_array_ts_diff = event_array_message->header.stamp.toSec() - prev_event_right_array_ts;
    if (right_event_array_ts_diff < 0)
      ROS_WARN("%s", colorful_char::warning("Right Event Array stream detects a chronological error!").c_str());
    prev_event_right_array_ts = event_array_message->header.stamp.toSec();
  }
  for (auto & event : event_array_message->events) {
    if (event.ts.toNSec() >= prev_event_right_ts) prev_event_right_ts = event.ts.toNSec();
    else
      ROS_WARN_ONCE("%s", colorful_char::warning("Right Event stream detects a chronological error!").c_str());
  }
  event_right_ts.emplace_back(event_array_message->header.stamp.toSec());
  event_right_size.emplace_back(event_array_message->events.size());
}

void CameraLeftHandler(const Image::ConstPtr & image_message) {
  if (!receive_camera_left) {
    receive_camera_left = true;
    prev_camera_left_ts = image_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives data from the Left Regular Camera!").c_str());
  } else {
    double left_camera_ts_diff = image_message->header.stamp.toSec() - prev_camera_left_ts;
    if (left_camera_ts_diff < (1 - TS_DIFF_THLD) * CAM_PERIOD || left_camera_ts_diff > (1 + TS_DIFF_THLD) * CAM_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Left Image stream detects an unusual frequency!").c_str());
    if (left_camera_ts_diff < 0) ROS_WARN("%s", colorful_char::warning("Left Image stream detects a chronological error!").c_str());
    prev_camera_left_ts = image_message->header.stamp.toSec();
  }
}

void CameraRightHandler(const Image::ConstPtr & image_message) {
  if (!receive_camera_right) {
    receive_camera_right = true;
    prev_camera_right_ts = image_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives data from the Right Regular Camera!").c_str());
  } else {
    double right_camera_ts_diff = image_message->header.stamp.toSec() - prev_camera_right_ts;
    if (right_camera_ts_diff < (1 - TS_DIFF_THLD) * CAM_PERIOD || right_camera_ts_diff > (1 + TS_DIFF_THLD) * CAM_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Right Image stream detects an unusual frequency!").c_str());
    if (right_camera_ts_diff < 0) ROS_WARN("%s", colorful_char::warning("Right Image stream detects a chronological error!").c_str());
    prev_camera_right_ts = image_message->header.stamp.toSec();
  }
}

void KinectColorHandler(const Image::ConstPtr & image_message) {
  if (!receive_kinect_color) {
    receive_kinect_color = true;
    prev_kinect_color_ts = image_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives data from the Kinect Color Camera!").c_str());
  } else {
    double kinect_color_ts_diff = image_message->header.stamp.toSec() - prev_kinect_color_ts;
    if (kinect_color_ts_diff > (2 - TS_DIFF_THLD) * KINECT_PERIOD && kinect_color_ts_diff < (2 + TS_DIFF_THLD) * KINECT_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Kinect Color stream detects one lost frame!").c_str());
    else if (kinect_color_ts_diff > (3 - TS_DIFF_THLD) * KINECT_PERIOD && kinect_color_ts_diff < (3 + TS_DIFF_THLD) * KINECT_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Kinect Color stream detects two lost frame!").c_str());
    else if (kinect_color_ts_diff < (1 - TS_DIFF_THLD) * KINECT_PERIOD || kinect_color_ts_diff > (1 + TS_DIFF_THLD) * KINECT_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Kinect Color stream detects an unusual frequency!").c_str());
    if (kinect_color_ts_diff < 0) ROS_WARN("%s", colorful_char::warning("Kinect Color stream detects a chronological error!").c_str());
    prev_kinect_color_ts = image_message->header.stamp.toSec();
  }
}

void KinectDepthHandler(const Image::ConstPtr & image_message) {
  if (!receive_kinect_depth) {
    receive_kinect_depth = true;
    prev_kinect_depth_ts = image_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives data from the Kinect Depth Camera!").c_str());
  } else {
    double kinect_depth_ts_diff = image_message->header.stamp.toSec() - prev_kinect_depth_ts;
    if (kinect_depth_ts_diff > (2 - TS_DIFF_THLD) * KINECT_PERIOD && kinect_depth_ts_diff < (2 + TS_DIFF_THLD) * KINECT_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Kinect Depth stream detects one lost frame!").c_str());
    else if (kinect_depth_ts_diff > (3 - TS_DIFF_THLD) * KINECT_PERIOD && kinect_depth_ts_diff < (3 + TS_DIFF_THLD) * KINECT_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Kinect Depth stream detects two lost frame!").c_str());
    else if (kinect_depth_ts_diff < (1 - TS_DIFF_THLD) * KINECT_PERIOD || kinect_depth_ts_diff > (1 + TS_DIFF_THLD) * KINECT_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Kinect Depth stream detects an unusual frequency!").c_str());
    if (kinect_depth_ts_diff < 0) ROS_WARN("%s", colorful_char::warning("Kinect Depth stream detects a chronological error!").c_str());
    prev_kinect_depth_ts = image_message->header.stamp.toSec();
  }
}

void LidarHandler(const PointCloud::ConstPtr & point_cloud_message) {
  if (!receive_lidar) {
    receive_lidar = true;
    prev_lidar_ts = point_cloud_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives data from the LiDAR!").c_str());
  } else {
    double lidar_ts_diff = point_cloud_message->header.stamp.toSec() - prev_lidar_ts;
    // 98%-102% confidence here!
    if (lidar_ts_diff < (1 - 2 * TS_DIFF_THLD) * LIDAR_PERIOD || lidar_ts_diff > (1 + 2 * TS_DIFF_THLD) * LIDAR_PERIOD)
      ROS_WARN("%s", colorful_char::warning("LiDAR stream detects an unusual frequency!").c_str());
    if (lidar_ts_diff < 0) ROS_WARN("%s", colorful_char::warning("LiDAR stream detects a chronological error!").c_str());
    prev_lidar_ts = point_cloud_message->header.stamp.toSec();
  }
}

void GroundTruthHandler(const PoseStamped::ConstPtr & pose_stamped_message) {
  if (!receive_gt) {
    receive_gt = true;
    prev_gt_ts = pose_stamped_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives data from the Ground Truth Signal!").c_str());
  } else {
    double gt_ts_diff = pose_stamped_message->header.stamp.toSec() - prev_gt_ts;
    if (gt_ts_diff < (1 - TS_DIFF_THLD) * GT_PERIOD || gt_ts_diff > (1 + TS_DIFF_THLD) * GT_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Ground Truth stream detects an unusual frequency!").c_str());
    if (gt_ts_diff < 0) ROS_WARN("%s", colorful_char::warning("Ground Truth stream detects a chronological error!").c_str());
    prev_gt_ts = pose_stamped_message->header.stamp.toSec();
  }
}

void EventLeftDepthHandler(const Image::ConstPtr & image_message) {
  if (!receive_event_left_depth) {
    receive_event_left_depth = true;
    prev_event_left_depth_ts = image_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives reprojected depth data from the Left Event Camera!").c_str());
  } else {
    double event_left_depth_ts_diff = image_message->header.stamp.toSec() - prev_event_left_depth_ts;
    if (event_left_depth_ts_diff < (1 - TS_DIFF_THLD) * KINECT_PERIOD || event_left_depth_ts_diff > (3 + TS_DIFF_THLD) * KINECT_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Left Event Camera's depth stream detects an unusual frequency!").c_str());
    if (event_left_depth_ts_diff < 0)
      ROS_WARN("%s", colorful_char::warning("Left Event Camera's depth stream detects a chronological error!").c_str());
    prev_event_left_depth_ts = image_message->header.stamp.toSec();
  }
}

void EventRightDepthHandler(const Image::ConstPtr & image_message) {
  if (!receive_event_right_depth) {
    receive_event_right_depth = true;
    prev_event_right_depth_ts = image_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives reprojected depth data from the Right Event Camera!").c_str());
  } else {
    double event_right_depth_ts_diff = image_message->header.stamp.toSec() - prev_event_right_depth_ts;
    if (event_right_depth_ts_diff < (1 - TS_DIFF_THLD) * KINECT_PERIOD || event_right_depth_ts_diff > (3 + TS_DIFF_THLD) * KINECT_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Right Event Camera's depth stream detects an unusual frequency!").c_str());
    if (event_right_depth_ts_diff < 0)
      ROS_WARN("%s", colorful_char::warning("Right Event Camera's depth stream detects a chronological error!").c_str());
    prev_event_right_depth_ts = image_message->header.stamp.toSec();
  }
}

void CameraLeftDepthHandler(const Image::ConstPtr & image_message) {
  if (!receive_camera_left_depth) {
    receive_camera_left_depth = true;
    prev_camera_left_depth_ts = image_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives reprojected depth data from the Left Regular Camera!").c_str());
  } else {
    double camera_left_depth_ts_diff = image_message->header.stamp.toSec() - prev_camera_left_depth_ts;
    if (camera_left_depth_ts_diff < (1 - TS_DIFF_THLD) * KINECT_PERIOD || camera_left_depth_ts_diff > (3 + TS_DIFF_THLD) * KINECT_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Left Regular Camera's depth stream detects an unusual frequency!").c_str());
    if (camera_left_depth_ts_diff < 0)
      ROS_WARN("%s", colorful_char::warning("Left Regular Camera's depth stream detects a chronological error!").c_str());
    prev_camera_left_depth_ts = image_message->header.stamp.toSec();
  }
}

void CameraRightDepthHandler(const Image::ConstPtr & image_message) {
  if (!receive_camera_right_depth) {
    receive_camera_right_depth = true;
    prev_camera_right_depth_ts = image_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives reprojected depth data from the Right Regular Camera!").c_str());
  } else {
    double camera_right_depth_ts_diff = image_message->header.stamp.toSec() - prev_camera_right_depth_ts;
    if (camera_right_depth_ts_diff < (1 - TS_DIFF_THLD) * KINECT_PERIOD || camera_right_depth_ts_diff > (3 + TS_DIFF_THLD) * KINECT_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Right Regular Camera's depth stream detects an unusual frequency!").c_str());
    if (camera_right_depth_ts_diff < 0)
      ROS_WARN("%s", colorful_char::warning("Right Regular Camera's depth stream detects a chronological error!").c_str());
    prev_camera_right_depth_ts = image_message->header.stamp.toSec();
  }
}

void CameraLeftUndistotrtHandler(const Image::ConstPtr & image_message) {
  if (!receive_camera_left_undistort) {
    receive_camera_left_undistort = true;
    prev_camera_left_undistort_ts = image_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives undistorted data from the Left Regular Camera!").c_str());
  } else {
    double camera_left_undistort_ts_diff = image_message->header.stamp.toSec() - prev_camera_left_undistort_ts;
    if (camera_left_undistort_ts_diff < (1 - TS_DIFF_THLD) * CAM_PERIOD || camera_left_undistort_ts_diff > (1 + TS_DIFF_THLD) * CAM_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Right Image stream detects an unusual frequency!").c_str());
    if (camera_left_undistort_ts_diff < 0)
      ROS_WARN("%s", colorful_char::warning("Left Regular Camera's depth stream detects a chronological error!").c_str());
    prev_camera_left_undistort_ts = image_message->header.stamp.toSec();
  }
}

void CameraRightUndistotrtHandler(const Image::ConstPtr & image_message) {
  if (!receive_camera_right_undistort) {
    receive_camera_right_undistort = true;
    prev_camera_right_undistort_ts = image_message->header.stamp.toSec();
    ROS_INFO("%s", colorful_char::info("Receives undistorted data from the Right Regular Camera!").c_str());
  } else {
    double camera_right_undistort_ts_diff = image_message->header.stamp.toSec() - prev_camera_right_undistort_ts;
    if (camera_right_undistort_ts_diff < (1 - TS_DIFF_THLD) * CAM_PERIOD
        || camera_right_undistort_ts_diff > (1 + TS_DIFF_THLD) * CAM_PERIOD)
      ROS_WARN("%s", colorful_char::warning("Right Image stream detects an unusual frequency!").c_str());
    if (camera_right_undistort_ts_diff < 0)
      ROS_WARN("%s", colorful_char::warning("Right Regular Camera's depth stream detects a chronological error!").c_str());
    prev_camera_right_undistort_ts = image_message->header.stamp.toSec();
  }
}

// ------------------------------------------------------------------------- //

int main(int argc, char ** argv) {
  ros::init(argc, argv, "data_validator");
  ros::NodeHandle nh;

  std::string imu_topic, event_left_topic, event_right_topic, camera_left_topic, camera_right_topic, kinect_color_topic, kinect_depth_topic,
    lidar_topic, gt_topic, event_left_depth_topic, event_right_depth_topic, camera_left_depth_topic, camera_right_depth_topic,
    camera_left_undistort_topic, camera_right_undistort_topic;
  ros::param::get("/imu_topic", imu_topic);
  ros::param::get("/event_left_topic", event_left_topic);
  ros::param::get("/event_right_topic", event_right_topic);
  ros::param::get("/camera_left_topic", camera_left_topic);
  ros::param::get("/camera_right_topic", camera_right_topic);
  ros::param::get("/kinect_color_topic", kinect_color_topic);
  ros::param::get("/kinect_depth_topic", kinect_depth_topic);
  ros::param::get("/lidar_topic", lidar_topic);
  ros::param::get("/ground_truth_topic", gt_topic);
  ros::param::get("/event_left_depth_topic", event_left_depth_topic);
  ros::param::get("/event_right_depth_topic", event_right_depth_topic);
  ros::param::get("/camera_left_depth_topic", camera_left_depth_topic);
  ros::param::get("/camera_right_depth_topic", camera_right_depth_topic);
  ros::param::get("/camera_left_undistort_topic", camera_left_undistort_topic);
  ros::param::get("/camera_right_undistort_topic", camera_right_undistort_topic);

  ros::Subscriber sub_imu                    = nh.subscribe<IMU>(imu_topic, 1000, &ImuHandler);
  ros::Subscriber sub_event_left             = nh.subscribe<EventArray>(event_left_topic, 100000, &EventLeftHandler);
  ros::Subscriber sub_event_right            = nh.subscribe<EventArray>(event_right_topic, 100000, &EventRightHandler);
  ros::Subscriber sub_camera_left            = nh.subscribe<Image>(camera_left_topic, 1000, &CameraLeftHandler);
  ros::Subscriber sub_camera_right           = nh.subscribe<Image>(camera_right_topic, 1000, &CameraRightHandler);
  ros::Subscriber sub_kinect_color           = nh.subscribe<Image>(kinect_color_topic, 1000, &KinectColorHandler);
  ros::Subscriber sub_kinect_depth           = nh.subscribe<Image>(kinect_depth_topic, 1000, &KinectDepthHandler);
  ros::Subscriber sub_lidar                  = nh.subscribe<PointCloud>(lidar_topic, 1000, &LidarHandler);
  ros::Subscriber sub_gt                     = nh.subscribe<PoseStamped>(gt_topic, 1000, &GroundTruthHandler);
  ros::Subscriber sub_event_left_depth       = nh.subscribe<Image>(event_left_depth_topic, 1000, &EventLeftDepthHandler);
  ros::Subscriber sub_event_right_depth      = nh.subscribe<Image>(event_right_depth_topic, 1000, &EventRightDepthHandler);
  ros::Subscriber sub_camera_left_depth      = nh.subscribe<Image>(camera_left_depth_topic, 1000, &CameraLeftDepthHandler);
  ros::Subscriber sub_camera_right_depth     = nh.subscribe<Image>(camera_right_depth_topic, 1000, &CameraRightDepthHandler);
  ros::Subscriber sub_camera_left_undistort  = nh.subscribe<Image>(camera_left_undistort_topic, 1000, &CameraLeftUndistotrtHandler);
  ros::Subscriber sub_camera_right_undistort = nh.subscribe<Image>(camera_right_undistort_topic, 1000, &CameraRightUndistotrtHandler);
  ros::spin();

  // By cutting off the first and last X seconds, calculate the Mean Event Rate for each event streaem.
  if (!ros::ok() && receive_event_left) {
    double event_left_start_ts   = event_left_ts.front() + EVENT_SKIP_TS;
    double event_left_end_ts     = event_left_ts.back() - EVENT_SKIP_TS;
    double event_left_duration   = event_left_end_ts - event_left_start_ts;
    double event_left_total_size = 0;
    for (size_t idx = 0; idx < event_left_ts.size(); ++idx)
      if (event_left_ts[idx] >= event_left_start_ts && event_left_ts[idx] <= event_left_end_ts)
        event_left_total_size += event_left_size[idx];
    std::cout << colorful_char::info("The Mean Event Rate for the Left Event stream is "
                                     + std::to_string(event_left_total_size / event_left_duration / 1000000)
                                     + " million events per second.")
              << std::endl;
  }
  if (!ros::ok() && receive_event_right) {
    double event_right_start_ts   = event_right_ts.front() + EVENT_SKIP_TS;
    double event_right_end_ts     = event_right_ts.back() - EVENT_SKIP_TS;
    double event_right_duration   = event_right_end_ts - event_right_start_ts;
    double event_right_total_size = 0;
    for (size_t idx = 0; idx < event_right_ts.size(); ++idx) {
      if (event_right_ts[idx] >= event_right_start_ts && event_right_ts[idx] <= event_right_end_ts)
        event_right_total_size += event_right_size[idx];
    }
    std::cout << colorful_char::info("The Mean Event Rate for the Right Event stream is "
                                     + std::to_string(event_right_total_size / event_right_duration / 1000000)
                                     + " million events per second.")
              << std::endl;
  }

  ros::shutdown();
  return 0;
}
