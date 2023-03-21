#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <utility.hpp>

#define EVENT_DENSE_THLD 0.00100  // [s]
#define EVENT_JUMP_THLD  0.00001  // [s]
#define EVENT_JUMP_TS    0.00500  // [s]

// ------------------------------------------------------------------------- //

// Sensor: Before Trigger --> SYNC --> After Trigger
// IMU                 NA    200Hz     NA
// Camera             5Hz     30Hz     5Hz
// KINECT              NA     30Hz     30Hz
// LIDAR         Fake 1Hz     10Hz     Fake 1Hz
// Event               NA        ?     Operates in another 20ms

int main(int argc, char ** argv) {
  ros::init(argc, argv, "timeline_reconstructor");
  ros::NodeHandle nh;

  std::string bag_in_path, gt_in_path, in_imu_topic, in_event_left_topic, in_event_right_topic, in_camera_left_topic, in_camera_right_topic,
    in_kinect_color_topic, in_kinect_depth_topic, in_lidar_topic, out_imu_topic, out_event_left_topic, out_event_right_topic,
    out_camera_left_topic, out_camera_right_topic, out_kinect_color_topic, out_kinect_depth_topic, out_lidar_topic, out_gt_topic;
  ros::param::get("/timeline_reconstructor/bag_path", bag_in_path);
  ros::param::get("/timeline_reconstructor/gt_path", gt_in_path);
  ros::param::get("/timeline_reconstructor/in_imu_topic", in_imu_topic);
  ros::param::get("/timeline_reconstructor/in_event_left_topic", in_event_left_topic);
  ros::param::get("/timeline_reconstructor/in_event_right_topic", in_event_right_topic);
  ros::param::get("/timeline_reconstructor/in_camera_left_topic", in_camera_left_topic);
  ros::param::get("/timeline_reconstructor/in_camera_right_topic", in_camera_right_topic);
  ros::param::get("/timeline_reconstructor/in_kinect_color_topic", in_kinect_color_topic);
  ros::param::get("/timeline_reconstructor/in_kinect_depth_topic", in_kinect_depth_topic);
  ros::param::get("/timeline_reconstructor/in_lidar_topic", in_lidar_topic);
  ros::param::get("/timeline_reconstructor/imu_topic", out_imu_topic);
  ros::param::get("/timeline_reconstructor/event_left_topic", out_event_left_topic);
  ros::param::get("/timeline_reconstructor/event_right_topic", out_event_right_topic);
  ros::param::get("/timeline_reconstructor/camera_left_topic", out_camera_left_topic);
  ros::param::get("/timeline_reconstructor/camera_right_topic", out_camera_right_topic);
  ros::param::get("/timeline_reconstructor/kinect_color_topic", out_kinect_color_topic);
  ros::param::get("/timeline_reconstructor/kinect_depth_topic", out_kinect_depth_topic);
  ros::param::get("/timeline_reconstructor/lidar_topic", out_lidar_topic);
  ros::param::get("/timeline_reconstructor/ground_truth_topic", out_gt_topic);
  std::string bag_out_path = bag_in_path.substr(0, bag_in_path.size() - 4) + ".synced.bag";

  bool                         receive_gt = false;
  std::deque<PoseStamped::Ptr> gt_buffer;
  if (gt_in_path != "None") {
    std::ifstream gt_file;
    gt_file.open(gt_in_path);
    if (!gt_file.is_open()) {
      ROS_ERROR("%s", colorful_char::error("Found no Ground Truth file under the path: " + gt_in_path).c_str());
      ros::shutdown();
      return -1;
    }
    receive_gt         = true;
    u_int32_t   gt_seq = 0;
    std::string input_line;
    // Ground Truth in OptiTrack format.
    for (size_t idx = 0; idx < 7; ++idx) std::getline(gt_file, input_line);
    while (std::getline(gt_file, input_line)) {
      std::vector<std::string> input_data(9, "");
      std::istringstream       read_data(input_line);
      for (size_t idx = 0; idx < 9; ++idx) std::getline(read_data, input_data[idx], ',');
      gt_buffer.emplace_back(boost::make_shared<PoseStamped>());
      gt_buffer.back()->header.stamp       = ros::Time().fromSec(gt_seq * GT_PERIOD);  // Make sure GT_PERIOD = 0.100000000
      gt_buffer.back()->header.seq         = gt_seq++;
      gt_buffer.back()->pose.orientation.x = atof(input_data[2].c_str());
      gt_buffer.back()->pose.orientation.y = atof(input_data[3].c_str());
      gt_buffer.back()->pose.orientation.z = atof(input_data[4].c_str());
      gt_buffer.back()->pose.orientation.w = atof(input_data[5].c_str());
      gt_buffer.back()->pose.position.x    = atof(input_data[6].c_str());
      gt_buffer.back()->pose.position.y    = atof(input_data[7].c_str());
      gt_buffer.back()->pose.position.z    = atof(input_data[8].c_str());
    }
    // Ground Truth in TUM format.
    // while (std::getline(gt_file, input_line)) {
    //   std::vector<std::string> input_data(8, "");
    //   std::istringstream       read_data(input_line);
    //   for (size_t idx = 0; idx < 8; ++idx) std::getline(read_data, input_data[idx], ' ');
    //   gt_buffer.emplace_back(boost::make_shared<PoseStamped>());
    //   gt_buffer.back()->header.stamp = ros::Time().fromSec(gt_seq * GT_PERIOD);  // Make sure GT_PERIOD = 0.008333333
    //   gt_buffer.back()->header.seq         = gt_seq++;
    //   gt_buffer.back()->pose.orientation.x = atof(input_data[4].c_str());
    //   gt_buffer.back()->pose.orientation.y = atof(input_data[5].c_str());
    //   gt_buffer.back()->pose.orientation.z = atof(input_data[6].c_str());
    //   gt_buffer.back()->pose.orientation.w = atof(input_data[7].c_str());
    //   gt_buffer.back()->pose.position.x    = atof(input_data[1].c_str());
    //   gt_buffer.back()->pose.position.y    = atof(input_data[2].c_str());
    //   gt_buffer.back()->pose.position.z    = atof(input_data[3].c_str());
    // }
    ROS_INFO("%s", colorful_char::info("Successfully load the Ground Truth file!").c_str());
  }

  rosbag::Bag bag_in, bag_out;
  bag_in.open(bag_in_path, rosbag::bagmode::Read);
  bag_out.open(bag_out_path, rosbag::bagmode::Write);
  rosbag::View view(bag_in);
  uint32_t     num_msg = view.size();

  bool receive_imu          = false;
  bool receive_event_left   = false;
  bool receive_event_right  = false;
  bool receive_camera_left  = false;
  bool receive_camera_right = false;
  bool receive_kinect_color = false;
  bool receive_kinect_depth = false;
  bool receive_lidar        = false;

  std::deque<ros::Time>                  imu_ts_buffer;
  std::deque<bool>                       event_left_ts_counter;
  std::deque<bool>                       event_right_ts_counter;
  std::deque<std::pair<ros::Time, bool>> camera_left_ts_buffer;
  std::deque<std::pair<ros::Time, bool>> camera_right_ts_buffer;
  std::vector<ros::Time>                 kinect_color_ts_buffer;
  std::vector<ros::Time>                 kinect_depth_ts_buffer;
  std::deque<std::pair<ros::Time, bool>> lidar_ts_buffer;
  kinect_color_ts_buffer.reserve(num_msg);
  kinect_depth_ts_buffer.reserve(num_msg);
  for (const rosbag::MessageInstance & msg : view) {
    if (msg.getTopic() == in_imu_topic) {
      receive_imu = true;
      imu_ts_buffer.emplace_back(msg.instantiate<IMU>()->header.stamp);
    } else if (msg.getTopic() == in_event_left_topic) {
      receive_event_left = true;
      event_left_ts_counter.emplace_back(true);
    } else if (msg.getTopic() == in_event_right_topic) {
      receive_event_right = true;
      event_right_ts_counter.emplace_back(true);
    } else if (msg.getTopic() == in_camera_left_topic) {
      receive_camera_left = true;
      camera_left_ts_buffer.emplace_back(msg.instantiate<Image>()->header.stamp, false);
      if (camera_left_ts_buffer.size() != 1) {
        size_t last_idx = camera_left_ts_buffer.size() - 1;
        if (camera_left_ts_buffer[last_idx].first.toSec() - camera_left_ts_buffer[last_idx - 1].first.toSec() < CAM_PERIOD * 1.5) {
          camera_left_ts_buffer[last_idx].second     = true;
          camera_left_ts_buffer[last_idx - 1].second = true;
        }
      }
    } else if (msg.getTopic() == in_camera_right_topic) {
      receive_camera_right = true;
      camera_right_ts_buffer.emplace_back(msg.instantiate<Image>()->header.stamp, false);
      if (camera_right_ts_buffer.size() != 1) {
        size_t last_idx = camera_right_ts_buffer.size() - 1;
        if (camera_right_ts_buffer[last_idx].first.toSec() - camera_right_ts_buffer[last_idx - 1].first.toSec() < CAM_PERIOD * 1.5) {
          camera_right_ts_buffer[last_idx].second     = true;
          camera_right_ts_buffer[last_idx - 1].second = true;
        }
      }
    } else if (msg.getTopic() == in_kinect_color_topic) {
      receive_kinect_color = true;
      kinect_color_ts_buffer.emplace_back(msg.instantiate<Image>()->header.stamp);
    } else if (msg.getTopic() == in_kinect_depth_topic) {
      receive_kinect_depth = true;
      kinect_depth_ts_buffer.emplace_back(msg.instantiate<Image>()->header.stamp);
    } else if (msg.getTopic() == in_lidar_topic) {
      receive_lidar = true;
      lidar_ts_buffer.emplace_back(msg.instantiate<PointCloud>()->header.stamp, false);
      if (lidar_ts_buffer.size() != 1) {
        size_t last_idx = lidar_ts_buffer.size() - 1;
        if (lidar_ts_buffer[last_idx].first.toSec() - lidar_ts_buffer[last_idx - 1].first.toSec() < LIDAR_PERIOD * 1.5) {
          lidar_ts_buffer[last_idx].second     = true;
          lidar_ts_buffer[last_idx - 1].second = true;
        }
      }
    }
  }
  ROS_INFO("%s", colorful_char::info("Successfully load the rosbag!").c_str());

  double start_ts                 = imu_ts_buffer.front().toSec();
  size_t imu_msg_idx              = 0;
  size_t camera_left_msg_idx      = 0;
  size_t camera_right_msg_idx     = 0;
  size_t kinect_color_msg_idx     = 0;
  size_t kinect_color_lost_frames = 1;
  double kinect_color_msg_prev_ts = 0;
  size_t kinect_depth_msg_idx     = 0;
  size_t kinect_depth_lost_frames = 1;
  double kinect_depth_msg_prev_ts = 0;
  double lidar_first_msg_ts       = 0;
  for (ros::Time & time : imu_ts_buffer) time.fromSec(start_ts + (imu_msg_idx++) * IMU_PERIOD);
  for (auto & pair : camera_left_ts_buffer)
    if (pair.second) pair.first.fromSec(start_ts + (camera_left_msg_idx++) * CAM_PERIOD);
  for (auto & pair : camera_right_ts_buffer)
    if (pair.second) pair.first.fromSec(start_ts + (camera_right_msg_idx++) * CAM_PERIOD);
  for (size_t idx = 0; idx < kinect_color_ts_buffer.size(); ++idx) {
    if (kinect_color_msg_idx != 0) {
      double ts_diff = kinect_color_ts_buffer[idx].toSec() - kinect_color_msg_prev_ts;
      if (ts_diff > 1.75 * KINECT_PERIOD) {
        if (ts_diff > 3.75 * KINECT_PERIOD) {
          kinect_color_lost_frames += 3;
          ROS_ERROR("%s", colorful_char::error("Kinect Color stream lost three consecutive messages! Unreliable data sequence!").c_str());
          ROS_ERROR("%s", colorful_char::error("Timeline reconstruction shut down! Please re-record another bag instead!").c_str());
          ros::shutdown();
          return -1;
        } else if (ts_diff > 2.75 * KINECT_PERIOD) {
          kinect_color_lost_frames += 2;
          ROS_WARN("%s", colorful_char::warning("Kinect Color stream lost two consecutive messages! Not recommend!").c_str());
        } else if (idx == kinect_color_ts_buffer.size() - 1
                   || kinect_color_ts_buffer[idx + 1].toSec() - kinect_color_ts_buffer[idx].toSec() > 0.5 * KINECT_PERIOD) {
          kinect_color_lost_frames += 1;
          ROS_WARN("%s", colorful_char::warning("Kinect Color stream lost one message!").c_str());
        }
      }
    }
    kinect_color_msg_prev_ts = kinect_color_ts_buffer[idx].toSec();
    kinect_color_ts_buffer[idx].fromSec(start_ts + (kinect_color_lost_frames + kinect_color_msg_idx++) * KINECT_PERIOD);
  }
  for (size_t idx = 0; idx < kinect_depth_ts_buffer.size(); ++idx) {
    if (kinect_depth_msg_idx != 0) {
      double ts_diff = kinect_depth_ts_buffer[idx].toSec() - kinect_depth_msg_prev_ts;
      if (ts_diff > 1.75 * KINECT_PERIOD) {
        if (ts_diff > 3.75 * KINECT_PERIOD) {
          kinect_depth_lost_frames += 3;
          ROS_ERROR("%s", colorful_char::error("Kinect Depth stream lost three consecutive messages! Unreliable data sequence!").c_str());
          ROS_ERROR("%s", colorful_char::error("Timeline reconstruction shut down! Please re-record another bag instead!").c_str());
          ros::shutdown();
          return -1;
        } else if (ts_diff > 2.75 * KINECT_PERIOD) {
          kinect_depth_lost_frames += 2;
          ROS_WARN("%s", colorful_char::warning("Kinect Depth stream lost two consecutive messages! Not recommend!").c_str());
        } else if (idx == kinect_depth_ts_buffer.size() - 1
                   || kinect_depth_ts_buffer[idx + 1].toSec() - kinect_depth_ts_buffer[idx].toSec() > 0.5 * KINECT_PERIOD) {
          kinect_depth_lost_frames += 1;
          ROS_WARN("%s", colorful_char::warning("Kinect Depth stream lost one message!").c_str());
        }
      }
    }
    kinect_depth_msg_prev_ts = kinect_depth_ts_buffer[idx].toSec();
    kinect_depth_ts_buffer[idx].fromSec(start_ts + (kinect_depth_lost_frames + kinect_depth_msg_idx++) * KINECT_PERIOD);
  }
  for (auto & pair : lidar_ts_buffer) {
    if (pair.second) {
      if (lidar_first_msg_ts == 0) lidar_first_msg_ts = pair.first.toSec();
      pair.first.fromSec(start_ts + pair.first.toSec() - lidar_first_msg_ts);
    }
  }
  for (auto & msg : gt_buffer) msg->header.stamp.fromSec(start_ts + msg->header.stamp.toSec());
  ROS_INFO("%s", colorful_char::info("Successfully reconstruct the ros message timeline!").c_str());

  std::deque<IMU::Ptr>        imu_buffer;
  std::deque<EventArray::Ptr> event_left_buffer;
  std::deque<EventArray::Ptr> event_right_buffer;
  std::deque<Image::Ptr>      camera_left_buffer;
  std::deque<Image::Ptr>      camera_right_buffer;
  std::deque<Image::Ptr>      kinect_color_buffer;
  std::deque<Image::Ptr>      kinect_depth_buffer;
  std::deque<PointCloud::Ptr> lidar_buffer;

  bool   is_event_left_dense         = false;
  double prev_event_left_ts          = 0;
  double prev_event_left_trigger_ts  = 0;
  double event_left_ts_offset        = 0;
  bool   is_event_right_dense        = false;
  double prev_event_right_ts         = 0;
  double prev_event_right_trigger_ts = 0;
  double event_right_ts_offset       = 0;
  size_t kinect_color_ts_idx         = 0;
  size_t kinect_depth_ts_idx         = 0;
  for (const rosbag::MessageInstance & msg : view) {
    if (msg.getTopic() == in_imu_topic) imu_buffer.emplace_back(msg.instantiate<IMU>());
    else if (msg.getTopic() == in_event_left_topic) {
      event_left_buffer.emplace_back(msg.instantiate<EventArray>());
      for (auto & event : event_left_buffer.back()->events) {
        if (is_event_left_dense) {
          if (event.ts.toSec() >= prev_event_left_trigger_ts + (EVENT_JUMP_TS - EVENT_JUMP_THLD)
              && event.ts.toSec() <= prev_event_left_trigger_ts + (EVENT_JUMP_TS + EVENT_JUMP_THLD)) {
            event_left_ts_offset -= EVENT_JUMP_TS;
            ROS_WARN("%s", colorful_char::warning("Left Event stream receives two consecutive triggers!").c_str());
            ROS_WARN("%s", colorful_char::warning("Left Event stream delay = " + std::to_string(event_left_ts_offset) + "s").c_str());
          }
          if (event.ts.toSec() <= prev_event_left_trigger_ts - (EVENT_JUMP_TS - EVENT_JUMP_THLD)
              && event.ts.toSec() >= prev_event_left_trigger_ts - (EVENT_JUMP_TS + EVENT_JUMP_THLD)) {
            event_left_ts_offset += EVENT_JUMP_TS;
            ROS_WARN("%s", colorful_char::warning("Left Event stream receives no trigger within a short time window!").c_str());
            ROS_WARN("%s", colorful_char::warning("Left Event stream delay = " + std::to_string(event_left_ts_offset) + "s").c_str());
          }
        }
        prev_event_left_trigger_ts = event.ts.toSec();
        event.ts.fromSec(event.ts.toSec() + event_left_ts_offset);
        if (event.ts.toSec() < prev_event_left_ts) event.ts.fromSec(prev_event_left_ts);
        is_event_left_dense = (event.ts.toSec() < prev_event_left_ts + EVENT_DENSE_THLD) ? true : false;
        prev_event_left_ts  = event.ts.toSec();
        event.ts.fromSec(start_ts + event.ts.toSec());
      }
      event_left_buffer.back()->header.stamp.fromSec(event_left_buffer.back()->events.front().ts.toSec());
    } else if (msg.getTopic() == in_event_right_topic) {
      event_right_buffer.emplace_back(msg.instantiate<EventArray>());
      for (auto & event : event_right_buffer.back()->events) {
        if (is_event_right_dense) {
          if (event.ts.toSec() >= prev_event_right_trigger_ts + (EVENT_JUMP_TS - EVENT_JUMP_THLD)
              && event.ts.toSec() <= prev_event_right_trigger_ts + (EVENT_JUMP_TS + EVENT_JUMP_THLD)) {
            event_right_ts_offset -= EVENT_JUMP_TS;
            ROS_WARN("%s", colorful_char::warning("Right Event stream receives two consecutive triggers!").c_str());
            ROS_WARN("%s", colorful_char::warning("Right Event stream delay = " + std::to_string(event_right_ts_offset) + "s").c_str());
          }
          if (event.ts.toSec() <= prev_event_right_trigger_ts - (EVENT_JUMP_TS - EVENT_JUMP_THLD)
              && event.ts.toSec() >= prev_event_right_trigger_ts - (EVENT_JUMP_TS + EVENT_JUMP_THLD)) {
            event_right_ts_offset += EVENT_JUMP_TS;
            ROS_WARN("%s", colorful_char::warning("Right Event stream receives no trigger within a short time window!").c_str());
            ROS_WARN("%s", colorful_char::warning("Right Event stream delay = " + std::to_string(event_right_ts_offset) + "s").c_str());
          }
        }
        prev_event_right_trigger_ts = event.ts.toSec();
        event.ts.fromSec(event.ts.toSec() + event_right_ts_offset);
        if (event.ts.toSec() < prev_event_right_ts) event.ts.fromSec(prev_event_right_ts);
        is_event_right_dense = (event.ts.toSec() < prev_event_right_ts + EVENT_DENSE_THLD) ? true : false;
        prev_event_right_ts  = event.ts.toSec();
        event.ts.fromSec(start_ts + event.ts.toSec());
      }
      event_right_buffer.back()->header.stamp.fromSec(event_right_buffer.back()->events.front().ts.toSec());
    } else if (msg.getTopic() == in_camera_left_topic) {
      if (!camera_left_ts_buffer.front().second) camera_left_ts_buffer.pop_front();
      else
        camera_left_buffer.emplace_back(msg.instantiate<Image>());
    } else if (msg.getTopic() == in_camera_right_topic) {
      if (!camera_right_ts_buffer.front().second) camera_right_ts_buffer.pop_front();
      else
        camera_right_buffer.emplace_back(msg.instantiate<Image>());
    } else if (msg.getTopic() == in_kinect_color_topic)
      kinect_color_buffer.emplace_back(msg.instantiate<Image>());
    else if (msg.getTopic() == in_kinect_depth_topic)
      kinect_depth_buffer.emplace_back(msg.instantiate<Image>());
    else if (msg.getTopic() == in_lidar_topic) {
      if (!lidar_ts_buffer.front().second) lidar_ts_buffer.pop_front();
      else
        lidar_buffer.emplace_back(msg.instantiate<PointCloud>());
    }

    while (receive_imu) {
      int    earliest_topic = 0;
      double earliest_ts    = DBL_MAX;
      if (receive_imu) {
        if (imu_buffer.size() == 0) break;
        else if (earliest_ts > imu_ts_buffer.front().toSec()) {
          earliest_topic = 1;
          earliest_ts    = imu_ts_buffer.front().toSec();
        }
      }
      if (receive_event_left) {
        if (event_left_buffer.size() == 0) break;
        else if (earliest_ts > event_left_buffer.front()->header.stamp.toSec()) {
          earliest_topic = 2;
          earliest_ts    = event_left_buffer.front()->header.stamp.toSec();
        }
      }
      if (receive_event_right) {
        if (event_right_buffer.size() == 0) break;
        else if (earliest_ts > event_right_buffer.front()->header.stamp.toSec()) {
          earliest_topic = 3;
          earliest_ts    = event_right_buffer.front()->header.stamp.toSec();
        }
      }
      if (receive_camera_left) {
        if (camera_left_buffer.size() == 0) break;
        else if (earliest_ts > camera_left_ts_buffer.front().first.toSec()) {
          earliest_topic = 4;
          earliest_ts    = camera_left_ts_buffer.front().first.toSec();
        }
      }
      if (receive_camera_right) {
        if (camera_right_buffer.size() == 0) break;
        else if (earliest_ts > camera_right_ts_buffer.front().first.toSec()) {
          earliest_topic = 5;
          earliest_ts    = camera_right_ts_buffer.front().first.toSec();
        }
      }
      if (receive_kinect_color) {
        if (kinect_color_buffer.size() == 0) break;
        else if (earliest_ts > kinect_color_ts_buffer[kinect_color_ts_idx].toSec()) {
          earliest_topic = 6;
          earliest_ts    = kinect_color_ts_buffer[kinect_color_ts_idx].toSec();
        }
      }
      if (receive_kinect_depth) {
        if (kinect_depth_buffer.size() == 0) break;
        else if (earliest_ts > kinect_depth_ts_buffer[kinect_depth_ts_idx].toSec()) {
          earliest_topic = 7;
          earliest_ts    = kinect_depth_ts_buffer[kinect_depth_ts_idx].toSec();
        }
      }
      if (receive_lidar) {
        if (lidar_buffer.size() == 0) break;
        else if (earliest_ts > lidar_ts_buffer.front().first.toSec()) {
          earliest_topic = 8;
          earliest_ts    = lidar_ts_buffer.front().first.toSec();
        }
      }
      if (receive_gt) {
        if (gt_buffer.size() == 0) break;
        else if (earliest_ts > gt_buffer.front()->header.stamp.toSec()) {
          earliest_topic = 9;
          earliest_ts    = gt_buffer.front()->header.stamp.toSec();
        }
      }

      switch (earliest_topic) {
        case 1 :
          imu_buffer.front()->header.stamp.fromSec(imu_ts_buffer.front().toSec());
          bag_out.write(out_imu_topic, imu_ts_buffer.front(), imu_buffer.front());
          imu_buffer.pop_front();
          imu_ts_buffer.pop_front();
          if (imu_ts_buffer.size() == 0) receive_imu = false;
          break;
        case 2 :
          bag_out.write(out_event_left_topic, event_left_buffer.front()->header.stamp, event_left_buffer.front());
          event_left_buffer.pop_front();
          event_left_ts_counter.pop_front();
          if (event_left_ts_counter.size() == 0) receive_event_left = false;
          break;
        case 3 :
          bag_out.write(out_event_right_topic, event_right_buffer.front()->header.stamp, event_right_buffer.front());
          event_right_buffer.pop_front();
          event_right_ts_counter.pop_front();
          if (event_right_ts_counter.size() == 0) receive_event_right = false;
          break;
        case 4 :
          camera_left_buffer.front()->header.stamp.fromSec(camera_left_ts_buffer.front().first.toSec());
          bag_out.write(out_camera_left_topic, camera_left_ts_buffer.front().first, camera_left_buffer.front());
          camera_left_buffer.pop_front();
          camera_left_ts_buffer.pop_front();
          if (camera_left_ts_buffer.size() == 0 || !camera_left_ts_buffer.front().second) receive_camera_left = false;
          break;
        case 5 :
          camera_right_buffer.front()->header.stamp.fromSec(camera_right_ts_buffer.front().first.toSec());
          bag_out.write(out_camera_right_topic, camera_right_ts_buffer.front().first, camera_right_buffer.front());
          camera_right_buffer.pop_front();
          camera_right_ts_buffer.pop_front();
          if (camera_right_ts_buffer.size() == 0 || !camera_right_ts_buffer.front().second) receive_camera_right = false;
          break;
        case 6 :
          kinect_color_buffer.front()->header.stamp.fromSec(kinect_color_ts_buffer[kinect_color_ts_idx].toSec());
          bag_out.write(out_kinect_color_topic, kinect_color_ts_buffer[kinect_color_ts_idx++], kinect_color_buffer.front());
          kinect_color_buffer.pop_front();
          if (kinect_color_ts_idx == kinect_color_ts_buffer.size()) receive_kinect_color = false;
          break;
        case 7 :
          kinect_depth_buffer.front()->header.stamp.fromSec(kinect_depth_ts_buffer[kinect_depth_ts_idx].toSec());
          bag_out.write(out_kinect_depth_topic, kinect_depth_ts_buffer[kinect_depth_ts_idx++], kinect_depth_buffer.front());
          kinect_depth_buffer.pop_front();
          if (kinect_depth_ts_idx == kinect_depth_ts_buffer.size()) receive_kinect_depth = false;
          break;
        case 8 :
          lidar_buffer.front()->header.stamp.fromSec(lidar_ts_buffer.front().first.toSec());
          bag_out.write(out_lidar_topic, lidar_ts_buffer.front().first, lidar_buffer.front());
          lidar_buffer.pop_front();
          lidar_ts_buffer.pop_front();
          if (lidar_ts_buffer.size() == 0 || !lidar_ts_buffer.front().second) receive_lidar = false;
          break;
        case 9 :
          bag_out.write(out_gt_topic, gt_buffer.front()->header.stamp, gt_buffer.front());
          gt_buffer.pop_front();
          if (gt_buffer.size() == 0) receive_gt = false;
          break;
      }
    }
  }
  bag_out.close();
  ROS_INFO("%s", colorful_char::info("Timeline reconstruction ends successfully. Enjoy the new bag!").c_str());

  ros::shutdown();
  return 0;
}
