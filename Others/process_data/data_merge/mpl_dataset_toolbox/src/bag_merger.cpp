#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <utility.hpp>

// ------------------------------------------------------------------------- //

int BagNameParser(const std::string & file_name, std::string & prefix) {
  bool last_dot     = false;
  int  last_dot_idx = -1;
  for (int idx = file_name.size() - 1; idx >= 0; --idx) {
    if (!last_dot && file_name[idx] == '.') {
      last_dot     = true;
      last_dot_idx = idx;
    } else if (last_dot && file_name[idx] == '.') {
      prefix                 = file_name.substr(0, idx);
      std::string topic_name = file_name.substr(idx + 1, last_dot_idx - idx - 1);
      if (topic_name == "imu") return 1;
      else if (topic_name == "left_event")
        return 2;
      else if (topic_name == "right_event")
        return 3;
      else if (topic_name == "left_camera")
        return 4;
      else if (topic_name == "right_camera")
        return 5;
      else if (topic_name == "kinect_color")
        return 6;
      else if (topic_name == "kinect_depth")
        return 7;
      else if (topic_name == "lidar")
        return 8;
      else if (topic_name == "gt")
        return 9;
      else if (topic_name == "depth_reprojection")
        return 10;
      else if (topic_name == "left_event_depth")
        return 11;
      else if (topic_name == "right_event_depth")
        return 12;
      else if (topic_name == "left_camera_depth")
        return 13;
      else if (topic_name == "right_camera_depth")
        return 14;
      else if (topic_name == "left_camera_undistort")
        return 15;
      else if (topic_name == "right_camera_undistort")
        return 16;
      else
        return 0;
    }
  }
  return 0;
}

void PrintDirectoryInfo(std::map<std::string, std::vector<int>> & out_bags_info) {
  ROS_INFO("%s", colorful_char::info("The following valid rosbags have been detected: ").c_str());
  for (auto bag_it = out_bags_info.begin(); bag_it != out_bags_info.end(); ++bag_it) {
    std::sort(bag_it->second.begin(), bag_it->second.end());
    std::string topic_list = "";
    for (int topic_id : bag_it->second) {
      switch (topic_id) {
        case 1 : topic_list += "IMU"; break;
        case 2 : topic_list += "Left Event Camera"; break;
        case 3 : topic_list += "Right Event Camera"; break;
        case 4 : topic_list += "Left Regular Camera"; break;
        case 5 : topic_list += "Right Regular Camera"; break;
        case 6 : topic_list += "Kinect Color Camera"; break;
        case 7 : topic_list += "Kinect Depth Camera"; break;
        case 8 : topic_list += "LiDAR"; break;
        case 9 : topic_list += "Ground Truth Signal"; break;
        case 10 : topic_list += "Full Depth Reprojection"; break;
        case 11 : topic_list += "Left Event Camera (Depth)"; break;
        case 12 : topic_list += "Right Event Camera (Depth)"; break;
        case 13 : topic_list += "Left Regular Camera (Depth)"; break;
        case 14 : topic_list += "Right Regular Camera (Depth)"; break;
        case 15 : topic_list += "Left Regular Camera (Undistorted)"; break;
        case 16 : topic_list += "Right Regular Camera (Undistorted)"; break;
      }
      if (topic_id != bag_it->second.back()) topic_list += " | ";
    }
    if (bag_it->second.size() == 1) {
      ROS_INFO_STREAM("Data Sequence Name:   \033[1;36m" << bag_it->first << "\033[0m");
      ROS_INFO_STREAM("Data Sequence Topic:  \033[1;36m" << topic_list << "\033[1;31m --> Single bag cannot be merged! \033[0m");
    } else {
      ROS_INFO_STREAM("Data Sequence Name:   \033[1;36m" << bag_it->first << "\033[0m");
      ROS_INFO_STREAM("Data Sequence Topics: \033[1;36m" << topic_list << "\033[0m");
    }
  }
}

// ------------------------------------------------------------------------- //

int main(int argc, char ** argv) {
  ros::init(argc, argv, "bag_merger");
  ros::NodeHandle nh;

  // Check the directory path and create a directory for merged bags under same path.
  std::string in_dir_path, out_dir_path;
  ros::param::get("/directory_path", in_dir_path);
  if (!directory_path_check(in_dir_path)) {
    ros::shutdown();
    return -1;
  };
  out_dir_path = in_dir_path + "merged_bags/";
  fs::create_directory(out_dir_path);

  // Read each rosbag's absolute path from the input directory and classify by their prefix.
  fs::path                                dir(in_dir_path);
  fs::directory_iterator                  bag_path_it(dir);
  fs::directory_iterator                  end_it;
  std::map<std::string, std::vector<int>> out_bags_info;
  for (; bag_path_it != end_it; ++bag_path_it) {
    if (fs::is_regular_file(bag_path_it->status()) && bag_path_it->path().extension().string() == ".bag") {
      std::string bag_name = bag_path_it->path().filename().string();
      std::string prefix;
      int         topic_id = BagNameParser(bag_name, prefix);
      if (topic_id == 0) continue;
      out_bags_info[prefix].emplace_back(topic_id);
    }
  }
  if (out_bags_info.size() == 0) {
    ROS_ERROR("%s", colorful_char::error("Found no rosbag under the directory path: " + in_dir_path).c_str());
    ros::shutdown();
    return -1;
  }
  PrintDirectoryInfo(out_bags_info);

  // Merge all bags with the same prefix.
  for (auto bag_it = out_bags_info.begin(); bag_it != out_bags_info.end(); ++bag_it) {
    if (bag_it->second.size() == 1) continue;
    rosbag::Bag bag_out;
    bag_out.open(out_dir_path + bag_it->first + ".merged.bag", rosbag::bagmode::Write);

    // Read each single-topic bag.
    size_t                              topic_num = bag_it->second.size();
    size_t                              msg_size  = 0;
    rosbag::Bag                         bags_in[topic_num];
    rosbag::View                        bags_views[topic_num];
    std::vector<rosbag::View::iterator> bags_msg_ptr;
    std::vector<bool>                   are_bags_valid;
    for (size_t idx = 0; idx < bag_it->second.size(); ++idx) {
      switch (bag_it->second[idx]) {
        case 1 : bags_in[idx].open(in_dir_path + bag_it->first + ".imu.bag", rosbag::bagmode::Read); break;
        case 2 : bags_in[idx].open(in_dir_path + bag_it->first + ".left_event.bag", rosbag::bagmode::Read); break;
        case 3 : bags_in[idx].open(in_dir_path + bag_it->first + ".right_event.bag", rosbag::bagmode::Read); break;
        case 4 : bags_in[idx].open(in_dir_path + bag_it->first + ".left_camera.bag", rosbag::bagmode::Read); break;
        case 5 : bags_in[idx].open(in_dir_path + bag_it->first + ".right_camera.bag", rosbag::bagmode::Read); break;
        case 6 : bags_in[idx].open(in_dir_path + bag_it->first + ".kinect_color.bag", rosbag::bagmode::Read); break;
        case 7 : bags_in[idx].open(in_dir_path + bag_it->first + ".kinect_depth.bag", rosbag::bagmode::Read); break;
        case 8 : bags_in[idx].open(in_dir_path + bag_it->first + ".lidar.bag", rosbag::bagmode::Read); break;
        case 9 : bags_in[idx].open(in_dir_path + bag_it->first + ".gt.bag", rosbag::bagmode::Read); break;
        case 10 : bags_in[idx].open(in_dir_path + bag_it->first + ".depth_reprojection.bag", rosbag::bagmode::Read); break;
        case 11 : bags_in[idx].open(in_dir_path + bag_it->first + ".left_event_depth.bag", rosbag::bagmode::Read); break;
        case 12 : bags_in[idx].open(in_dir_path + bag_it->first + ".right_event_depth.bag", rosbag::bagmode::Read); break;
        case 13 : bags_in[idx].open(in_dir_path + bag_it->first + ".left_camera_depth.bag", rosbag::bagmode::Read); break;
        case 14 : bags_in[idx].open(in_dir_path + bag_it->first + ".right_camera_depth.bag", rosbag::bagmode::Read); break;
        case 15 : bags_in[idx].open(in_dir_path + bag_it->first + ".left_camera_undistort.bag", rosbag::bagmode::Read); break;
        case 16 : bags_in[idx].open(in_dir_path + bag_it->first + ".right_camera_undistort.bag", rosbag::bagmode::Read); break;
      }
      bags_views[idx].addQuery(bags_in[idx]);
      msg_size += bags_views[idx].size();
      bags_msg_ptr.emplace_back(bags_views[idx].begin());
      are_bags_valid.emplace_back(true);
    }

    // Merge all messages from each single-topic bag chronologically.
    size_t msg_idx = 0;
    while (topic_num != 0) {
      int    earliest_idx;
      double earliest_ts = DBL_MAX;
      for (size_t idx = 0; idx < bag_it->second.size(); ++idx) {
        if (are_bags_valid[idx] && earliest_ts > bags_msg_ptr[idx]->getTime().toSec()) {
          earliest_idx = idx;
          earliest_ts  = bags_msg_ptr[idx]->getTime().toSec();
        }
      }
      bag_out.write(bags_msg_ptr[earliest_idx]->getTopic(), bags_msg_ptr[earliest_idx]->getTime(), *bags_msg_ptr[earliest_idx]);
      ++bags_msg_ptr[earliest_idx];
      if (bags_msg_ptr[earliest_idx] == bags_views[earliest_idx].end()) {
        are_bags_valid[earliest_idx] = false;
        --topic_num;
        bags_in[earliest_idx].close();
      }

      ++msg_idx;
      if (msg_idx % 100000 == 0)
        ROS_INFO_STREAM(msg_idx << "/" << msg_size << " messages from data sequence \"" << bag_it->first << "\" have been processed.");
    }
    ROS_INFO("%s", colorful_char::info("All \"" + bag_it->first + "\"-related bags have been merged into a new bag! Enjoy!").c_str());
    bag_out.close();
  }

  ros::shutdown();
  return 0;
}
