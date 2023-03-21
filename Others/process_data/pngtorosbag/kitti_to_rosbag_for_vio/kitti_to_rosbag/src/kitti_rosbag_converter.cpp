/*
Copyright (c) 2016, Helen Oleynikova, ETH Zurich, Switzerland
You can contact the author at <helen dot oleynikova at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <rosbag/bag.h>
#include <tf/tfMessage.h>

#include "kitti_to_rosbag/kitti_parser.h"
#include "kitti_to_rosbag/kitti_ros_conversions.h"

#include "fstream"

namespace kitti {

class KittiBagConverter {
 public:
  KittiBagConverter(const std::string& calibration_path,
                    const std::string& dataset_path,
                    const std::string& output_filename);

  ~KittiBagConverter();

  void convertAll();
  bool convertEntry(uint64_t entry);
  void convertTf(uint64_t timestamp_ns, const Transformation& imu_pose);

  void convertAllImg();
  bool convertEntryImg(uint64_t entry);

  void convertAllImu();
  bool convertEntryImu(uint64_t entry);

 private:
  kitti::KittiParser parser_;

  rosbag::Bag bag_;

  std::string world_frame_id_;
  std::string imu_frame_id_;
  std::string cam_frame_id_prefix_;
  std::string velodyne_frame_id_;

  std::string pose_topic_;
  std::string transform_topic_;
  std::string pointcloud_topic_;
  std::string imu_topic_;

  std::ofstream fPosition;
};

KittiBagConverter::KittiBagConverter(const std::string& calibration_path,
                                     const std::string& dataset_path,
                                     const std::string& output_filename)
    : parser_(calibration_path, dataset_path, true),
      world_frame_id_("world"),
      imu_frame_id_("imu"),
      cam_frame_id_prefix_("cam"),
      velodyne_frame_id_("velodyne"),
      pose_topic_("pose_imu"),
      transform_topic_("transform_imu"),
      pointcloud_topic_("velodyne_points"),
      imu_topic_("imu_data") {
  // Load all the timestamp maps and calibration parameters.
  parser_.loadCalibration();
  parser_.loadTimestampMaps();

  bag_.open(output_filename, rosbag::bagmode::Write);

  fPosition.open((dataset_path+"position_gt.txt").c_str(), std::ofstream::trunc);
}

KittiBagConverter::~KittiBagConverter() {
  fPosition.close();
}

void KittiBagConverter::convertAll() {
  uint64_t entry = 0;
  while (convertEntry(entry)) {
    entry++;
  }
  std::cout << "Converted " << entry << " entries into a rosbag.\n";
}

bool KittiBagConverter::convertEntry(uint64_t entry) {
  ros::Time timestamp_ros;
  uint64_t timestamp_ns;

  // Convert poses + TF transforms.
  Transformation pose;
  if (parser_.getPoseAtEntry(entry, &timestamp_ns, &pose)) {
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TransformStamped transform_msg;

    timestampToRos(timestamp_ns, &timestamp_ros);
    pose_msg.header.frame_id = world_frame_id_;
    pose_msg.header.stamp = timestamp_ros;
    transform_msg.header.frame_id = world_frame_id_;
    transform_msg.header.stamp = timestamp_ros;

    poseToRos(pose, &pose_msg);
    transformToRos(pose, &transform_msg);

    bag_.write(pose_topic_, timestamp_ros, pose_msg);
    bag_.write(transform_topic_, timestamp_ros, transform_msg);

    convertTf(timestamp_ns, pose);
  } else {
    return false;
  }

  // Convert images.
  cv::Mat image;
  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    if (parser_.getImageAtEntry(entry, cam_id, &timestamp_ns, &image)) {
      timestampToRos(timestamp_ns, &timestamp_ros);

      sensor_msgs::Image image_msg;
      imageToRos(image, &image_msg);
      image_msg.header.stamp = timestamp_ros;
      image_msg.header.frame_id = getCameraFrameId(cam_id);

      // TODO(helenol): cache this.
      // Get the calibration info for this camera.
      CameraCalibration cam_calib;
      parser_.getCameraCalibration(cam_id, &cam_calib);
      sensor_msgs::CameraInfo cam_info;
      calibrationToRos(cam_id, cam_calib, &cam_info);
      cam_info.header = image_msg.header;

      bag_.write(getCameraFrameId(cam_id) + "/image_raw", timestamp_ros,
                 image_msg);
      bag_.write(getCameraFrameId(cam_id) + "/camera_info", timestamp_ros,
                 cam_info);
    }
  }

  // Convert pointclouds.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  if (parser_.getPointcloudAtEntry(entry, &timestamp_ns, &pointcloud)) {
    timestampToRos(timestamp_ns, &timestamp_ros);

    // This value is in MICROSECONDS, not nanoseconds.
    pointcloud.header.stamp = timestamp_ns / 1000;
    pointcloud.header.frame_id = velodyne_frame_id_;

    bag_.write(pointcloud_topic_, timestamp_ros, pointcloud);
  }

  return true;
}

void KittiBagConverter::convertAllImg() {
  uint64_t entry = 0;
  while (convertEntryImg(entry)) {
    entry++;
  }
  std::cout << "Converted image(s) " << entry << " entries into the rosbag.\n";
}

bool KittiBagConverter::convertEntryImg(uint64_t entry) {
  ros::Time timestamp_ros;
  uint64_t timestamp_ns;
  // std::cout<< "entry = " << entry<<std::endl;

  // Convert images.
  cv::Mat image;
  //for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
  for (size_t cam_id = 0; cam_id < 2; ++cam_id) {
    if (parser_.getImageAtEntry(entry, cam_id, &timestamp_ns, &image)) { //if we have image
      timestampToRos(timestamp_ns, &timestamp_ros);//get timestamp

      sensor_msgs::Image image_msg;
      imageToRos(image, &image_msg);
      image_msg.header.stamp = timestamp_ros;
      image_msg.header.frame_id = getCameraFrameId(cam_id);

      // TODO(helenol): cache this.
      // Get the calibration info for this camera.
      CameraCalibration cam_calib;
      parser_.getCameraCalibration(cam_id, &cam_calib);
      sensor_msgs::CameraInfo cam_info;
      calibrationToRos(cam_id, cam_calib, &cam_info);
      cam_info.header = image_msg.header;

      bag_.write(getCameraFrameId(cam_id) + "/image_raw", timestamp_ros,
                 image_msg);
      bag_.write(getCameraFrameId(cam_id) + "/camera_info", timestamp_ros,
                 cam_info);
    } else {
      return false;
    }
  }
}

void KittiBagConverter::convertAllImu() {
  uint64_t entry = 0;
  while (convertEntryImu(entry)) {
    entry++;
  }
  std::cout << "Converted imu(s) " << entry << " entries into the rosbag.\n";
}

bool KittiBagConverter::convertEntryImu(uint64_t entry) {
  ros::Time timestamp_ros;
  uint64_t timestamp_ns;

  // Convert imu.
  double a_flu[3];
  double w_flu[3];
  if (parser_.getImuAtEntry(entry, &timestamp_ns, a_flu, w_flu)) {
    sensor_msgs::Imu imu_msg;

    timestampToRos(timestamp_ns, &timestamp_ros);
    imu_msg.header.frame_id = imu_frame_id_;
    imu_msg.header.stamp = timestamp_ros;

    imuToRos(a_flu, w_flu, &imu_msg);
    bag_.write(imu_topic_, timestamp_ros, imu_msg);

    Transformation pose;
    parser_.getPoseAtEntry(entry, &timestamp_ns, &pose);
    auto position = pose.getPosition();
    fPosition << timestamp_ns << " " << position(0) 
	<< " " << position(1) << " " << position(2) << std::endl;
  } else {
    return false;
  }
}

void KittiBagConverter::convertTf(uint64_t timestamp_ns,
                                  const Transformation& imu_pose) {
  tf::tfMessage tf_msg;
  ros::Time timestamp_ros;
  timestampToRos(timestamp_ns, &timestamp_ros);

  // Create the full transform chain.
  Transformation T_imu_world = imu_pose;
  Transformation T_vel_imu = parser_.T_vel_imu();
  Transformation T_cam_imu;

  geometry_msgs::TransformStamped tf_imu_world, tf_vel_imu, tf_cam_imu;
  transformToRos(T_imu_world, &tf_imu_world);
  tf_imu_world.header.frame_id = world_frame_id_;
  tf_imu_world.child_frame_id = imu_frame_id_;
  tf_imu_world.header.stamp = timestamp_ros;
  transformToRos(T_vel_imu.inverse(), &tf_vel_imu);
  tf_vel_imu.header.frame_id = imu_frame_id_;
  tf_vel_imu.child_frame_id = velodyne_frame_id_;
  tf_vel_imu.header.stamp = timestamp_ros;

  // Put them into one tf_msg.
  tf_msg.transforms.push_back(tf_imu_world);
  tf_msg.transforms.push_back(tf_vel_imu);

  // Get all of the camera transformations as well.
  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    T_cam_imu = parser_.T_camN_imu(cam_id);
    transformToRos(T_cam_imu.inverse(), &tf_cam_imu);
    tf_cam_imu.header.frame_id = imu_frame_id_;
    tf_cam_imu.child_frame_id = getCameraFrameId(cam_id);
    tf_cam_imu.header.stamp = timestamp_ros;
    tf_msg.transforms.push_back(tf_cam_imu);
  }

  bag_.write("/tf", timestamp_ros, tf_msg);
}

}  // namespace kitti

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  if (argc < 4) {
    std::cout << "Usage: rosrun kitti_to_rosbag kitti_rosbag_converter "
                 "calibration_path dataset_path output_path\n";
    std::cout << "Note: no trailing slashes.\n";
    return 0;
  }

  const std::string calibration_path = argv[1];
  const std::string dataset_path = argv[2];
  const std::string output_path = argv[3];

  kitti::KittiBagConverter converter(calibration_path, dataset_path,
                                     output_path);
  //converter.convertAll();
  converter.convertAllImg();
  // converter.convertAllImu();

  return 0;
}
