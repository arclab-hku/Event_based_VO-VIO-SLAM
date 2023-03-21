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

#include <iostream>
#include <fstream>
#include <iomanip>

#include <opencv2/highgui/highgui.hpp>

#include "kitti_to_rosbag/kitti_parser.h"

#include <cv_bridge/cv_bridge.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

#include "kitti_to_rosbag/kitti_ros_conversions.h"

namespace kitti {

const std::string KittiParser::kVelToCamCalibrationFilename =
    "calib_velo_to_cam.txt";
const std::string KittiParser::kCamToCamCalibrationFilename =
    "calib_cam_to_cam.txt";
const std::string KittiParser::kImuToVelCalibrationFilename =
    "calib_imu_to_velo.txt";

const std::string KittiParser::kVelodyneFolder = "velodyne_points";
const std::string KittiParser::kCameraFolder = "image_";
const std::string KittiParser::kPoseFolder = "oxts";

const std::string KittiParser::kTimestampFilename = "timestamps.txt";
const std::string KittiParser::kDataFolder = "data";

KittiParser::KittiParser(const std::string& calibration_path,
                         const std::string& dataset_path, bool rectified)
    : calibration_path_(calibration_path),
      dataset_path_(dataset_path),
      rectified_(rectified),
      initial_pose_set_(false) {}

bool KittiParser::loadCalibration() {
  loadVelToCamCalibration();
  loadImuToVelCalibration();
  loadCamToCamCalibration();
  return true;
}

bool KittiParser::loadVelToCamCalibration() {
  std::string filename = calibration_path_ + "/" + kVelToCamCalibrationFilename;
  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }

  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    // Check what the header is. Each line consists of two parts:
    // a header followed by a ':' followed by space-separated data.
    std::string header;
    std::getline(line_stream, header, ':');
    std::string data;
    std::getline(line_stream, data, ':');

    std::vector<double> parsed_doubles;
    // Compare all header possibilities...
    if (header.compare("R") == 0) {
      // Parse the rotation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Matrix3d R(parsed_doubles.data());
        // All matrices are written row-major but Eigen is column-major
        // (can swap this, but I guess it's anyway easier to just transpose
        // for these small matrices).
        T_cam0_vel_.getRotation() =
            Rotation::fromApproximateRotationMatrix(R.transpose());
      }
    } else if (header.compare("T") == 0) {
      // Parse the translation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Vector3d T(parsed_doubles.data());
        T_cam0_vel_.getPosition() = T;
      }
    }
  }
  // How do we return false?
  return true;
}

bool KittiParser::loadImuToVelCalibration() {
  std::string filename = calibration_path_ + "/" + kImuToVelCalibrationFilename;
  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }

  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    // Check what the header is. Each line consists of two parts:
    // a header followed by a ':' followed by space-separated data.
    std::string header;
    std::getline(line_stream, header, ':');
    std::string data;
    std::getline(line_stream, data, ':');

    std::vector<double> parsed_doubles;
    // Compare all header possibilities...
    if (header.compare("R") == 0) {
      // Parse the rotation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Matrix3d R(parsed_doubles.data());
        // All matrices are written row-major but Eigen is column-major
        // (can swap this, but I guess it's anyway easier to just transpose
        // for these small matrices).
        T_vel_imu_.getRotation() =
            Rotation::fromApproximateRotationMatrix(R.transpose());
      }
    } else if (header.compare("T") == 0) {
      // Parse the translation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Vector3d T(parsed_doubles.data());
        T_vel_imu_.getPosition() = T;
      }
    }
  }
  std::cout << "Transform T_vel_imu: " << T_vel_imu_.getTransformationMatrix()
            << std::endl;
  return true;
}

bool KittiParser::loadCamToCamCalibration() {
  std::string filename = calibration_path_ + "/" + kCamToCamCalibrationFilename;
  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }

  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    // Check what the header is. Each line consists of two parts:
    // a header followed by a ':' followed by space-separated data.
    std::string header;
    std::getline(line_stream, header, ':');
    std::string data;
    std::getline(line_stream, data, ':');

    std::vector<double> parsed_doubles;
    // Compare all header possibilities...

    std::cout << "Header: " << header << "\n";

    // Load image size.
    if (header.compare(0, 6, "S_rect") == 0) {
      std::cout << "S_rect header: " << header
                << " substring: " << header.substr(7) << std::endl;
      if (rectified_) {
        // Figure out which number this is.
        int index = std::stoi(header.substr(7));
        std::cout << "Index: " << index << std::endl;
        if (camera_calibrations_.size() <= index) {
          camera_calibrations_.resize(index + 1);
        }
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          camera_calibrations_[index].image_size =
              Eigen::Vector2d(parsed_doubles.data());
          std::cout << "Image size: " << camera_calibrations_[index].image_size
                    << std::endl;
        }
      }
    } else if (!rectified_ && header.compare(0, 2, "S_") == 0) {
      int index = std::stoi(header.substr(2));
      if (camera_calibrations_.size() <= index) {
        camera_calibrations_.resize(index + 1);
      }
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        camera_calibrations_[index].image_size =
            Eigen::Vector2d(parsed_doubles.data());
      }
    }

    // Load rectification matrix.
    if (header.compare(0, 6, "R_rect") == 0) {
      std::cout << "R_rect header: " << header
                << " substring: " << header.substr(7) << std::endl;
      if (rectified_) {
        // Figure out which number this is.
        int index = std::stoi(header.substr(7));
        if (camera_calibrations_.size() <= index) {
          camera_calibrations_.resize(index + 1);
        }
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          camera_calibrations_[index].rect_mat =
              Eigen::Matrix3d(parsed_doubles.data()).transpose();
        }
      }
      continue;
    }

    // Projection mat.
    if (header.compare(0, 6, "P_rect") == 0) {
      std::cout << "P_rect header: " << header
                << " substring: " << header.substr(7) << std::endl;
      if (rectified_) {
        // Figure out which number this is.
        int index = std::stoi(header.substr(7));
        if (camera_calibrations_.size() <= index) {
          camera_calibrations_.resize(index + 1);
        }
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          camera_calibrations_[index].projection_mat =
              Eigen::Matrix<double, 4, 3>(parsed_doubles.data()).transpose();

          std::cout << "Projection mat:\n"
                    << camera_calibrations_[index].projection_mat << std::endl;
        }
      }
      continue;
    } else if (!rectified_ && header.compare(0, 2, "P_") == 0) {
      int index = std::stoi(header.substr(2));
      if (camera_calibrations_.size() <= index) {
        camera_calibrations_.resize(index + 1);
      }
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        camera_calibrations_[index].projection_mat =
            Eigen::Matrix<double, 4, 3>(parsed_doubles.data()).transpose();
      }
      continue;
    }

    // Try to load unrectified K, and if using raw images, also load D.
    if (header.compare(0, 1, "K") == 0) {
      int index = std::stoi(header.substr(2));
      if (camera_calibrations_.size() <= index) {
        camera_calibrations_.resize(index + 1);
      }
      // Parse the rotation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Matrix3d K(parsed_doubles.data());
        // All matrices are written row-major but Eigen is column-major
        // (can swap this, but I guess it's anyway easier to just transpose
        // for these small matrices).
        camera_calibrations_[index].K = K.transpose();
      }
      continue;
    }
    if (!rectified_) {
      if (header.compare(0, 1, "D") == 0) {
        int index = std::stoi(header.substr(2));
        if (camera_calibrations_.size() <= index) {
          camera_calibrations_.resize(index + 1);
        }
        // Parse the distortion vector.
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          Eigen::Matrix<double, 1, 5> D(parsed_doubles.data());
          camera_calibrations_[index].D = D;
          camera_calibrations_[index].distorted = !rectified_;
        }
        continue;
      }
    }

    if (header.compare(0, 1, "R") == 0) {
      int index = std::stoi(header.substr(2));
      if (camera_calibrations_.size() <= index) {
        camera_calibrations_.resize(index + 1);
      }
      // Parse the rotation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Matrix3d R(parsed_doubles.data());
        // All matrices are written row-major but Eigen is column-major
        // (can swap this, but I guess it's anyway easier to just transpose
        // for these small matrices).
        camera_calibrations_[index].T_cam0_cam.getRotation() =
            Rotation::fromApproximateRotationMatrix(R.transpose());
      }
      continue;
    } else if (header.compare(0, 1, "T") == 0) {
      int index = std::stoi(header.substr(2));
      if (camera_calibrations_.size() <= index) {
        camera_calibrations_.resize(index + 1);
      }
      // Parse the translation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Vector3d T(parsed_doubles.data());
        camera_calibrations_[index].T_cam0_cam.getPosition() = T;
      }
      continue;
    }
  }
  return true;
}

bool KittiParser::parseVectorOfDoubles(const std::string& input,
                                       std::vector<double>* output) const {
  output->clear();
  // Parse the line as a stringstream for space-delimeted doubles.
  std::stringstream line_stream(input);
  if (line_stream.eof()) {
    return false;
  }

  while (!line_stream.eof()) {
    std::string element;
    std::getline(line_stream, element, ' ');
    if (element.empty()) {
      continue;
    }
    try {
      output->emplace_back(std::stod(element));
    } catch (const std::exception& exception) {
      std::cout << "Could not parse number in import file.\n";
      return false;
    }
  }
  return true;
}

void KittiParser::loadTimestampMaps() {
  // Load timestamps for poses.
  std::string filename =
      dataset_path_ + "/" + kPoseFolder + "/" + kTimestampFilename;
  loadTimestampsIntoVector(filename, &timestamps_pose_ns_);

  std::cout << "Timestmap map for pose:\n";
  for (size_t i = 0; i < timestamps_pose_ns_.size(); ++i) {
    std::cout << i << " " << timestamps_pose_ns_[i] << std::endl;
  }

  // Velodyne.
  filename = dataset_path_ + "/" + kVelodyneFolder + "/" + kTimestampFilename;
  loadTimestampsIntoVector(filename, &timestamps_vel_ns_);

  // One per camera. ORIGINAL
  // timestamps_cam_ns_.resize(camera_calibrations_.size());
  // for (int i = 0; i < camera_calibrations_.size(); ++i) {
  //   filename = dataset_path_ + "/" + getFolderNameForCamera(i) + "/" +
  //              kTimestampFilename;
  //   loadTimestampsIntoVector(filename, &timestamps_cam_ns_[i]);
  // }

  // // One per camera. FOR DSEC
  timestamps_cam_ns_.resize(camera_calibrations_.size());
  for (int i = 0; i < camera_calibrations_.size(); ++i) {
    filename = dataset_path_ + "/" + getFolderNameForCamera(i) + "/" +
               kTimestampFilename;
    loadTimestampsIntoRosTime(filename, &timestamps_cam_ns_[i]);//no ns, acutually us
  }
}

bool KittiParser::loadTimestampsIntoVector(
    const std::string& filename, std::vector<uint64_t>* timestamp_vec) const {
  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }

  timestamp_vec->clear();
  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    std::string timestamp_string = line_stream.str();
    std::tm t = {};
    t.tm_year = std::stoi(timestamp_string.substr(0, 4)) - 1900;
    t.tm_mon = std::stoi(timestamp_string.substr(5, 2)) - 1;
    t.tm_mday = std::stoi(timestamp_string.substr(8, 2));
    t.tm_hour = std::stoi(timestamp_string.substr(11, 2));
    t.tm_min = std::stoi(timestamp_string.substr(14, 2));
    t.tm_sec = std::stoi(timestamp_string.substr(17, 2));
    t.tm_isdst = -1;

    static const uint64_t kSecondsToNanoSeconds = 1e9;
    time_t time_since_epoch = mktime(&t);

    uint64_t timestamp = time_since_epoch * kSecondsToNanoSeconds +
                         std::stoi(timestamp_string.substr(20, 9));
    timestamp_vec->push_back(timestamp);
  }

  std::cout << "Timestamps: " << std::endl
            << timestamp_vec->front() << " " << timestamp_vec->back()
            << std::endl;

  return true;
}

bool KittiParser::loadTimestampsIntoRosTime(
    const std::string& filename, std::vector<uint64_t>* timestamp_vec) const {
  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }

  timestamp_vec->clear();
  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    std::string timestamp_string = line_stream.str();

    static const uint64_t kSecondsToNanoSeconds = 1e3;

    uint64_t timestamp = kSecondsToNanoSeconds * ( 1e2 * std::stoi(timestamp_string.substr(0,9))+std::stoi(timestamp_string.substr(9,2)));
    // std::cout<< "long: "<< timestamp << std::endl;

    timestamp_vec->push_back(timestamp);
  }

  std::cout << "Timestamps: " << std::endl
            << timestamp_vec->front() << " " << timestamp_vec->back()
            << std::endl;

  return true;
}

bool KittiParser::getCameraCalibration(uint64_t cam_id,
                                       CameraCalibration* cam) const {
  if (cam_id >= camera_calibrations_.size()) {
    return false;
  }
  *cam = camera_calibrations_[cam_id];
  return true;
}

bool KittiParser::getPoseAtEntry(uint64_t entry, uint64_t* timestamp,
                                 Transformation* pose) {
  std::string filename = dataset_path_ + "/" + kPoseFolder + "/" + kDataFolder +
                         "/" + getFilenameForEntry(entry) + ".txt";

  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }
  if (timestamps_pose_ns_.size() <= entry) {
    return false;
  }
  *timestamp = timestamps_pose_ns_[entry];

  std::string line;
  std::vector<double> parsed_doubles;
  while (std::getline(import_file, line)) {
    if (parseVectorOfDoubles(line, &parsed_doubles)) {
      if (convertGpsToPose(parsed_doubles, pose)) {
        return true;
      }
    }
  }
  return false;
}

uint64_t KittiParser::getPoseTimestampAtEntry(uint64_t entry) {
  if (timestamps_pose_ns_.size() <= entry) {
    return 0;
  }
  return timestamps_pose_ns_[entry];
}

bool KittiParser::getPointcloudAtEntry(
    uint64_t entry, uint64_t* timestamp,
    pcl::PointCloud<pcl::PointXYZI>* ptcloud) {
  // Get the timestamp for this first.
  if (timestamps_vel_ns_.size() <= entry) {
    std::cout << "Warning: no timestamp for this entry!\n";
    return false;
  }

  *timestamp = timestamps_vel_ns_[entry];

  // Load the actual pointcloud.
  const size_t kMaxNumberOfPoints = 1e6;  // From Readme for raw files.
  ptcloud->clear();
  ptcloud->reserve(kMaxNumberOfPoints);

  std::string filename = dataset_path_ + "/" + kVelodyneFolder + "/" +
                         kDataFolder + "/" + getFilenameForEntry(entry) +
                         ".bin";

  std::ifstream input(filename, std::ios::in | std::ios::binary);
  if (!input) {
    std::cout << "Could not open pointcloud file.\n";
    return false;
  }

  // From yanii's kitti-pcl toolkit:
  // https://github.com/yanii/kitti-pcl/blob/master/src/kitti2pcd.cpp
  for (size_t i = 0; input.good() && !input.eof(); i++) {
    pcl::PointXYZI point;
    input.read((char*)&point.x, 3 * sizeof(float));
    input.read((char*)&point.intensity, sizeof(float));
    ptcloud->push_back(point);
  }
  input.close();
  return true;
}

bool KittiParser::getImageAtEntry(uint64_t entry, uint64_t cam_id,
                                  uint64_t* timestamp, cv::Mat* image) {
  // Get the timestamp for this first.
  if (timestamps_cam_ns_.size() <= cam_id ||
      timestamps_cam_ns_[cam_id].size() <= entry) {
    std::cout << "Warning: no timestamp for this entry!\n";
    // std::cout << "timestamps_cam_ns_.size() = " << timestamps_cam_ns_.size() << std::endl;
    // std::cout << "cam_id = " << cam_id << std::endl;
    // std::cout << "timestamps_cam_ns_[cam_id].size() = " << timestamps_cam_ns_[cam_id].size() << std::endl;
    // std::cout << "entry = " << entry << std::endl;
    return false;
  }
  *timestamp = (timestamps_cam_ns_[cam_id])[entry];
  // std::cout << "entry = " << entry << std::endl;
  std::string filename = dataset_path_ + "/" + getFolderNameForCamera(cam_id) +
                         "/" + kDataFolder + "/" + getFilenameForEntry(entry) +
                         ".png";

  *image = cv::imread(filename, cv::IMREAD_UNCHANGED);

  if (!image->data) {
    std::cout << "Could not load image data.\n";
    return false;
  }
  return true;
}

bool KittiParser::getImageAtEntry_new(uint64_t entry, uint64_t cam_id,
                                  uint64_t* timestamp, cv::Mat* image, ros::Time* timestamp_ros) {
  // Get the timestamp for this first.
  if (timestamps_cam_ns_.size() <= cam_id ||
      timestamps_cam_ns_[cam_id].size() <= entry) {
    std::cout << "Warning: no timestamp for this entry!\n";
    /*std::cout << "timestamps_cam_ns_.size() = " << timestamps_cam_ns_.size() << std::endl;
    std::cout << "cam_id = " << cam_id << std::endl;
    std::cout << "timestamps_cam_ns_[cam_id].size() = " << timestamps_cam_ns_[cam_id].size() << std::endl;
    std::cout << "entry = " << entry << std::endl;*/
    return false;
  }
  *timestamp = (timestamps_cam_ns_[cam_id])[entry];
  // *timestamp_ros = (timestamps_cam_ns_[cam_id])[entry];

  std::string filename = dataset_path_ + "/" + getFolderNameForCamera(cam_id) +
                         "/" + kDataFolder + "/" + getFilenameForEntry(entry) +
                         ".png";

  *image = cv::imread(filename, cv::IMREAD_UNCHANGED);

  if (!image->data) {
    std::cout << "Could not load image data.\n";
    return false;
  }
  return true;
}

bool KittiParser::getImuAtEntry(uint64_t entry, uint64_t* timestamp,
			double* a_flu, double* w_flu) {
  std::string filename = dataset_path_ + "/" + kPoseFolder + "/" + kDataFolder +
                         "/" + getFilenameForEntry(entry) + ".txt";

  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }
  if (timestamps_pose_ns_.size() <= entry) {
    return false;
  }
  *timestamp = timestamps_pose_ns_[entry];

  std::string line;
  std::vector<double> parsed_doubles;
  while (std::getline(import_file, line)) {
    if (parseVectorOfDoubles(line, &parsed_doubles)) {
      if (loadImuDataToDoubles(parsed_doubles, a_flu, w_flu)) {
        return true;
      }
    }
  }
  return false;
}

// From the MATLAB raw data dev kit.
bool KittiParser::convertGpsToPose(const std::vector<double>& oxts,
                                   Transformation* pose) {
  if (oxts.size() < 6) {
    return false;
  }

  double lat = oxts[0];
  double lon = oxts[1];
  double alt = oxts[2];

  double roll = oxts[3];
  double pitch = oxts[4];
  double yaw = oxts[5];

  // Position.
  if (!initial_pose_set_) {
    mercator_scale_ = latToScale(lat);
  }
  Eigen::Vector2d mercator;
  latlonToMercator(lat, lon, mercator_scale_, &mercator);
  Eigen::Vector3d position(mercator.x(), mercator.y(), alt);

  // Rotation.
  const Eigen::AngleAxisd axis_roll(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd axis_pitch(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd axis_yaw(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond rotation = axis_yaw * axis_pitch * axis_roll;

  Transformation transform(position, rotation);

  // Undo the initial transformation, if one is set.
  // If not, set it.
  // The transformation only undoes translation and yaw, not roll and pitch
  // (as these are observable from the gravity vector).
  if (!initial_pose_set_) {
    T_initial_pose_.getPosition() = transform.getPosition();
    T_initial_pose_.getRotation() = Rotation(axis_yaw);
    initial_pose_set_ = true;
  }
  // Get back to local coordinates.
  *pose = T_initial_pose_.inverse() * transform;

  return true;
}

bool KittiParser::loadImuDataToDoubles(const std::vector<double>& oxts, double* a_flu, double* w_flu) {
  if (oxts.size() < 6) {
    return false;
  }

  /*a_flu[0] = oxts[14];
  a_flu[1] = oxts[15];
  a_flu[2] = oxts[16];
  w_flu[0] = oxts[20];
  w_flu[1] = oxts[21];
  w_flu[2] = oxts[22];*/

  a_flu[0] = oxts[11];
  a_flu[1] = oxts[12];
  a_flu[2] = oxts[13];
  w_flu[0] = oxts[17];
  w_flu[1] = oxts[18];
  w_flu[2] = oxts[19];

  return true;
}

// From the MATLAB raw data dev kit.
double KittiParser::latToScale(double lat) const {
  return cos(lat * M_PI / 180.0);
}

// From the MATLAB raw data dev kit.
void KittiParser::latlonToMercator(double lat, double lon, double scale,
                                   Eigen::Vector2d* mercator) const {
  double er = 6378137;
  mercator->x() = scale * lon * M_PI * er / 180.0;
  mercator->y() = scale * er * log(tan((90.0 + lat) * M_PI / 360.0));
}

std::string KittiParser::getFolderNameForCamera(int cam_number) const {
  char buffer[20];
  sprintf(buffer, "%s%02d", kCameraFolder.c_str(), cam_number);
  return std::string(buffer);
}

std::string KittiParser::getFilenameForEntry(uint64_t entry) const {
  char buffer[20];
  sprintf(buffer, "%06llu", entry);
  return std::string(buffer);
}

Transformation KittiParser::T_camN_vel(int cam_number) const {
  return camera_calibrations_[cam_number].T_cam0_cam * T_cam0_vel_;
}

Transformation KittiParser::T_camN_imu(int cam_number) const {
  return T_camN_vel(cam_number) * T_vel_imu_;
}

Transformation KittiParser::T_cam0_vel() const { return T_cam0_vel_; }

Transformation KittiParser::T_vel_imu() const { return T_vel_imu_; }

bool KittiParser::interpolatePoseAtTimestamp(uint64_t timestamp,
                                             Transformation* pose) {
  // Look up the closest 2 timestamps to this.
  size_t left_index = timestamps_pose_ns_.size();
  for (size_t i = 0; i < timestamps_pose_ns_.size(); ++i) {
    if (timestamps_pose_ns_[i] > timestamp) {
      if (i == 0) {
        // Then we can't interpolate the pose since we're outside the range.
        return false;
      }
      left_index = i - 1;
      break;
    }
  }
  if (left_index >= timestamps_pose_ns_.size()) {
    return false;
  }
  // Make sure we don't go over the size
  // if (left_index == timestamps_pose_ns_.size() - 1) {
  //  left_index--;
  //}

  // Figure out what 't' should be, where t = 0 means 100% left boundary,
  // and t = 1 means 100% right boundary.
  double t = (timestamp - timestamps_pose_ns_[left_index]) /
             static_cast<double>(timestamps_pose_ns_[left_index + 1] -
                                 timestamps_pose_ns_[left_index]);

  std::cout << "Timestamp: " << timestamp
            << " timestamp left: " << timestamps_pose_ns_[left_index]
            << " timestamp right: " << timestamps_pose_ns_[left_index + 1]
            << " t: " << t << std::endl;

  // Load the two transformations.
  uint64_t timestamp_left, timestamp_right;
  Transformation transform_left, transform_right;
  if (!getPoseAtEntry(left_index, &timestamp_left, &transform_left) ||
      !getPoseAtEntry(left_index + 1, &timestamp_right, &transform_right)) {
    // For some reason couldn't load the poses.
    return false;
  }

  // Interpolate between them.
  *pose = interpolateTransformations(transform_left, transform_right, t);
  return true;
}

size_t KittiParser::getNumCameras() const {
  return camera_calibrations_.size();
}

}  // namespace kitti
