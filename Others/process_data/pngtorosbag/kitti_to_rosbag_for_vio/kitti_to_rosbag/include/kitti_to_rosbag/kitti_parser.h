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

#ifndef KITTI_TO_ROSBAG_KITTI_PARSER_H_
#define KITTI_TO_ROSBAG_KITTI_PARSER_H_

#include <memory>
#include <map>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <opencv2/core/core.hpp>

#include "kitti_to_rosbag/kitti_common.h"

#include <cv_bridge/cv_bridge.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

#include "kitti_to_rosbag/kitti_ros_conversions.h"

namespace kitti {

class KittiParser {
 public:
  // Constants for filenames for calibration files.
  static const std::string kVelToCamCalibrationFilename;
  static const std::string kCamToCamCalibrationFilename;
  static const std::string kImuToVelCalibrationFilename;
  static const std::string kVelodyneFolder;
  static const std::string kCameraFolder;
  static const std::string kPoseFolder;
  static const std::string kTimestampFilename;
  static const std::string kDataFolder;

  KittiParser(const std::string& calibration_path,
              const std::string& dataset_path, bool rectified);

  // MAIN API: all you should need to use!
  // Loading calibration files.
  bool loadCalibration();
  void loadTimestampMaps();

  // Load specific entries (indexed by filename).
  bool getPoseAtEntry(uint64_t entry, uint64_t* timestamp,
                      Transformation* pose);
  uint64_t getPoseTimestampAtEntry(uint64_t entry);

  bool interpolatePoseAtTimestamp(uint64_t timestamp, Transformation* pose);

  bool getGpsAtEntry() { /* TODO! */
    return false;
  }

  bool getImuAtEntry(uint64_t entry, uint64_t* timestamp,
			double* a_flu, double* w_flu);

  bool getPointcloudAtEntry(uint64_t entry, uint64_t* timestamp,
                            pcl::PointCloud<pcl::PointXYZI>* ptcloud);
  bool getImageAtEntry(uint64_t entry, uint64_t cam_id, uint64_t* timestamp,
                       cv::Mat* image);
  bool getImageAtEntry_new(uint64_t entry, uint64_t cam_id, uint64_t* timestamp,
                       cv::Mat* image, ros::Time* timestamp_ros);

  bool getCameraCalibration(uint64_t cam_id, CameraCalibration* cam) const;

  Transformation T_camN_vel(int cam_number) const;
  Transformation T_camN_imu(int cam_number) const;

  // Returns the nanosecond timestamp since epoch for a particular entry.
  // Returns -1 if no valid timestamp is found.
  // int64_t getTimestampNsAtEntry(int64_t entry) const;

  // Basic accessors.
  Transformation T_cam0_vel() const;
  Transformation T_vel_imu() const;

  size_t getNumCameras() const;

 private:
  bool loadCamToCamCalibration();
  bool loadVelToCamCalibration();
  bool loadImuToVelCalibration();

  bool convertGpsToPose(const std::vector<double>& oxts, Transformation* pose);
  double latToScale(double lat) const;
  void latlonToMercator(double lat, double lon, double scale,
                        Eigen::Vector2d* mercator) const;
  bool loadTimestampsIntoVector(const std::string& filename,
                                std::vector<uint64_t>* timestamp_vec) const;
  bool loadTimestampsIntoRosTime(const std::string& filename,
                                std::vector<uint64_t>* timestamp_vec) const;

  bool loadImuDataToDoubles(const std::vector<double>& oxts, double* a_flu, double* w_flu);

  bool parseVectorOfDoubles(const std::string& input,
                            std::vector<double>* output) const;

  std::string getFolderNameForCamera(int cam_number) const;
  std::string getFilenameForEntry(uint64_t entry) const;

  // Base paths.
  std::string calibration_path_;
  std::string dataset_path_;
  // Whether this dataset contains raw or rectified images. This determines
  // which calibration is read.
  bool rectified_;

  // Cached calibration parameters -- std::vector of camera calibrations.
  CameraCalibrationVector camera_calibrations_;

  // Transformation chain (cam-to-cam extrinsics stored above in cam calib
  // struct).
  Transformation T_cam0_vel_;
  Transformation T_vel_imu_;

  // Timestamp map from index to nanoseconds.
  std::vector<uint64_t> timestamps_vel_ns_;
  std::vector<uint64_t> timestamps_pose_ns_;
  // Vector of camera timestamp vectors.
  std::vector<std::vector<uint64_t> > timestamps_cam_ns_;

  // Cached pose information, to correct to odometry frame (instead of absolute
  // world coordinates).
  bool initial_pose_set_;
  Transformation T_initial_pose_;
  double mercator_scale_;
};
}

#endif  // KITTI_TO_ROSBAG_KITTI_PARSER_H_
