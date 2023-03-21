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

#ifndef KITTI_TO_ROSBAG_KITTI_COMMON_H_
#define KITTI_TO_ROSBAG_KITTI_COMMON_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <kindr/minimal/quat-transformation.h>

namespace kitti {

// Transformation type for defining sensor orientation.
typedef kindr::minimal::QuatTransformation Transformation;
typedef kindr::minimal::RotationQuaternion Rotation;

struct CameraCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Intrinsics.
  Eigen::Vector2d image_size = Eigen::Vector2d::Zero();  // S_xx in calibration.
  Eigen::Matrix3d rect_mat =
      Eigen::Matrix3d::Identity();  // R_rect_xx in calibration.
  Eigen::Matrix<double, 3, 4> projection_mat =
      Eigen::Matrix<double, 3, 4>::Identity();  // P_xx in calibration.

  // Unrectified (raw) intrinsics. Should only be used if rectified set to
  // false.
  Eigen::Matrix3d K =
      Eigen::Matrix3d::Zero();  // Camera intrinsics, K_xx in calibration.
  Eigen::Matrix<double, 1, 5> D =
      Eigen::Matrix<double, 1,
                    5>::Zero();  // Distortion parameters, radtan model.

  // Extrinsics.
  Transformation T_cam0_cam;

  bool distorted = false;
};

typedef std::vector<CameraCalibration,
                    Eigen::aligned_allocator<CameraCalibration> >
    CameraCalibrationVector;

// Where t = 0 means 100% left transformation,
// and t = 1 means 100% right transformation.
Transformation interpolateTransformations(const Transformation& left,
                                          const Transformation& right,
                                          double t);

}  // namespace kitti

#endif  // KITTI_TO_ROSBAG_KITTI_COMMON_H_
