// Copyright (c) 2015, Autonomous Systems Lab, ETH Zurich
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef EIGEN_CHECKS_TEST_HELPER_H_
#define EIGEN_CHECKS_TEST_HELPER_H_
#include <limits>
#include <random>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <glog/logging.h>

#include <eigen-checks/internal/gtest-equal.h>

template <typename Scalar>
class EigenChecks : public testing::Test {
 protected:
  virtual void SetUp() {
    int random_seed_ = 42;
    std::mt19937 gen(random_seed_);
    std::normal_distribution<> dis(10.0, 5.0);
    Eigen::Matrix<Scalar, 5, 1> perturbance_v;
    perturbance_v << dis(gen), dis(gen), dis(gen), dis(gen), dis(gen);
    perturbance_v /= perturbance_v.maxCoeff();

    ground_truth_vector_5 << dis(gen), dis(gen), dis(gen), dis(gen), dis(gen);

    ground_truth_vector_5_zero.setZero();

    test_vector_D.resize(4, Eigen::NoChange);
    test_vector_D.template block<4, 1>(0, 0) =
        ground_truth_vector_5.template block<4, 1>(0, 0);

    test_vector_5_random << dis(gen), dis(gen), dis(gen), dis(gen), dis(gen);

    test_vector_5_near_e_minus_5 = ground_truth_vector_5;
    test_vector_5_near_e_minus_5 +=
        perturbance_v * static_cast<Scalar>(1e-5);

    test_vector_5_equal = ground_truth_vector_5;

    test_vector_5_equal_floating_point = ground_truth_vector_5;
    test_vector_5_equal_floating_point +=
        perturbance_v * static_cast<Scalar>(0.9 *
            eigen_checks::internal::kDefaultPrecision);

    // Test matrix.
    Eigen::Matrix<Scalar, 5, 4> perturbance_m;
    perturbance_m <<
        dis(gen), dis(gen), dis(gen), dis(gen),
        dis(gen), dis(gen), dis(gen), dis(gen),
        dis(gen), dis(gen), dis(gen), dis(gen),
        dis(gen), dis(gen), dis(gen), dis(gen),
        dis(gen), dis(gen), dis(gen), dis(gen);
    perturbance_m /= perturbance_m.maxCoeff();

    ground_truth_matrix_54 <<
        dis(gen), dis(gen), dis(gen), dis(gen),
        dis(gen), dis(gen), dis(gen), dis(gen),
        dis(gen), dis(gen), dis(gen), dis(gen),
        dis(gen), dis(gen), dis(gen), dis(gen),
        dis(gen), dis(gen), dis(gen), dis(gen);

    ground_truth_matrix_54_zero.setZero();

    test_matrix_D4.resize(4, Eigen::NoChange);
    test_matrix_D4.template block<4, 4>(0, 0) =
        ground_truth_matrix_54.template block<4, 4>(0, 0);

    test_matrix_54_random <<
        dis(gen), dis(gen), dis(gen), dis(gen),
        dis(gen), dis(gen), dis(gen), dis(gen),
        dis(gen), dis(gen), dis(gen), dis(gen),
        dis(gen), dis(gen), dis(gen), dis(gen),
        dis(gen), dis(gen), dis(gen), dis(gen);

    test_matrix_54_near_e_minus_5 = ground_truth_matrix_54;
    test_matrix_54_near_e_minus_5 +=
        perturbance_m * static_cast<Scalar>(1e-5);

    test_matrix_54_equal = ground_truth_matrix_54;

    test_matrix_54_equal_floating_point = ground_truth_matrix_54;
    test_matrix_54_equal_floating_point +=
        perturbance_m * static_cast<Scalar>(0.9 *
            eigen_checks::internal::kDefaultPrecision);
  }

 protected:
  Eigen::Matrix<Scalar, 5, 1> ground_truth_vector_5;
  Eigen::Matrix<Scalar, 5, 1> ground_truth_vector_5_zero;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> test_vector_D;
  Eigen::Matrix<Scalar, 5, 1> test_vector_5_random;
  Eigen::Matrix<Scalar, 5, 1> test_vector_5_near_e_minus_5;
  Eigen::Matrix<Scalar, 5, 1> test_vector_5_equal;
  Eigen::Matrix<Scalar, 5, 1> test_vector_5_equal_floating_point;
  Eigen::Matrix<Scalar, 5, 4> ground_truth_matrix_54;
  Eigen::Matrix<Scalar, 5, 4> ground_truth_matrix_54_zero;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 4> test_matrix_D4;
  Eigen::Matrix<Scalar, 5, 4> test_matrix_54_random;
  Eigen::Matrix<Scalar, 5, 4> test_matrix_54_near_e_minus_5;
  Eigen::Matrix<Scalar, 5, 4> test_matrix_54_equal;
  Eigen::Matrix<Scalar, 5, 4> test_matrix_54_equal_floating_point;
};

typedef ::testing::Types<double, float> ScalarTypes;
TYPED_TEST_CASE(EigenChecks, ScalarTypes);

#define GLOG_TEST_EXPECT_NO_DEATH(X) X

#define GLOG_TEST_EXPECT_DEATH_DIFFERENT_SIZE(X) \
  EXPECT_DEATH(X, "^")  // ^The matrices have a different$

#define GLOG_TEST_EXPECT_DEATH_DIFFERENT_DATA(X) \
  EXPECT_DEATH(X, "^")  // ^The matrices are different$

#endif  // EIGEN_CHECKS_TEST_HELPER_H_
