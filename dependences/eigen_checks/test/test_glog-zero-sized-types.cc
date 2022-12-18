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
#include <eigen-checks/entrypoint.h>
#include <eigen-checks/glog.h>
#include <gtest/gtest.h>

#include "test_helper.h"

TEST(EigenChecks, EigenMatrixEqualGlog_Zero_Sized_Dynamic_Matrix) {
  Eigen::Matrix2Xd A;
  Eigen::Matrix2Xd B(2, 3);
  GLOG_TEST_EXPECT_DEATH_DIFFERENT_DATA(CHECK_EIGEN_MATRIX_NEAR(A, B, 0.01));
  GLOG_TEST_EXPECT_NO_DEATH(CHECK_EIGEN_MATRIX_NEAR(A, A, 0.01));
}

TEST(EigenChecks, EigenVectorEqualGlog_Zero_Sized_Dynamic_Vector) {
  Eigen::VectorXd A;
  Eigen::VectorXd B(3);
  GLOG_TEST_EXPECT_DEATH_DIFFERENT_DATA(CHECK_EIGEN_MATRIX_NEAR(A, B, 0.01));
  GLOG_TEST_EXPECT_NO_DEATH(CHECK_EIGEN_MATRIX_NEAR(A, A, 0.01));
}

UNITTEST_ENTRYPOINT

