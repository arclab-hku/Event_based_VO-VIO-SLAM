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
#ifndef EIGEN_CHECKS_GTEST_H_
#define EIGEN_CHECKS_GTEST_H_
#include <limits>
#include <type_traits>

#include <eigen-checks/internal/gtest-equal.h>

#define EIGEN_MATRIX_EQUAL(MatrixA, MatrixB)                                 \
  eigen_checks::internal::MatricesEqual(MatrixA, #MatrixA, MatrixB, #MatrixB, \
      static_cast<typename std::remove_reference<                            \
      decltype(MatrixA)>::type::Scalar>(0.0), "0.0")

#define EIGEN_MATRIX_EQUAL_DOUBLE(MatrixA, MatrixB)                          \
  eigen_checks::internal::MatricesEqual(MatrixA, #MatrixA, MatrixB, #MatrixB, \
      static_cast<typename std::remove_reference<                            \
      decltype(MatrixA)>::type::Scalar>(                                     \
      eigen_checks::internal::kDefaultPrecision), "Floating point precision")

#define EIGEN_MATRIX_NEAR(MatrixA, MatrixB, Precision)                       \
  eigen_checks::internal::MatricesNear(MatrixA, #MatrixA, MatrixB, #MatrixB, \
                                       Precision, #Precision)

#define EIGEN_MATRIX_ZERO(MatrixA, Precision)                                \
  eigen_checks::internal::MatrixZero(MatrixA, #MatrixA, Precision, #Precision)

#endif  // EIGEN_CHECKS_GTEST_H_
