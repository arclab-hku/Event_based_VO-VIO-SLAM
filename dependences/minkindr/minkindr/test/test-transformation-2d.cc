#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen-checks/gtest.h>
#include <gtest/gtest.h>

#include "kindr/minimal/transform-2d.h"

#ifndef TEST
#define TEST(a, b) int Test_##a##_##b()
#endif

namespace kindr {
namespace minimal {

constexpr double kEpsilon = 1e-10;
static const Rotation2D r(0.77);
static const Position2D t(1.23597132, -2.247102);
static Eigen::Vector2d v(6.26257419, 1.58356548);

TEST(TestTransformation2D, TestInitialization) {
  const Transformation2D T;
  EXPECT_NEAR(T.getRotation().angle(), 0.0, kEpsilon);
  EXPECT_TRUE(
      EIGEN_MATRIX_NEAR(T.getPosition(), Eigen::Vector2d(0.0, 0.0), kEpsilon));

  const Transformation2D T_rot_pos(r, t);
  EXPECT_NEAR(T_rot_pos.getRotation().angle(), r.angle(), kEpsilon);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(T_rot_pos.getPosition(), t, kEpsilon));

  const Eigen::Matrix<double, 3, 3> transformation_matrix =
      (Eigen::Matrix<double, 3, 3>() << std::cos(r.angle()),
       -std::sin(r.angle()), t.x(), std::sin(r.angle()), std::cos(r.angle()),
       t.y(), 0.0, 0.0, 1.0)
          .finished();
  const Transformation2D T_from_mat(transformation_matrix);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      transformation_matrix, T_from_mat.getTransformationMatrix(), kEpsilon));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      transformation_matrix, T_rot_pos.getTransformationMatrix(), kEpsilon));

  const Eigen::Matrix<double, 3, 3> invalid_transformation_matrix =
      (Eigen::Matrix<double, 3, 3>() << 1.0, 1.0, t.x(), 1.0, 1.0, t.y(), 0.0,
       0.0, 1.1)
          .finished();
  EXPECT_DEATH(
      const Transformation2D T_invalid(invalid_transformation_matrix), "");
}

TEST(TestTransformation2D, TestGettersAndSetters) {
  Transformation2D T;
  T.getRotation() = r;
  T.getPosition() = t;
  EXPECT_NEAR(T.getRotation().angle(), r.angle(), kEpsilon);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(T.getPosition(), t, kEpsilon));

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      T.asVector(), (Eigen::Vector3d() << r.angle(), t).finished(), kEpsilon));

  constexpr double angle_new = 3.1415;
  T.getRotation().angle() = angle_new;
  EXPECT_NEAR(T.getRotation().angle(), angle_new, kEpsilon);
}

Eigen::Vector2d fromHomogeneous(const Eigen::Vector3d& v) {
  return v.head<2>() / v[2];
}

Eigen::Vector3d toHomogeneous(const Eigen::Vector2d& v) {
  return (Eigen::Vector3d() << v, 1.0).finished();
}

TEST(TestTransformation2D, TestComposition) {
  const Eigen::Vector3d vh = toHomogeneous(v);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(v, fromHomogeneous(vh), kEpsilon));

  const Transformation2D T(r, t);
  const Eigen::Matrix<double, 3, 3> T_mat = T.getTransformationMatrix();
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(T * v, fromHomogeneous(T_mat * vh), kEpsilon));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      T * T * v, fromHomogeneous(T_mat * T_mat * vh), kEpsilon));

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      T.inverse().getTransformationMatrix(), T_mat.inverse().eval(), kEpsilon));

  const Eigen::MatrixXd v_vectorized = Eigen::MatrixXd::Random(2, 10);
  const Eigen::MatrixXd v_result = T.transformVectorized(v_vectorized);
  for (int i = 0; i < v_vectorized.cols(); ++i) {
    EXPECT_TRUE(
        EIGEN_MATRIX_NEAR(v_result.col(i), T * v_vectorized.col(i), kEpsilon));
  }
}

TEST(TestTransformation2D, TestCast) {
  const Transformation2D T_double(r, t);
  const Transformation2DTemplate<float> T_float =
      T_double.template cast<float>();

  EXPECT_NEAR(
      T_double.getRotation().angle(),
      static_cast<double>(T_float.getRotation().angle()), 1e-5);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
      T_double.getPosition(), T_float.getPosition().cast<double>(), 1e-5));
}

}  // namespace minimal
}  // namespace kindr
