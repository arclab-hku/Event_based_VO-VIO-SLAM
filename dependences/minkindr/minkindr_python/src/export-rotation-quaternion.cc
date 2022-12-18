#include <kindr/minimal/rotation-quaternion.h>
#include <numpy_eigen/boost_python_headers.hpp>

using namespace boost::python;

typedef kindr::minimal::RotationQuaternionTemplate<double> Quaternion;

Eigen::Vector4d getQuaternionXYZW(const Quaternion& quaternion) {
  return Eigen::Vector4d(
      quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
}

Eigen::Vector4d getQuaternionWXYZ(const Quaternion& quaternion) {
  return Eigen::Vector4d(
      quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
}


Quaternion createQuaternionFromXYZW(const Eigen::Vector4d& xyzw) {
  return Quaternion(xyzw(3), xyzw(0), xyzw(1), xyzw(2));
}

Quaternion createQuaternionFromWXYZ(const Eigen::Vector4d& wxyz) {
  return Quaternion(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
}

Quaternion createQuaternionFromApproximateRotationMatrix(
    const Eigen::Matrix3d& R) {
  return Quaternion::constructAndRenormalize(R);
}

Quaternion createQuaternionFromRotationVectorRads(const Eigen::Vector3d& rotation_vector_xyz_rads) {
  return Quaternion(rotation_vector_xyz_rads);
}

Eigen::Vector3d getRotationVector(const Quaternion& quaternion) {
  const kindr::minimal::AngleAxis angle_axis(quaternion);
  return angle_axis.axis() * angle_axis.angle();
}

void normalize(Quaternion& quaternion) {
  quaternion.normalize();
}

void exportRotationQuaternion() {
  using namespace boost::python;

  class_< Quaternion, boost::shared_ptr<Quaternion> >( "Quaternion", init<>() )
    .def(init<const Eigen::Matrix3d&>())
    .def(init<const double, const double, const double, const double>(
        "Quaternion(w, x, y, z)"))
    .def("w", &Quaternion::w)
    .def("x", &Quaternion::x)
    .def("y", &Quaternion::y)
    .def("z", &Quaternion::z)
    .def("getRotationMatrix", &Quaternion::getRotationMatrix)
    .def("getQuaternionWXYZ", getQuaternionWXYZ)
    .def("getQuaternionXYZW", getQuaternionXYZW)
    .def("inverse", &Quaternion::inverse)
    .def("getRotationVector", getRotationVector)
    .def("normalize", normalize)
    .def(self * self)
    ;

  def("createQuaternionFromXYZW", createQuaternionFromXYZW,
      "Creates a Quaternion from a 4D vector [x,y,z,w].");
  def("createQuaternionFromWXYZ", createQuaternionFromWXYZ,
      "Creates a Quaternion from a 4D vector [w, x,y,z].");
  def("createQuaternionFromApproximateRotationMatrix",
      createQuaternionFromApproximateRotationMatrix,
      "Creates a quaternion from an (approximate) numpy 3x3 rotation matrix.");
  def("createQuaternionFromRotationVectorRads",
      createQuaternionFromRotationVectorRads,
      "Creates a quaternion from a rotation vector [x, y, z] in radians.");
}
