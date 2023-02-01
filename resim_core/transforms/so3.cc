#include "resim_core/transforms/so3.hh"

#include <utility>

#include "resim_core/transforms/cross_matrix.hh"

namespace resim::transforms {

using TangentVector = SO3::TangentVector;
using TangentMapping = SO3::TangentMapping;

SO3::SO3(const Eigen::AngleAxisd &angle_axis)
    : rotation_matrix_(angle_axis.toRotationMatrix()) {}

SO3::SO3(const double angle, const Eigen::Vector3d &axis)
    : SO3::SO3{Eigen::AngleAxisd(angle, axis)} {}

SO3::SO3(const Eigen::Quaterniond &quaternion)
    : rotation_matrix_(quaternion.toRotationMatrix()) {}

SO3::SO3(Eigen::Matrix3d rotation_matrix)
    : rotation_matrix_(std::move(rotation_matrix)) {}

SO3 SO3::identity() { return SO3(Eigen::Matrix3d::Identity()); }

SO3 SO3::operator*(const SO3 &other) const {
  return SO3(rotation_matrix_ * other.rotation_matrix_);
}

Eigen::Vector3d SO3::operator*(const Eigen::Vector3d &source_vector) const {
  return rotation_matrix_ * source_vector;
}

SO3 SO3::inverse() const { return SO3(rotation_matrix_.transpose()); }

SO3 SO3::interp(const double fraction) const {
  return SO3::exp(this->log() * fraction);
}

SO3 SO3::exp(const TangentVector &alg) {
  return SO3(Eigen::AngleAxisd(alg.norm(), alg.normalized()));
}

TangentVector SO3::log() const {
  const Eigen::AngleAxisd aa(rotation_matrix_);
  TangentVector alg = aa.axis() * aa.angle();
  return alg;
}

TangentMapping SO3::adjoint() const { return rotation_matrix_; }

TangentMapping SO3::adjoint(const TangentVector &alg) {
  return cross_matrix(alg);
}

TangentVector SO3::adjoint_times(const TangentVector &alg) const {
  return rotation_matrix_ * alg;
}

TangentVector SO3::adjoint_times(
    const TangentVector &alg_0,
    const TangentVector &alg_1) {
  return alg_0.cross(alg_1);
}

bool SO3::is_approx(const SO3 &other) const {
  return rotation_matrix_.isApprox(other.rotation_matrix_);
}

const Eigen::Matrix3d &SO3::rotation_matrix() const { return rotation_matrix_; }

Eigen::Quaterniond SO3::quaternion() const {
  return Eigen::Quaterniond{rotation_matrix()};
}

}  // namespace resim::transforms
