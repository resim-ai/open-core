#include "transforms/so3.hh"

namespace resim {
namespace transforms {

using TangentVector = SO3::TangentVector;

SO3::SO3(const Eigen::AngleAxisd &angle_axis) :
    rotation_matrix_(angle_axis.toRotationMatrix()) {}

SO3::SO3(const Eigen::Quaterniond &quaternion) :
    rotation_matrix_(quaternion.toRotationMatrix()) {}

SO3::SO3(const Eigen::Matrix3d &rotation_matrix) :
    rotation_matrix_(rotation_matrix) {}

SO3 SO3::identity() { return SO3(Eigen::Matrix3d::Identity()); }

SO3 SO3::operator*(const SO3 &other) const {
    return SO3(rotation_matrix_ * other.rotation_matrix_);
}

Eigen::Vector3d SO3::operator*(const Eigen::Vector3d & source_vector) const {
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
    const TangentVector alg = aa.axis() * aa.angle();
    return alg;
}

TangentVector SO3::adjoint_times(const TangentVector &alg) const {
    return rotation_matrix_ * alg;
}

bool SO3::is_approx(const SO3 &other) const {
    return rotation_matrix_.isApprox(other.rotation_matrix_);
}

const Eigen::Matrix3d &SO3::rotation_matrix() const { return rotation_matrix_; }

} // namespace transforms
} // namespace resim
