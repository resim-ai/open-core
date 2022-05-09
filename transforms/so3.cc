#include "transforms/so3.hh"

namespace resim {
namespace transforms {

using TangentVector = SO3::TangentVector;

SO3::SO3(const Eigen::AngleAxisd &angle_axis) :
    rotation_matrix_(angle_axis.toRotationMatrix()) {}

SO3::SO3(const Eigen::Matrix3d &rotation_matrix) :
    rotation_matrix_(rotation_matrix) {}

SO3 SO3::identity() { return SO3(Eigen::Matrix3d::Identity()); }

const Eigen::Matrix3d &SO3::rotation_matrix() const { return rotation_matrix_; }

SO3 SO3::exp(const TangentVector &alg) {
    return SO3(Eigen::AngleAxisd(alg.norm(), alg.normalized()));
}

TangentVector SO3::log() const {
    const Eigen::AngleAxisd aa(rotation_matrix_);
    const TangentVector alg = aa.axis() * aa.angle();
    return alg;
}

bool SO3::is_approx(const SO3 &other) const {
    return rotation_matrix_.isApprox(other.rotation_matrix_);
}

} // namespace transforms
} // namespace resim
