#include "resim/transforms/so3.hh"

#include <utility>

#include "resim/assert/assert.hh"
#include "resim/transforms/cross_matrix.hh"
#include "resim/transforms/liegroup.hh"
#include "resim/transforms/liegroup_exp_diff.hh"

namespace resim::transforms {

namespace {
using TangentVector = SO3::TangentVector;
using TangentMapping = SO3::TangentMapping;
using LieGroupSO3 = LieGroup<SO3::DIMS, SO3::DOF>;
using Frame3 = Frame<SO3::DIMS>;
}  //  namespace

template <typename... Args>
SO3::SO3(const Eigen::AngleAxisd &angle_axis, Args... args)
    : LieGroupSO3(std::move(args)...),
      rotation_matrix_(angle_axis.toRotationMatrix()) {}

template <typename... Args>
SO3::SO3(const double angle, const Eigen::Vector3d &axis, Args... args)
    : SO3::SO3(Eigen::AngleAxisd(angle, axis), std::move(args)...) {}

template <typename... Args>
SO3::SO3(const Eigen::Quaterniond &quaternion, Args... args)
    : LieGroupSO3(std::move(args)...),
      rotation_matrix_(quaternion.toRotationMatrix()) {}

template <typename... Args>
SO3::SO3(Eigen::Matrix3d rotation_matrix, Args... args)
    : LieGroupSO3(std::move(args)...),
      rotation_matrix_(std::move(rotation_matrix)) {}

template <typename... Args>
SO3 SO3::identity(Args &&...args) {
  return SO3(Eigen::Matrix3d::Identity(), std::move(args)...);
}

SO3 SO3::operator*(const SO3 &other) const {
  if (!this->is_framed() or !other.is_framed()) {
    // Unless both SO3s are framed make an unframed SO3
    return SO3(rotation_matrix_ * other.rotation_matrix_);
  }
  // Both SO3s are framed so do strong frame checking.
  constexpr auto FRAME_ERR = "Inner frames must match for valid composition";
  REASSERT(from() == other.into(), FRAME_ERR);
  return SO3(rotation_matrix_ * other.rotation_matrix_, into(), other.from());
}

Eigen::Vector3d SO3::operator*(const Eigen::Vector3d &source_vector) const {
  return rotation_matrix_ * source_vector;
}

Eigen::Vector3d SO3::rotate(const Eigen::Vector3d &source_vector) const {
  return rotation_matrix_ * source_vector;
}

FramedVector<SO3::DIMS> SO3::rotate(
    const FramedVector<SO3::DIMS> &source_vector) const {
  constexpr auto UNFRAMED_ERR =
      "Please don't use unframed SO3s on framed vectors, we cannot return "
      "meaningful results. Pass a regular Eigen Vector to rotate instead.";
  REASSERT(this->is_framed(), UNFRAMED_ERR);
  constexpr auto FRAME_ERR = "Vector frame must match the from frame.";
  REASSERT(from() == source_vector.frame(), FRAME_ERR);
  return FramedVector<SO3::DIMS>(rotate(source_vector.vector()), into());
}

SO3 SO3::inverse() const {
  return SO3(rotation_matrix_.transpose(), from(), into());
}

SO3 SO3::interp(const double fraction) const {
  return SO3::exp(this->log() * fraction, into(), from());
}

SO3 SO3::interp(double fraction, const Frame<SO3::DIMS> &new_from) const {
  constexpr auto UNFRAMED_ERR =
      "Please don't interpolate unframed SO3s with a new from frame, we cannot "
      "return meaningful results. Use interp(double fraction) instead.";
  REASSERT(this->is_framed(), UNFRAMED_ERR);
  return SO3::exp(this->log() * fraction, into(), new_from);
}

template <typename... Args>
SO3 SO3::exp(const TangentVector &alg, Args &&...args) {
  return SO3(
      Eigen::AngleAxisd(alg.norm(), alg.normalized()),
      std::forward<Args>(args)...);
}

SO3::TangentMapping SO3::exp_diff(const TangentVector &alg) {
  const double theta_sq = alg.squaredNorm();
  const ExpDiffCoeffs coeffs{derivative_of_exp_so3(theta_sq)};
  return coeffs.a * SO3::TangentMapping::Identity() +
         coeffs.b * cross_matrix(alg) + coeffs.c * alg * alg.transpose();
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
  return this->is_approx_transform(other) &&
         this->verify_frames(other.into(), other.from());
}

bool SO3::is_approx_transform(const SO3 &other) const {
  return rotation_matrix_.isApprox(other.rotation_matrix_);
}

const Eigen::Matrix3d &SO3::rotation_matrix() const { return rotation_matrix_; }

Eigen::Quaterniond SO3::quaternion() const {
  return Eigen::Quaterniond{rotation_matrix()};
}

template SO3::SO3(const Eigen::AngleAxisd &);
template SO3::SO3(const Eigen::AngleAxisd &, Frame3, Frame3);

template SO3::SO3(double, const Eigen::Vector3d &);
template SO3::SO3(double, const Eigen::Vector3d &, Frame3, Frame3);

template SO3::SO3(const Eigen::Quaterniond &);
template SO3::SO3(const Eigen::Quaterniond &, Frame3, Frame3);

template SO3::SO3(Eigen::Matrix3d);
template SO3::SO3(Eigen::Matrix3d, Frame3, Frame3);

template SO3 SO3::identity();
template SO3 SO3::identity(const Frame3 &, const Frame3 &);
template SO3 SO3::identity(Frame3 &&, Frame3 &&);
template SO3 SO3::identity(const Frame3 &, Frame3 &&);
template SO3 SO3::identity(Frame3 &&, const Frame3 &);

template SO3 SO3::exp(const TangentVector &);
template SO3 SO3::exp(const TangentVector &, const Frame3 &, const Frame3 &);
template SO3 SO3::exp(const TangentVector &, Frame3 &&, Frame3 &&);
template SO3 SO3::exp(const TangentVector &, const Frame3 &, Frame3 &&);
template SO3 SO3::exp(const TangentVector &, Frame3 &&, const Frame3 &);

}  // namespace resim::transforms
