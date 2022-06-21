#include "transforms/se3.hh"

#include <utility>

#include "transforms/liegroup_exp_diff.hh"

namespace resim::transforms {

using TangentVector = SE3::TangentVector;

SE3::SE3(SO3 rotation)
    : rotation_(std::move(rotation)),
      translation_(Eigen::Vector3d::Zero()) {}

SE3::SE3(Eigen::Vector3d translation)
    : rotation_(SO3::identity()),
      translation_(std::move(translation)) {}

SE3::SE3(SO3 rotation, Eigen::Vector3d translation)
    : rotation_(std::move(rotation)),
      translation_(std::move(translation)) {}

SE3 SE3::identity() { return SE3(SO3::identity(), Eigen::Vector3d::Zero()); }

SE3 SE3::operator*(const SE3 &other) const {
  return SE3(
      rotation_ * other.rotation_,
      (rotation_ * other.translation_) + translation_);
}

Eigen::Vector3d SE3::operator*(const Eigen::Vector3d &source_vector) const {
  return rotation_ * source_vector + translation_;
}

SE3 SE3::inverse() const {
  const SO3 inverse_rotation = rotation_.inverse();
  return SE3(inverse_rotation, inverse_rotation * -translation_);
}

double SE3::arc_length() const {
  return tangent_vector_translation_part(this->log()).norm();
}

SE3 SE3::interp(const double fraction) const {
  return SE3::exp(this->log() * fraction);
}

SE3 SE3::exp(const TangentVector &alg) {
  const SO3::TangentVector alg_rot = tangent_vector_rotation_part(alg);
  const Eigen::Vector3d alg_trans = tangent_vector_translation_part(alg);
  const ExpDiffCoeffs coeffs = derivative_of_exp_so3(alg_rot.squaredNorm());
  const Eigen::Vector3d translation =
      coeffs.a * alg_trans + coeffs.b * alg_rot.cross(alg_trans) +
      coeffs.c * alg_rot.dot(alg_trans) * alg_rot;
  return SE3(SO3::exp(alg_rot), translation);
}

TangentVector SE3::log() const {
  // Some constants:
  // Note the TINY_SQUARE_ANGLE const comes from liegroup_exp_diff.hh
  constexpr double LARGE_SQUARE_ANGLE = 9.;
  constexpr double TINY_COEFF_A = 1. / 12;
  constexpr double TINY_COEFF_B = 1. / 720;
  constexpr double TINY_COEFF_C = 1. / 30240;
  constexpr double HALF = 0.5;

  const SO3::TangentVector alg_rot = rotation_.log();
  const double square_angle = alg_rot.squaredNorm();
  const ExpDiffCoeffs coeffs = derivative_of_exp_so3(square_angle);

  double order_2_coeff = 0;
  if (square_angle < TINY_SQUARE_ANGLE) {
    order_2_coeff =
        TINY_COEFF_A                                                    //
        + square_angle * (TINY_COEFF_B + square_angle + TINY_COEFF_C);  //
  } else if (square_angle > LARGE_SQUARE_ANGLE) {
    order_2_coeff = (coeffs.b - (HALF * coeffs.a)) / (coeffs.b * square_angle);
  } else {
    order_2_coeff = (coeffs.b * HALF - coeffs.c) / coeffs.a;
  }
  const Eigen::Vector3d rot_x_trans = alg_rot.cross(translation_);
  const Eigen::Vector3d alg_trans =
      translation_                                   //
      - HALF * rot_x_trans                           //
      + order_2_coeff * alg_rot.cross(rot_x_trans);  //
  return tangent_vector_from_parts(alg_rot, alg_trans);
}

TangentVector SE3::adjoint_times(const TangentVector &alg) const {
  const Eigen::Matrix3d &rotation = rotation_.rotation_matrix();
  const SO3::TangentVector alg_rot = tangent_vector_rotation_part(alg);
  const Eigen::Vector3d alg_trans = tangent_vector_translation_part(alg);
  const Eigen::Vector3d rotated_alg = rotation * alg_rot;
  return tangent_vector_from_parts(
      rotated_alg,
      translation_.cross(rotated_alg) + (rotation * alg_trans));
}

bool SE3::is_approx(const SE3 &other) const {
  return rotation_.is_approx(other.rotation_) &&
         translation_.isApprox(other.translation_);
}

const SO3 &SE3::rotation() const { return rotation_; }

const Eigen::Vector3d &SE3::translation() const { return translation_; }

SO3::TangentVector SE3::tangent_vector_rotation_part(const TangentVector &alg) {
  return alg.head<3>();
}

Eigen::Vector3d SE3::tangent_vector_translation_part(const TangentVector &alg) {
  return alg.tail<3>();
}

TangentVector SE3::tangent_vector_from_parts(
    const SO3::TangentVector &alg_rot,
    const Eigen::Vector3d &alg_trans) {
  TangentVector alg;
  alg.head<3>() = alg_rot;
  alg.tail<3>() = alg_trans;
  return alg;
}

}  // namespace resim::transforms
