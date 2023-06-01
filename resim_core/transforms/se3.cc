#include "resim_core/transforms/se3.hh"

#include <utility>

#include "resim_core/transforms/cross_matrix.hh"
#include "resim_core/transforms/liegroup_exp_diff.hh"

namespace resim::transforms {

namespace {
using TangentVector = SE3::TangentVector;
using TangentMapping = SE3::TangentMapping;
using LieGroupSE3 = LieGroup<SE3::DIMS, SE3::DOF>;
using Frame3 = Frame<SE3::DIMS>;
}  //  namespace

SE3::SE3(SO3 rotation)
    : rotation_(std::move(rotation)),
      translation_(Eigen::Vector3d::Zero()) {
  // SE3 is the sole owner of the frames for the transformation.
  rotation_.set_unframed();
}

SE3::SE3(SO3 rotation, Frame<SE3::DIMS> into, Frame<SE3::DIMS> from)
    : LieGroupSE3(into, from),
      rotation_(std::move(rotation)),
      translation_(Eigen::Vector3d::Zero()) {
  // SE3 is the sole owner of the frames for the transformation.
  rotation_.set_unframed();
}

template <typename... Args>
SE3::SE3(Eigen::Vector3d translation, Args... args)
    : LieGroupSE3(std::move(args)...),
      rotation_(SO3::identity()),
      translation_(std::move(translation)) {}

template <typename... Args>
SE3::SE3(SO3 rotation, Eigen::Vector3d translation, Args... args)
    : LieGroupSE3(std::move(args)...),
      rotation_(std::move(rotation)),
      translation_(std::move(translation)) {
  // SE3 is the sole owner of the frames for the transformation.
  rotation_.set_unframed();
}

template <typename... Args>
SE3 SE3::identity(Args &&...args) {
  return SE3(
      SO3::identity(),
      Eigen::Vector3d::Zero(),
      std::forward<Args>(args)...);
}

SE3 SE3::operator*(const SE3 &other) const {
  SO3 rotation = rotation_ * other.rotation_;
  Eigen::Vector3d translation = (rotation_ * other.translation_) + translation_;
  if (!this->is_framed() or !other.is_framed()) {
    // Unless both SE3s are framed make an unframed SE3
    return SE3(std::move(rotation), std::move(translation));
  }
  // Both SE3s are framed so do strong frame checking.
  constexpr auto FRAME_ERR = "Inner frames must match for valid composition";
  REASSERT(from() == other.into(), FRAME_ERR);
  return SE3(std::move(rotation), std::move(translation), into(), other.from());
}

Eigen::Vector3d SE3::operator*(const Eigen::Vector3d &source_vector) const {
  return rotation_ * source_vector + translation_;
}

Eigen::Vector3d SE3::rotate(const Eigen::Vector3d &source_vector) const {
  return rotation_ * source_vector;
}

FramedVector<SE3::DIMS> SE3::rotate(
    const FramedVector<SE3::DIMS> &source_vector) const {
  constexpr auto UNFRAMED_ERR =
      "Please don't use unframed SE3s on framed vectors, we cannot return "
      "meaningful results. Pass a regular Eigen Vector to rotate instead.";
  REASSERT(this->is_framed(), UNFRAMED_ERR);
  constexpr auto FRAME_ERR = "Vector frame must match the from frame.";
  REASSERT(from() == source_vector.frame(), FRAME_ERR);
  return FramedVector<SE3::DIMS>(rotate(source_vector.vector()), into());
}

SE3 SE3::inverse() const {
  const SO3 inverse_rotation = rotation_.inverse();
  return SE3(
      inverse_rotation,
      inverse_rotation * -translation_,
      from(),
      into());
}

double SE3::arc_length() const {
  return tangent_vector_translation_part(this->log()).norm();
}

SE3 SE3::interp(const double fraction) const {
  return SE3::exp(this->log() * fraction, into(), from());
}

SE3 SE3::interp(double fraction, const Frame<SE3::DIMS> &new_from) const {
  constexpr auto UNFRAMED_ERR =
      "Please don't interpolate unframed SE3s with a new from frame, we cannot "
      "return meaningful results. Use interp(double fraction) instead.";
  REASSERT(this->is_framed(), UNFRAMED_ERR);
  return SE3::exp(this->log() * fraction, into(), new_from);
}

template <typename... Args>
SE3 SE3::exp(const TangentVector &alg, Args &&...args) {
  const SO3::TangentVector alg_rot = tangent_vector_rotation_part(alg);
  const Eigen::Vector3d alg_trans = tangent_vector_translation_part(alg);
  const ExpDiffCoeffs coeffs = derivative_of_exp_so3(alg_rot.squaredNorm());
  const Eigen::Vector3d translation =
      coeffs.a * alg_trans + coeffs.b * alg_rot.cross(alg_trans) +
      coeffs.c * alg_rot.dot(alg_trans) * alg_rot;
  return SE3(SO3::exp(alg_rot), translation, std::forward<Args>(args)...);
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

TangentMapping SE3::adjoint() const {
  TangentMapping adj;
  const Eigen::Matrix3d adj_rot = rotation_.adjoint();
  adj.block<3, 3>(0, 0) = adj_rot;
  adj.block<3, 3>(0, 3).setZero();
  adj.block<3, 3>(3, 0) = cross_matrix(translation_) * adj_rot;
  adj.block<3, 3>(3, 3) = adj_rot;
  return adj;
}

TangentMapping SE3::adjoint(const TangentVector &alg) {
  const SO3::TangentVector alg_rot = tangent_vector_rotation_part(alg);
  const Eigen::Vector3d alg_trans = tangent_vector_translation_part(alg);
  const SO3::TangentMapping adj_rot = SO3::adjoint(alg_rot);
  const SO3::TangentMapping adj_trans = SO3::adjoint(alg_trans);
  TangentMapping adj;
  adj.block<3, 3>(0, 0) = adj_rot;
  adj.block<3, 3>(0, 3).setZero();
  adj.block<3, 3>(3, 0) = adj_trans;
  adj.block<3, 3>(3, 3) = adj_rot;
  return adj;
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

TangentVector SE3::adjoint_times(
    const TangentVector &alg_0,
    const TangentVector &alg_1) {
  const SO3::TangentVector alg_0_rot = tangent_vector_rotation_part(alg_0);
  const SO3::TangentVector alg_1_rot = tangent_vector_rotation_part(alg_1);
  const Eigen::Vector3d alg_0_trans = tangent_vector_translation_part(alg_0);
  const Eigen::Vector3d alg_1_trans = tangent_vector_translation_part(alg_1);

  return tangent_vector_from_parts(
      alg_0_rot.cross(alg_1_rot),
      alg_0_trans.cross(alg_1_rot) + alg_0_rot.cross(alg_1_trans));
}

bool SE3::is_approx(const SE3 &other) const {
  return this->is_approx_transform(other) &&
         this->verify_frames(other.into(), other.from());
}

bool SE3::is_approx_transform(const SE3 &other) const {
  return rotation_.is_approx(other.rotation_) &&
         translation_.isApprox(other.translation_);
}

const SO3 &SE3::rotation() const { return rotation_; }

const Eigen::Vector3d &SE3::translation() const { return translation_; }

Eigen::VectorBlock<const TangentVector, SE3::DIMS>
SE3::tangent_vector_rotation_part(const TangentVector &alg) {
  return alg.head<SE3::DIMS>();
}

Eigen::VectorBlock<TangentVector, SE3::DIMS> SE3::tangent_vector_rotation_part(
    TangentVector &alg) {
  return alg.head<SE3::DIMS>();
}

Eigen::VectorBlock<const TangentVector, SE3::DIMS>
SE3::tangent_vector_translation_part(const TangentVector &alg) {
  return alg.tail<SE3::DIMS>();
}

Eigen::VectorBlock<TangentVector, SE3::DIMS>
SE3::tangent_vector_translation_part(TangentVector &alg) {
  return alg.tail<SE3::DIMS>();
}

TangentVector SE3::tangent_vector_from_parts(
    const SO3::TangentVector &alg_rot,
    const Eigen::Vector3d &alg_trans) {
  TangentVector alg;
  alg.head<3>() = alg_rot;
  alg.tail<3>() = alg_trans;
  return alg;
}

template SE3::SE3(Eigen::Vector3d);
template SE3::SE3(Eigen::Vector3d, Frame3, Frame3);

template SE3::SE3(SO3, Eigen::Vector3d);
template SE3::SE3(SO3, Eigen::Vector3d, Frame3, Frame3);

template SE3 SE3::identity();
template SE3 SE3::identity(const Frame3 &, const Frame3 &);

template SE3 SE3::exp(const TangentVector &);
template SE3 SE3::exp(const TangentVector &, const Frame3 &, const Frame3 &);

}  // namespace resim::transforms
