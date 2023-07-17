#pragma once

#include <Eigen/Dense>

#include "resim/transforms/liegroup.hh"
#include "resim/transforms/liegroup_concepts.hh"
#include "resim/transforms/so3.hh"

namespace resim::transforms {

// Special Euclidean Group in 3-Dimensional space.
//
// Practical implementation of the LieGroup representing rigid
// six-degree-of-freedom transformations in 3-Dimensional space.
//
// Based on the LieGroups library and documentation available at:
// https://ethaneade.com/
//
// Example usages:
//     const SE3 global_from_robot(robot_pose_source);
//     // Apply the rigid transform to a vector by multiplication.
//     const Eigen::Vector3d point_on_robot;
//     const Eigen::Vector3d point_in_global_frame
//           = global_from_robot * point_on_robot;
//     // Compose two SE3s by multiplication.
//     const SE3 robot_from_sensor(sensor_pose_source);
//     const SE3 global_from_sensor = global_from_robot * robot_from_sensor;
//     // Interpolate a rigid transformation along the geodesic curve:
//     const SE3 global_from_robot_halfway = global_from_robot.interp(0.5):
// NOLINTNEXTLINE(readability-magic-numbers)
class SE3 : public LieGroup<3, 6> {
 public:
  SE3() = default;

  // Constructor
  // Create an SE3 with rotation only, from an SO3.
  // Please note: if you provide a framed SO3 the frames will be stripped. SE3
  // is the owner of the transform frames and they should be set explicitly.
  explicit SE3(SO3 rotation);

  // Constructor
  // Create a framed SE3 with rotation only, from an SO3.
  // Please note: if you provide a framed SO3 the frames will be stripped. SE3
  // is the owner of the transform frames and they should be set explicitly.
  SE3(SO3 rotation, Frame<SE3::DIMS> into, Frame<SE3::DIMS> from);

  // Constructor
  // Create an SE3 with translation only, from a 3-Vector.
  template <typename... Args>
  explicit SE3(Eigen::Vector3d translation, Args... args);

  // Constructor
  // Create an SE3 with both a rotation and translation component.
  // Please note: if you provide a framed SO3 the frames will be stripped. SE3
  // is the owner of the transform frames and they should be set explicitly.
  template <typename... Args>
  SE3(SO3 rotation, Eigen::Vector3d translation, Args... args);

  // Get an identity SE3
  template <typename... Args>
  static SE3 identity(Args &&...args);

  // TODO(https://app.asana.com/0/1203294986954613/1204516240910819/f)
  // Operator*
  // Compose this SE3 with another (multiplication)
  virtual SE3 operator*(const SE3 &other) const;

  // Operator*
  // Apply the SE3 action to a vector in 3-Dimensional space
  // (multiplication)
  Eigen::Vector3d operator*(const Eigen::Vector3d &source_vector) const;

  // Equivalent to .rotation()*
  // Providing an explicit interface to apply only the rotation part of the
  // transformation action to a vector.
  Eigen::Vector3d rotate(const Eigen::Vector3d &source_vector) const;

  // Apply the Group rotation to a FramedVector, this will change the frame
  // of the vector accordingly. This method will fail if the source_vector
  // frame does not match from().
  FramedVector<SE3::DIMS> rotate(
      const FramedVector<SE3::DIMS> &source_vector) const;

  // Return the inverse of this SE3.
  SE3 inverse() const;

  // Return the length of the geodesic curve between frames in the
  // transformation.
  double arc_length() const;

  // Interpolate this SE3
  // [param] fraction - interpolation is over a unit interval, where
  // fraction=0 returns identity and fraction=1 returns this SE3. In between
  // the SE3 returned is a linear interpolation. If fraction is greater than 1
  // or less than 0, a linear extrapolation will be returned. The into() and
  // from() frames are preserved.
  SE3 interp(double fraction) const;

  // Interpolate the SE3, returning a framed SE3 with a user-provided
  // from() frame. The into() frame is preserved.
  SE3 interp(double fraction, const Frame<SE3::DIMS> &new_from) const;

  // Create an SE3 from an element of the LieGroup algebra.
  template <typename... Args>
  static SE3 exp(const TangentVector &alg, Args &&...args);

  // Get the differential of the exponential map at alg.
  static TangentMapping exp_diff(const TangentVector &alg);

  // Retrieve the element of the LieGroup algebra that represents
  // this group element.
  TangentVector log() const;

  // Adjoint representation of this group element.
  TangentMapping adjoint() const;

  // Adjoint representation of a given algebra element.
  static TangentMapping adjoint(const TangentVector &alg);

  // Transform a TangentVector from the right tangent space to the left.
  TangentVector adjoint_times(const TangentVector &alg) const;

  // Adjoint times for algebra elements.
  static TangentVector adjoint_times(
      const TangentVector &alg_0,
      const TangentVector &alg_1);

  // Test for floating-point equality with another SE3.
  bool is_approx(const SE3 &other) const;

  // Test for floating-point equality with another SE3, ignoring frames.
  bool is_approx_transform(const SE3 &other) const;

  // Getter
  // Return the rotational part of the transform.
  const SO3 &rotation() const;

  // Getter
  // Return the translational part of the transform.
  const Eigen::Vector3d &translation() const;

  // Some TangentVector helpers:

  // Helper to split out the rotation part of an SE3::TangentVector.
  static Eigen::VectorBlock<const TangentVector, DIMS>
  tangent_vector_rotation_part(const TangentVector &alg);

  static Eigen::VectorBlock<TangentVector, DIMS> tangent_vector_rotation_part(
      TangentVector &alg);

  // Helper to split out the translation part of an SE3::TangentVector.
  static Eigen::VectorBlock<const TangentVector, DIMS>
  tangent_vector_translation_part(const TangentVector &alg);

  static Eigen::VectorBlock<TangentVector, DIMS>
  tangent_vector_translation_part(TangentVector &alg);

  // Helper to join rotational and translational parts of an SE3::TangentVector.
  static TangentVector tangent_vector_from_parts(
      const SO3::TangentVector &alg_rot,
      const Eigen::Vector3d &alg_trans);

 private:
  SO3 rotation_;
  Eigen::Vector3d translation_{Eigen::Vector3d::Zero()};
};

static_assert(
    LieGroupType<SE3>,
    "SE3 doesn't meet the requirements of a Lie Group.");

// Computes distance between two SE3 elements in the same (checked) "reference"
// frame, without explicitly composing (for performance reasons.) This is not
// provided for translation, to reduce risk of sign/direction errors. Null
// frames are permitted, provided both are null.
//
// i.e. given a_from_ref and b_from_ref, this computes:
//    a_from_b.translation().norm() = b_from_a.translation().norm()
//
// Note the ordering of arguments, and specific choice of "reference" frame does
// not matter to the output here.
double se3_distance(const SE3 &a_from_ref, const SE3 &b_from_ref);

// Computes distance between the inverse of two SE3 elements in the same
// (checked) "reference" frame without explicitly composing or inverting (for
// performance reasons.) This is not provided for translation, to reduce risk of
// sign/direction errors. Null frames are permitted, provided both are null.
//
// i.e. given ref_from_a and ref_from_b, this computes:
//    a_from_b.translation().norm() = b_from_a.translation().norm()
//
// Note the ordering of arguments, and specific choice of "reference" frame does
// not matter to the output here.
double se3_inverse_distance(const SE3 &ref_from_a, const SE3 &ref_from_b);

}  // namespace resim::transforms
