#pragma once

#include <Eigen/Dense>

#include "transforms/liegroup.hh"
#include "transforms/so3.hh"

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
class SE3 final : public LieGroup<SE3, 3, 6> {
 public:
  SE3() = default;

  // Constructor
  // Create an SE3 with rotation only from an SO3.
  explicit SE3(SO3 rotation);

  // Constructor
  // Create an SE3 with translation only from a 3-Vector.
  explicit SE3(Eigen::Vector3d translation);

  // Constructor
  // Create an SE3 with both a rotation and translation component.
  SE3(SO3 rotation, Eigen::Vector3d translation);

  // Get an identity SE3
  static SE3 identity();

  // Operator*
  // Compose this SE3 with another (multiplication)
  SE3 operator*(const SE3 &other) const override;

  // Operator*
  // Apply the SE3 action to a vector in 3-Dimensional space
  // (multiplication)
  Eigen::Vector3d operator*(
      const Eigen::Vector3d &source_vector) const override;

  // Return the inverse of this SE3.
  SE3 inverse() const override;

  // Return the length of the geodesic curve between frames in the
  // transformation.
  double arc_length() const;

  // Interpolate this SE3
  // [param] fraction - interpolation is over a unit interval, where
  // fraction=0 returns identity and fraction=1 returns this SE3. In between
  // the SE3 returned is a linear interpolation. If fraction is greater than 1
  // or less than 0, a linear extrapolation will be returned.
  SE3 interp(double fraction) const override;

  // Create an SE3 from an element of the LieGroup algebra.
  static SE3 exp(const TangentVector &alg);

  // Retrieve the element of the LieGroup algebra that represents
  // this group element.
  TangentVector log() const override;

  // Adjoint representation of this group element.
  TangentMapping adjoint() const override;

  // Adjoint representation of a given algebra element.
  static TangentMapping adjoint(const TangentVector &alg);

  // Transform a TangentVector from the right tangent space to the left.
  TangentVector adjoint_times(const TangentVector &alg) const override;

  // Adjoint times for algebra elements.
  static TangentVector adjoint_times(
      const TangentVector &alg_0,
      const TangentVector &alg_1);

  // Test for floating-point equality with another SE3.
  bool is_approx(const SE3 &other) const override;

  // Getter
  // Return the rotational part of the transform.
  const SO3 &rotation() const;

  // Getter
  // Return the translational part of the transform.
  const Eigen::Vector3d &translation() const;

  // Some TangentVector helpers:

  // Helper to split out the rotation part of an SE3::TangentVector.
  static SO3::TangentVector tangent_vector_rotation_part(
      const TangentVector &alg);

  // Helper to split out the translation part of an SE3::TangentVector.
  static Eigen::Vector3d tangent_vector_translation_part(
      const TangentVector &alg);

  // Helper to join rotational and translational parts of an SE3::TangentVector.
  static TangentVector tangent_vector_from_parts(
      const SO3::TangentVector &alg_rot,
      const Eigen::Vector3d &alg_trans);

 private:
  SO3 rotation_;
  Eigen::Vector3d translation_;
};

}  // namespace resim::transforms
