#pragma once

#include <Eigen/Dense>

#include "resim_core/transforms/liegroup.hh"
#include "resim_core/transforms/liegroup_concepts.hh"

namespace resim::transforms {

// Special Orthogonal Group in 3-Dimensional space.
//
// Practical implementation of the LieGroup representing rigid pure-rotations
// in 3-Dimensional space.
//
// Based on the LieGroups library and documentation available at:
// https://ethaneade.com/
//
// Example usages:
//     const SO3 global_from_robot(robot_pose_source);
//     // Apply the rotational transform to a vector by multiplication.
//     const Eigen::Vector3d point_on_robot;
//     const Eigen::Vector3d point_in_global_frame
//           = global_from_robot * point_on_robot;
//     // Compose two SO3s by multiplication.
//     const SO3 robot_from_sensor(sensor_pose_source);
//     const SO3 global_from_sensor = global_from_robot * robot_from_sensor;
//     // Interpolate a rotation
//     const SO3 global_from_robot_halfway = global_from_robot.interp(0.5):
class SO3 : public LieGroup<3, 3> {
 public:
  SO3() = default;

  // Constructor
  // Create an SO3 from an Eigen AngleAxisd object, which describes a
  // rotation as a right-handed rotation of 'angle' about a specified unit
  // axis.
  explicit SO3(const Eigen::AngleAxisd &angle_axis);

  // Constructor
  // Create an SO3 from an explicit angle and axis, which describes a
  // rotation as a right-handed rotation of 'angle' about a specified unit
  // 'axis'.
  SO3(double angle, const Eigen::Vector3d &axis);

  // Constructor
  // Create an SO3 from a quaternion.
  explicit SO3(const Eigen::Quaterniond &quaternion);

  // Constructor
  // Create and SO3 from a 3x3 rotation matrix.
  explicit SO3(Eigen::Matrix3d rotation_matrix);

  // Get an identity SO3
  static SO3 identity();

  // Operator*
  // Compose this SO3 with another (multiplication)
  virtual SO3 operator*(const SO3 &other) const;

  // Operator*
  // Apply the action SO3 rotation to a vector in 3-Dimensional space
  // (multiplication)
  Eigen::Vector3d operator*(const Eigen::Vector3d &source_vector) const;

  // Return the inverse of this SO3.
  SO3 inverse() const;

  // Interpolate this SO3
  // [param] fraction - interpolation is over a unit interval, where
  // fraction=0 returns identity and fraction=1 returns this SO3. In between
  // the SO3 returned is a linear interpolation. If fraction is greater than 1
  // or less than 0, a linear extrapolation will be returned.
  SO3 interp(double fraction) const;

  // Create an SO3 from an element of the LieGroup algebra.
  static SO3 exp(const TangentVector &alg);

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

  // Test for floating-point equality with another SO3.
  bool is_approx(const SO3 &other) const;

  // Getter
  // Retrieve a reference to the 3x3 rotation matrix that represents the
  // SO3 transform.
  const Eigen::Matrix3d &rotation_matrix() const;

  // Get a quaternion representing this SO3
  Eigen::Quaterniond quaternion() const;

 private:
  Eigen::Matrix3d rotation_matrix_{Eigen::Matrix3d::Identity()};
};

static_assert(
    LieGroupType<SO3>,
    "SO3 doesn't meet the requirements of a Lie Group.");

}  // namespace resim::transforms
