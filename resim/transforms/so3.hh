// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>

#include "resim/math/is_approx.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/framed_vector.hh"
#include "resim/transforms/liegroup.hh"
#include "resim/transforms/liegroup_concepts.hh"

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
  template <typename... Args>
  explicit SO3(const Eigen::AngleAxisd &angle_axis, Args... args);

  // Constructor
  // Create an SO3 from an explicit angle and axis, which describes a
  // rotation as a right-handed rotation of 'angle' about a specified unit
  // 'axis'.
  template <typename... Args>
  SO3(double angle, const Eigen::Vector3d &axis, Args... args);

  // Constructor
  // Create an SO3 from a quaternion.
  template <typename... Args>
  explicit SO3(const Eigen::Quaterniond &quaternion, Args... args);

  // Constructor
  // Create and SO3 from a 3x3 rotation matrix.
  template <typename... Args>
  explicit SO3(Eigen::Matrix3d rotation_matrix, Args... args);

  // Get an identity SO3
  template <typename... Args>
  static SO3 identity(Args &&...args);

  // Operator*
  // Compose this SO3 with another (multiplication)
  SO3 operator*(const SO3 &other) const;

  // Operator*
  // Apply the action SO3 rotation to a vector in 3-Dimensional space
  // (multiplication)
  Eigen::Vector3d operator*(const Eigen::Vector3d &source_vector) const;

  // Equivalent to operator*
  // Providing a more explicit interface, which is common with other Lie Groups
  // e.g. SE3.
  Eigen::Vector3d rotate(const Eigen::Vector3d &source_vector) const;

  // Apply the Group rotation to a FramedVector, this will change the frame
  // of the vector accordingly. This method will fail if the source_vector
  // frame does not match from_;
  FramedVector<SO3::DIMS> rotate(
      const FramedVector<SO3::DIMS> &source_vector) const;

  // Return the inverse of this SO3.
  SO3 inverse() const;

  // Interpolate this SO3
  // [param] fraction - interpolation is over a unit interval, where
  // fraction=0 returns identity and fraction=1 returns this SO3. In between
  // the SO3 returned is a linear interpolation. If fraction is greater than 1
  // or less than 0, a linear extrapolation will be returned. The into() and
  // from() frames are preserved.
  SO3 interp(double fraction) const;

  // Interpolate the SO3, returning a framed SO3 with a user-provided
  // from() frame. The into() frame is preserved.
  SO3 interp(double fraction, const Frame<SO3::DIMS> &new_from) const;

  // Create an SO3 from an element of the LieGroup algebra.
  template <typename... Args>
  static SO3 exp(const TangentVector &alg, Args &&...args);

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

  // Test for floating-point equality with another SO3.
  bool is_approx(const SO3 &other, double precision = math::DEFAULT_PRECISION)
      const;

  // Test for floating-point equality with another SO3, ignoring frames.
  bool is_approx_transform(
      const SO3 &other,
      double precision = math::DEFAULT_PRECISION) const;

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
