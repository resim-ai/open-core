#pragma once

#include <Eigen/Dense>

#include "transforms/so3.hh"

namespace resim {
namespace transforms {

// Special Euclidean Group in 3-Dimensional space.
//
// Practical implementation of the Liegroup representing rigid
// six-degree-of-freedom transformations in 3-Dimensional space.
//
// Based on the Liegroups library and documentation available at:
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
class SE3 {
 public:
  using TangentVector = Eigen::Matrix<double, 6, 1>;
  using TangentMatrix = Eigen::Matrix<double, 6, 6>;

  SE3() = default;

  // Constructor
  // Create an SE3 with rotation only from an SO3.
  explicit SE3(const SO3 &rotation);

  // Constructor
  // Create an SE3 with translation only from a 3-Vector.
  explicit SE3(const Eigen::Vector3d &translation);

  // Constructor
  // Create an SE3 with both a rotation and translation component.
  SE3(const SO3 &rotation, const Eigen::Vector3d &translation);

  // Getter
  // Return the rotational part of the transform.
  const SO3 &rotation() const;

  // Getter
  // Return the translational part of the transform.
  const Eigen::Vector3d &translation() const;

  // TODO(simon) implement the rest of SE3 as laid out in task:
  // https://app.asana.com/0/1202178773526279/1202227479029247/f

 private:
  SO3 rotation_;
  Eigen::Vector3d translation_;
};

}  // namespace transforms
}  // namespace resim
