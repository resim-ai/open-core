#pragma once

#include <Eigen/Dense>

namespace resim {
namespace transforms {

// Special Orthogonal Group in 3-Dimensional space.
//
// Implements rigid pure-rotation transforms in 3-Dimensional space.
// Example usages:
//     const SO3 global_from_robot(robot_pose_source);
//     // Apply the rotational transform to a vector by multiplication.
//     const Eigen::Vector3d point_on_robot;
//     const Eigen::Vector3d point_in_global_frame 
//           = global_from_robot * point_on_robot;
//     // Interpolate a rotation
//     const SO3 global_from_robot_halfway = global_from_robot.interp(0.5):
class SO3 {
  public:
    // Constructor 
    // Create an SO3 from and Eigen AngleAxisd object, which describes a
    // rotation as an right-handed rotation of 'angle' about a specified unit 
    // axis.
    explicit SO3(const Eigen::AngleAxisd &angle_axis);

    // Getter
    // Retrieve a reference to the 3x3 rotation matrix that represents the 
    // SO3 transform.
    const Eigen::Matrix3d &rotation_matrix() const;

    // TODO(simon) implement the rest of SO3 as laid out in task:
    // https://app.asana.com/0/1202178773526279/1202227479029247/f
  private:
    Eigen::Matrix3d rotation_matrix_;
};

} // namespace transforms
} // namespace resim
