#pragma once

#include <Eigen/Dense>

namespace resim {
namespace transforms {

// Special Orthogonal Group in 3-Dimensional space.
//
// Practical implementation of the Liegroup representing rigid pure-rotations
// in 3-Dimensional space.
// 
// Based on the Liegroups library and documentation available at:
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
class SO3 {
  public:
    using TangentVector = Eigen::Vector3d;
    using TangentMapping = Eigen::Matrix3d;

    SO3() = default;

    // Constructor 
    // Create an SO3 from an Eigen AngleAxisd object, which describes a
    // rotation as an right-handed rotation of 'angle' about a specified unit 
    // axis.
    explicit SO3(const Eigen::AngleAxisd &angle_axis);
    
    // Constructor
    // Create an SO3 from a quaternion.
    explicit SO3(const Eigen::Quaterniond &quaternion);

    // Constructor
    // Create and SO3 from a 3x3 rotation matrix.
    explicit SO3(const Eigen::Matrix3d &rotation_matrix);

    // Get an identity SO3
    static SO3 identity();

    // Operator*
    // Compose this SO3 with another (multiplication)
    SO3 operator*(const SO3 &other) const;

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
    SO3 interp(const double fraction) const;

    // Create an SO3 from an element of the Liegroup algebra.
    static SO3 exp(const TangentVector &alg);
    
    // Retrieve the element of the Liegroup algebra that represents
    // this group element.
    TangentVector log() const;

    // Transform a TangentVector from the right tangent space to the left.
    TangentVector adjoint_times(const TangentVector &alg) const;
    
    // Test for floating-point equality with another SO3.
    bool is_approx(const SO3 &other) const;
    
    // Getter
    // Retrieve a reference to the 3x3 rotation matrix that represents the 
    // SO3 transform.
    const Eigen::Matrix3d &rotation_matrix() const;

  private:
    Eigen::Matrix3d rotation_matrix_;
};

} // namespace transforms
} // namespace resim
