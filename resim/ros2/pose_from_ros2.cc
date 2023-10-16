// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/ros2/pose_from_ros2.hh"

#include <cstdint>

#include "resim/assert/assert.hh"
#include "resim/transforms/proto/se3_to_proto.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::ros2 {

transforms::proto::SE3 convert_from_ros2(
    const geometry_msgs::msg::Pose &ros2_msg) {
  const transforms::SO3 rotation{Eigen::Quaterniond{
      ros2_msg.orientation.w,
      ros2_msg.orientation.x,
      ros2_msg.orientation.y,
      ros2_msg.orientation.z,
  }};

  const transforms::SE3 transform{
      rotation,
      Eigen::Vector3d{
          ros2_msg.position.x,
          ros2_msg.position.y,
          ros2_msg.position.z,
      }};

  transforms::proto::SE3 result;
  pack(transform, &result);
  return result;
}

geometry_msgs::msg::Pose convert_to_ros2(
    const transforms::proto::SE3 &resim_msg) {
  const transforms::SE3 pose{unpack(resim_msg)};
  geometry_msgs::msg::Pose result;

  result.position.x = pose.translation().x();
  result.position.y = pose.translation().y();
  result.position.z = pose.translation().z();

  const Eigen::Quaterniond rotation_quaternion = pose.rotation().quaternion();

  result.orientation.w = rotation_quaternion.w();
  result.orientation.x = rotation_quaternion.x();
  result.orientation.y = rotation_quaternion.y();
  result.orientation.z = rotation_quaternion.z();

  return result;
}

msg::PoseWithCovariance convert_from_ros2(
    const geometry_msgs::msg::PoseWithCovariance &ros2_msg) {
  msg::PoseWithCovariance result;

  result.mutable_pose()->CopyFrom(convert_from_ros2(ros2_msg.pose));

  constexpr std::size_t N = transforms::SE3::DOF;
  REASSERT(ros2_msg.covariance.size() == N * N, "Covariance has wrong size!");
  for (int ii = 0; ii < N * N; ++ii) {
    result.add_covariance(ros2_msg.covariance.at(ii));
  }
  return result;
}

geometry_msgs::msg::PoseWithCovariance convert_to_ros2(
    const msg::PoseWithCovariance &resim_msg) {
  geometry_msgs::msg::PoseWithCovariance result;
  result.pose = convert_to_ros2(resim_msg.pose());

  constexpr std::size_t N = transforms::SE3::DOF;
  REASSERT(resim_msg.covariance_size() == N * N, "Covariance has wrong size!");
  for (int ii = 0; ii < N * N; ++ii) {
    result.covariance.at(ii) = resim_msg.covariance(ii);
  }
  return result;
}

msg::Twist convert_from_ros2(const geometry_msgs::msg::Twist &ros2_msg) {
  msg::Twist result;

  result.add_algebra(ros2_msg.angular.x);
  result.add_algebra(ros2_msg.angular.y);
  result.add_algebra(ros2_msg.angular.z);

  result.add_algebra(ros2_msg.linear.x);
  result.add_algebra(ros2_msg.linear.y);
  result.add_algebra(ros2_msg.linear.z);

  return result;
}

geometry_msgs::msg::Twist convert_to_ros2(const msg::Twist &resim_msg) {
  geometry_msgs::msg::Twist result;
  constexpr std::size_t N = transforms::SE3::DOF;
  REASSERT(resim_msg.algebra_size() == N, "Algebra has wrong size!");

  // NOLINTBEGIN(readability-magic-numbers)
  result.angular.x = resim_msg.algebra(0);
  result.angular.y = resim_msg.algebra(1);
  result.angular.z = resim_msg.algebra(2);

  result.linear.x = resim_msg.algebra(3);
  result.linear.y = resim_msg.algebra(4);
  result.linear.z = resim_msg.algebra(5);
  // NOLINTEND(readability-magic-numbers)

  return result;
}

msg::TwistWithCovariance convert_from_ros2(
    const geometry_msgs::msg::TwistWithCovariance &ros2_msg) {
  msg::TwistWithCovariance result;

  result.mutable_twist()->CopyFrom(convert_from_ros2(ros2_msg.twist));

  constexpr std::size_t N = transforms::SE3::DOF;
  REASSERT(ros2_msg.covariance.size() == N * N, "Covariance has wrong size!");
  for (int ii = 0; ii < N * N; ++ii) {
    result.add_covariance(ros2_msg.covariance.at(ii));
  }
  return result;
}

geometry_msgs::msg::TwistWithCovariance convert_to_ros2(
    const msg::TwistWithCovariance &resim_msg) {
  geometry_msgs::msg::TwistWithCovariance result;
  result.twist = convert_to_ros2(resim_msg.twist());

  constexpr std::size_t N = transforms::SE3::DOF;
  REASSERT(resim_msg.covariance_size() == N * N, "Covariance has wrong size!");
  for (int ii = 0; ii < N * N; ++ii) {
    result.covariance.at(ii) = resim_msg.covariance(ii);
  }
  return result;
}

}  // namespace resim::ros2
