// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/transform_from_ros2.hh"

#include <Eigen/Dense>

#include "resim/msg/header_from_ros2.hh"
#include "resim/transforms/proto/se3_to_proto.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::msg {

transforms::proto::SE3 convert_from_ros2(
    const geometry_msgs::msg::Transform &ros2_msg) {
  const transforms::SO3 rotation{Eigen::Quaterniond{
      ros2_msg.rotation.w,
      ros2_msg.rotation.x,
      ros2_msg.rotation.y,
      ros2_msg.rotation.z,
  }};

  const transforms::SE3 transform{
      rotation,
      Eigen::Vector3d{
          ros2_msg.translation.x,
          ros2_msg.translation.y,
          ros2_msg.translation.z,
      }};

  transforms::proto::SE3 result;
  pack(transform, &result);
  return result;
}

geometry_msgs::msg::Transform convert_to_ros2_transform(
    const transforms::proto::SE3 &resim_msg) {
  const transforms::SE3 pose{unpack(resim_msg)};
  geometry_msgs::msg::Transform result;

  result.translation.x = pose.translation().x();
  result.translation.y = pose.translation().y();
  result.translation.z = pose.translation().z();

  const Eigen::Quaterniond rotation_quaternion = pose.rotation().quaternion();

  result.rotation.w = rotation_quaternion.w();
  result.rotation.x = rotation_quaternion.x();
  result.rotation.y = rotation_quaternion.y();
  result.rotation.z = rotation_quaternion.z();

  return result;
}

TransformStamped convert_from_ros2(
    const geometry_msgs::msg::TransformStamped &ros2_msg) {
  TransformStamped result;
  result.mutable_header()->CopyFrom(convert_from_ros2(ros2_msg.header));
  result.set_child_frame_id(ros2_msg.child_frame_id);
  result.mutable_transform()->CopyFrom(convert_from_ros2(ros2_msg.transform));
  return result;
}

geometry_msgs::msg::TransformStamped convert_to_ros2(
    const TransformStamped &resim_msg) {
  geometry_msgs::msg::TransformStamped result;
  result.header = convert_to_ros2(resim_msg.header());
  result.child_frame_id = resim_msg.child_frame_id();
  result.transform = convert_to_ros2_transform(resim_msg.transform());
  return result;
}

TransformArray convert_from_ros2(const tf2_msgs::msg::TFMessage &ros2_msg) {
  TransformArray result;
  for (const auto &transform : ros2_msg.transforms) {
    result.add_transforms()->CopyFrom(convert_from_ros2(transform));
  }
  return result;
}

tf2_msgs::msg::TFMessage convert_to_ros2(const TransformArray &resim_msg) {
  tf2_msgs::msg::TFMessage result;
  result.transforms.reserve(resim_msg.transforms_size());
  for (const auto &transform : resim_msg.transforms()) {
    result.transforms.push_back(convert_to_ros2(transform));
  }
  return result;
}

}  // namespace resim::msg
