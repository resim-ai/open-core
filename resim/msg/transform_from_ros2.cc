// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/transform_from_ros2.hh"

namespace resim::msg {

transforms::proto::SE3 convert_from_ros2(
    const geometry_msgs::msg::Transform &ros2_msg) {
  return transforms::proto::SE3();
}

geometry_msgs::msg::Transform convert_to_ros2(
    const transforms::proto::SE3 &resim_msg) {
  return geometry_msgs::msg::Transform();
}

TransformStamped convert_from_ros2(
    const geometry_msgs::msg::TransformStamped &ros2_msg) {
  return TransformStamped();
}

geometry_msgs::msg::TransformStamped convert_to_ros2(
    const TransformStamped &resim_msg) {
  return geometry_msgs::msg::TransformStamped();
}

TransformArray convert_from_ros2(const tf2_msgs::msg::TFMessage &ros2_msg) {
  return TransformArray();
}

tf2_msgs::msg::TFMessage convert_to_ros2(const TransformArray &resim_msg) {
  return tf2_msgs::msg::TFMessage();
}

}  // namespace resim::msg
