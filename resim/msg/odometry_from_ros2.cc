// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/odometry_from_ros2.hh"

#include "resim/msg/header_from_ros2.hh"
#include "resim/msg/pose_from_ros2.hh"

namespace resim::msg {

// ROS2 converters for odometry

Odometry convert_from_ros2(const nav_msgs::msg::Odometry &ros2_msg) {
  Odometry result;

  result.mutable_header()->CopyFrom(convert_from_ros2(ros2_msg.header));
  result.set_child_frame_id(ros2_msg.child_frame_id);
  result.mutable_pose()->CopyFrom(convert_from_ros2(ros2_msg.pose));
  result.mutable_twist()->CopyFrom(convert_from_ros2(ros2_msg.twist));

  return result;
}

nav_msgs::msg::Odometry convert_to_ros2(const Odometry &resim_msg) {
  nav_msgs::msg::Odometry result;
  result.header = convert_to_ros2(resim_msg.header());
  result.child_frame_id = resim_msg.child_frame_id();
  result.pose = convert_to_ros2(resim_msg.pose());
  result.twist = convert_to_ros2(resim_msg.twist());
  return result;
}

}  // namespace resim::msg
