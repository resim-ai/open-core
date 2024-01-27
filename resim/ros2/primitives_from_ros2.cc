// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/ros2/primitives_from_ros2.hh"

namespace resim::ros2 {

// ROS2 converters for odometry

msg::Bool convert_from_ros2(const std_msgs::msg::Bool &ros2_msg) {
  msg::Bool result;
  result.set_data(ros2_msg.data);
  return result;
}

std_msgs::msg::Bool convert_to_ros2(const msg::Bool &resim_msg) {
  std_msgs::msg::Bool result;
  result.data = resim_msg.data();
  return result;
}

}  // namespace resim::ros2
