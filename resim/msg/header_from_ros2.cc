// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/header_from_ros2.hh"

#include "resim/msg/time_from_ros2.hh"

namespace resim::msg {

Header convert_from_ros2(const std_msgs::msg::Header &ros2_msg) {
  Header header;
  header.mutable_stamp()->CopyFrom(convert_from_ros2(ros2_msg.stamp));
  header.set_frame_id(ros2_msg.frame_id);
  return header;
}

std_msgs::msg::Header convert_to_ros2(const Header &header) {
  std_msgs::msg::Header result;
  result.stamp = convert_to_ros2(header.stamp());
  result.frame_id = header.frame_id();
  return result;
}

}  // namespace resim::msg
