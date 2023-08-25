// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/detection_from_ros2.hh"

#include "resim/msg/header_from_ros2.hh"
#include "resim/msg/oriented_box_from_ros2.hh"

namespace resim::msg {

// ROS2 converters for bounding boxes

Detection3D convert_from_ros2(const vision_msgs::msg::Detection3D &ros2_msg) {
  Detection3D result;
  result.mutable_header()->CopyFrom(convert_from_ros2(ros2_msg.header));
  result.mutable_bbox()->CopyFrom(convert_from_ros2(ros2_msg.bbox));
  return result;
}

vision_msgs::msg::Detection3D convert_to_ros2(const Detection3D &resim_msg) {
  vision_msgs::msg::Detection3D result;
  result.header = convert_to_ros2(resim_msg.header());
  result.bbox = convert_to_ros2(resim_msg.bbox());
  return result;
}

}  // namespace resim::msg
