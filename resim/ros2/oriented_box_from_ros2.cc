// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/ros2/oriented_box_from_ros2.hh"

#include "resim/assert/assert.hh"
#include "resim/ros2/pose_from_ros2.hh"

namespace resim::ros2 {

// ROS2 converters for message headers

geometry::proto::OrientedBoxSE3 convert_from_ros2(
    const vision_msgs::msg::BoundingBox3D &ros2_msg) {
  geometry::proto::OrientedBoxSE3 result;
  result.mutable_reference_from_box()->CopyFrom(
      convert_from_ros2(ros2_msg.center));
  result.add_extents(ros2_msg.size.x);
  result.add_extents(ros2_msg.size.y);
  result.add_extents(ros2_msg.size.z);
  return result;
}

vision_msgs::msg::BoundingBox3D convert_to_ros2(
    const geometry::proto::OrientedBoxSE3 &resim_msg) {
  vision_msgs::msg::BoundingBox3D result;

  result.center = convert_to_ros2(resim_msg.reference_from_box());
  REASSERT(resim_msg.extents_size() == 3U);
  result.size.x = resim_msg.extents(0);
  result.size.y = resim_msg.extents(1);
  result.size.z = resim_msg.extents(2);
  return result;
}

}  // namespace resim::ros2
