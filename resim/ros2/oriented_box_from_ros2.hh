// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <vision_msgs/msg/bounding_box3_d.hpp>

#include "resim/geometry/proto/oriented_box.pb.h"

namespace resim::ros2 {

// ROS2 converters for bounding boxes

geometry::proto::OrientedBoxSE3 convert_from_ros2(
    const vision_msgs::msg::BoundingBox3D &ros2_msg);

vision_msgs::msg::BoundingBox3D convert_to_ros2(
    const geometry::proto::OrientedBoxSE3 &resim_msg);

}  // namespace resim::ros2
