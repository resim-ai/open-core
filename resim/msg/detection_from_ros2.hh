// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <vision_msgs/msg/detection3_d.hpp>

#include "resim/msg/detection.pb.h"

namespace resim::msg {

// ROS2 converters for detections

Detection3D convert_from_ros2(const vision_msgs::msg::Detection3D &ros2_msg);

vision_msgs::msg::Detection3D convert_to_ros2(const Detection3D &resim_msg);

}  // namespace resim::msg
