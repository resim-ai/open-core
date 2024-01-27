// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <std_msgs/msg/bool.hpp>

#include "resim/msg/primitives.pb.h"

namespace resim::ros2 {

// ROS2 converters for odometry

msg::Bool convert_from_ros2(const std_msgs::msg::Bool &ros2_msg);

std_msgs::msg::Bool convert_to_ros2(const msg::Bool &resim_msg);

}  // namespace resim::ros2
