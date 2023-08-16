// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <std_msgs/msg/header.hpp>

#include "resim/msg/header.pb.h"

namespace resim::msg {

// Convert a ROS2 timestamp into a resim::msg::proto::Header
Header convert_from_ros2(const std_msgs::msg::Header &ros2_msg);

std_msgs::msg::Header convert_to_ros2(const Header &header);

}  // namespace resim::msg
