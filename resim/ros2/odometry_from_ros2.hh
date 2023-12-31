// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <nav_msgs/msg/odometry.hpp>

#include "resim/msg/odometry.pb.h"

namespace resim::ros2 {

// ROS2 converters for odometry

msg::Odometry convert_from_ros2(const nav_msgs::msg::Odometry &ros2_msg);

nav_msgs::msg::Odometry convert_to_ros2(const msg::Odometry &resim_msg);

}  // namespace resim::ros2
