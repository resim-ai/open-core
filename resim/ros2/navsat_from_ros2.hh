// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "resim/msg/navsat.pb.h"

namespace resim::ros2 {

// ROS2 converters for nav_sat_fixes

NavSatFix convert_from_ros2(const sensor_msgs::msg::NavSatFix &ros2_msg);

sensor_msgs::msg::NavSatFix convert_to_ros2(const NavSatFix &resim_msg);

}  // namespace resim::ros2
