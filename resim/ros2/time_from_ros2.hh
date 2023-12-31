// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <google/protobuf/timestamp.pb.h>

#include <builtin_interfaces/msg/time.hpp>

namespace resim::ros2 {

// ROS2 converters for timestamps

google::protobuf::Timestamp convert_from_ros2(
    const builtin_interfaces::msg::Time &ros2_msg);

builtin_interfaces::msg::Time convert_to_ros2(
    const google::protobuf::Timestamp &time);

}  // namespace resim::ros2
