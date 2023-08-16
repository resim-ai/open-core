// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <google/protobuf/timestamp.pb.h>

#include <builtin_interfaces/msg/time.hpp>

namespace resim::msg {

// Convert a ROS2 timestamp into a google::protobuf::Timestamp
google::protobuf::Timestamp convert_from_ros2(
    const builtin_interfaces::msg::Time &ros2_msg);

builtin_interfaces::msg::Time convert_to_ros2(
    const google::protobuf::Timestamp &time);

}  // namespace resim::msg
