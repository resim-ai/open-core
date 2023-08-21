// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <geometry_msgs/msg/transform.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "resim/msg/transform.pb.h"
#include "resim/transforms/proto/se3.pb.h"

namespace resim::msg {

// ROS2 converters for transforms

transforms::proto::SE3 convert_from_ros2(
    const geometry_msgs::msg::Transform &ros2_msg);

geometry_msgs::msg::Transform convert_to_ros2(
    const transforms::proto::SE3 &resim_msg);

TransformStamped convert_from_ros2(
    const geometry_msgs::msg::TransformStamped &ros2_msg);

geometry_msgs::msg::TransformStamped convert_to_ros2(
    const TransformStamped &resim_msg);

TransformArray convert_from_ros2(const tf2_msgs::msg::TFMessage &ros2_msg);

tf2_msgs::msg::TFMessage convert_to_ros2(const TransformArray &resim_msg);

}  // namespace resim::msg
