// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include "resim/msg/pose.pb.h"
#include "resim/transforms/proto/se3.pb.h"

namespace resim::ros2 {

// ROS2 converters for poses and twists

transforms::proto::SE3 convert_from_ros2(
    const geometry_msgs::msg::Pose &ros2_msg);

geometry_msgs::msg::Pose convert_to_ros2(
    const transforms::proto::SE3 &resim_msg);

msg::PoseWithCovariance convert_from_ros2(
    const geometry_msgs::msg::PoseWithCovariance &ros2_msg);

geometry_msgs::msg::PoseWithCovariance convert_to_ros2(
    const msg::PoseWithCovariance &resim_msg);

msg::Twist convert_from_ros2(const geometry_msgs::msg::Twist &ros2_msg);

geometry_msgs::msg::Twist convert_to_ros2(const msg::Twist &resim_msg);

msg::TwistWithCovariance convert_from_ros2(
    const geometry_msgs::msg::TwistWithCovariance &ros2_msg);

geometry_msgs::msg::TwistWithCovariance convert_to_ros2(
    const msg::TwistWithCovariance &resim_msg);

}  // namespace resim::ros2
