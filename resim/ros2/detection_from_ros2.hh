// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "resim/msg/detection.pb.h"

namespace resim::ros2 {

// ROS2 converters for detections

ObjectHypothesis convert_from_ros2(
    const vision_msgs::msg::ObjectHypothesis &ros2_msg);

vision_msgs::msg::ObjectHypothesis convert_to_ros2(
    const ObjectHypothesis &resim_msg);

ObjectHypothesisWithPose convert_from_ros2(
    const vision_msgs::msg::ObjectHypothesisWithPose &ros2_msg);

vision_msgs::msg::ObjectHypothesisWithPose convert_to_ros2(
    const ObjectHypothesisWithPose &resim_msg);

Detection3D convert_from_ros2(const vision_msgs::msg::Detection3D &ros2_msg);

vision_msgs::msg::Detection3D convert_to_ros2(const Detection3D &resim_msg);

Detection3DArray convert_from_ros2(
    const vision_msgs::msg::Detection3DArray &ros2_msg);

vision_msgs::msg::Detection3DArray convert_to_ros2(
    const Detection3DArray &resim_msg);

BoundingBox2D convert_from_ros2(
    const vision_msgs::msg::BoundingBox2D &ros2_msg);

vision_msgs::msg::BoundingBox2D convert_to_ros2(const BoundingBox2D &resim_msg);

Detection2D convert_from_ros2(const vision_msgs::msg::Detection2D &ros2_msg);

vision_msgs::msg::Detection2D convert_to_ros2(const Detection2D &resim_msg);

Detection2DArray convert_from_ros2(
    const vision_msgs::msg::Detection2DArray &ros2_msg);

vision_msgs::msg::Detection2DArray convert_to_ros2(
    const Detection2DArray &resim_msg);

}  // namespace resim::ros2
