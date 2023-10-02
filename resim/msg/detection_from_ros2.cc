// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/detection_from_ros2.hh"

#include "resim/msg/header_from_ros2.hh"
#include "resim/msg/oriented_box_from_ros2.hh"

namespace resim::msg {

Detection3D convert_from_ros2(const vision_msgs::msg::Detection3D &ros2_msg) {
  Detection3D result;
  result.mutable_header()->CopyFrom(convert_from_ros2(ros2_msg.header));
  result.mutable_bbox()->CopyFrom(convert_from_ros2(ros2_msg.bbox));
  result.set_id(ros2_msg.id);
  return result;
}

vision_msgs::msg::Detection3D convert_to_ros2(const Detection3D &resim_msg) {
  vision_msgs::msg::Detection3D result;
  result.header = convert_to_ros2(resim_msg.header());
  result.bbox = convert_to_ros2(resim_msg.bbox());
  result.id = resim_msg.id();
  return result;
}

Detection3DArray convert_from_ros2(
    const vision_msgs::msg::Detection3DArray &ros2_msg) {
  Detection3DArray result;
  result.mutable_header()->CopyFrom(convert_from_ros2(ros2_msg.header));
  for (const auto &detection : ros2_msg.detections) {
    result.add_detections()->CopyFrom(convert_from_ros2(detection));
  }
  return result;
}

vision_msgs::msg::Detection3DArray convert_to_ros2(
    const Detection3DArray &resim_msg) {
  vision_msgs::msg::Detection3DArray result;
  result.header = convert_to_ros2(resim_msg.header());
  result.detections.reserve(resim_msg.detections_size());
  for (const auto &detection : resim_msg.detections()) {
    result.detections.push_back(convert_to_ros2(detection));
  }
  return result;
}

BoundingBox2D convert_from_ros2(
    const vision_msgs::msg::BoundingBox2D &ros2_msg) {
  BoundingBox2D result;
  result.set_center_x(ros2_msg.center.position.x);
  result.set_center_y(ros2_msg.center.position.y);
  result.set_theta_rad(ros2_msg.center.theta);
  result.set_size_x(ros2_msg.size_x);
  result.set_size_y(ros2_msg.size_y);
  return result;
}

vision_msgs::msg::BoundingBox2D convert_to_ros2(
    const BoundingBox2D &resim_msg) {
  vision_msgs::msg::BoundingBox2D result;
  result.center.position.x = resim_msg.center_x();
  result.center.position.y = resim_msg.center_y();
  result.center.theta = resim_msg.theta_rad();
  result.size_x = resim_msg.size_x();
  result.size_y = resim_msg.size_y();
  return result;
}

Detection2D convert_from_ros2(const vision_msgs::msg::Detection2D &ros2_msg) {
  Detection2D result;
  result.mutable_header()->CopyFrom(convert_from_ros2(ros2_msg.header));
  result.mutable_bbox()->CopyFrom(convert_from_ros2(ros2_msg.bbox));
  result.set_id(ros2_msg.id);
  return result;
}

vision_msgs::msg::Detection2D convert_to_ros2(const Detection2D &resim_msg) {
  vision_msgs::msg::Detection2D result;
  result.header = convert_to_ros2(resim_msg.header());
  result.bbox = convert_to_ros2(resim_msg.bbox());
  result.id = resim_msg.id();
  return result;
}

Detection2DArray convert_from_ros2(
    const vision_msgs::msg::Detection2DArray &ros2_msg) {
  Detection2DArray result;
  result.mutable_header()->CopyFrom(convert_from_ros2(ros2_msg.header));
  for (const auto &detection : ros2_msg.detections) {
    result.add_detections()->CopyFrom(convert_from_ros2(detection));
  }
  return result;
}

vision_msgs::msg::Detection2DArray convert_to_ros2(
    const Detection2DArray &resim_msg) {
  vision_msgs::msg::Detection2DArray result;
  result.header = convert_to_ros2(resim_msg.header());
  result.detections.reserve(resim_msg.detections_size());
  for (const auto &detection : resim_msg.detections()) {
    result.detections.push_back(convert_to_ros2(detection));
  }
  return result;
}

}  // namespace resim::msg
