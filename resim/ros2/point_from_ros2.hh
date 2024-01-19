#pragma once

#include <geometry_msgs/msg/point.hpp>

#include "resim/transforms/proto/framed_vector_3.pb.h"

namespace resim::ros2 {

// ROS2 converter for points
resim::transforms::proto::FramedVector_3 convert_from_ros2(
    const geometry_msgs::msg::Point &ros2_msg);

geometry_msgs::msg::Point convert_to_ros2(
    const resim::transforms::proto::FramedVector_3 &resim_msg);

}  // namespace resim::ros2
