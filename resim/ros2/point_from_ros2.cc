#include "resim/ros2/point_from_ros2.hh"

namespace resim::ros2 {

// ROS2 converter for points
resim::transforms::proto::FramedVector_3 convert_from_ros2(
    const geometry_msgs::msg::Point &ros2_msg) {
  resim::transforms::proto::FramedVector_3 result;

  result.add_algebra(ros2_msg.x);
  result.add_algebra(ros2_msg.y);
  result.add_algebra(ros2_msg.z);

  return result;
}

geometry_msgs::msg::Point convert_to_ros2(
    const resim::transforms::proto::FramedVector_3 &resim_msg) {
  geometry_msgs::msg::Point result;

  result.x = resim_msg.algebra(0);
  result.y = resim_msg.algebra(1);
  result.z = resim_msg.algebra(2);

  return result;
}
}  // namespace resim::ros2
