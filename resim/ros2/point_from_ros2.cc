#include "resim/ros2/point_from_ros2.hh"

#include "Eigen/Dense"
#include "resim/transforms/frame.hh"
#include "resim/transforms/proto/framed_vector_3_to_proto.hh"

namespace resim::ros2 {

// ROS2 converter for points
transforms::proto::FramedVector_3 convert_from_ros2(
    const geometry_msgs::msg::Point &ros2_msg) {
  transforms::proto::FramedVector_3 result;
  constexpr int DIMS = 3;

  const transforms::FramedVector<DIMS> vec(
      Eigen::Vector3d{ros2_msg.x, ros2_msg.y, ros2_msg.z},
      transforms::Frame<DIMS>::null_frame());

  transforms::proto::pack(vec, &result);
  return result;
}

geometry_msgs::msg::Point convert_to_ros2(
    const transforms::proto::FramedVector_3 &resim_msg) {
  geometry_msgs::msg::Point result;

  result.x = resim_msg.algebra(0);
  result.y = resim_msg.algebra(1);
  result.z = resim_msg.algebra(2);

  return result;
}
}  // namespace resim::ros2
