// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cstring>
#include <random>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <string>

#include "resim/assert/assert.hh"
#include "resim/geometry/proto/fuzz_helpers.hh"
#include "resim/msg/fuzz_helpers.hh"
#include "resim/ros2/converter_plugin.hh"
#include "resim/ros2/detection_from_ros2.hh"
#include "resim/ros2/header_from_ros2.hh"
#include "resim/ros2/navsat_from_ros2.hh"
#include "resim/ros2/odometry_from_ros2.hh"
#include "resim/ros2/oriented_box_from_ros2.hh"
#include "resim/ros2/pose_from_ros2.hh"
#include "resim/ros2/time_from_ros2.hh"
#include "resim/ros2/transform_from_ros2.hh"
#include "resim/transforms/proto/fuzz_helpers.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/proto/dependency_file_descriptor_set.hh"

namespace resim::ros2 {

namespace {
const ConverterPlugin plugin{"resim/ros2/default_converter_plugin.so"};
}

using Ros2Types = ::testing::Types<
    builtin_interfaces::msg::Time,
    geometry_msgs::msg::Pose,
    geometry_msgs::msg::PoseWithCovariance,
    geometry_msgs::msg::Transform,
    geometry_msgs::msg::TransformStamped,
    geometry_msgs::msg::Twist,
    geometry_msgs::msg::TwistWithCovariance,
    nav_msgs::msg::Odometry,
    sensor_msgs::msg::NavSatFix,
    std_msgs::msg::Header,
    tf2_msgs::msg::TFMessage,
    vision_msgs::msg::BoundingBox3D,
    vision_msgs::msg::Detection3D,
    vision_msgs::msg::Detection3DArray,
    vision_msgs::msg::BoundingBox2D,
    vision_msgs::msg::Detection2D,
    vision_msgs::msg::Detection2DArray>;

template <typename T>
struct DefaultConverterPluginTest : public ::testing::Test {
  static void SetUpTestSuite() {
    // Make sure we allow Protobuf to de-alloc its memory
    static bool registered = false;
    if (not registered) {
      std::atexit(google::protobuf::ShutdownProtobufLibrary);
      registered = true;
    }
  }
};

TYPED_TEST_SUITE(DefaultConverterPluginTest, Ros2Types);

template <typename T>
std::string resim_type_name() {
  return T::GetDescriptor()->full_name();
}

TYPED_TEST(DefaultConverterPluginTest, TestSupports) {
  // SETUP

  const char *ros2_type_name = rosidl_generator_traits::name<TypeParam>();

  // ACTION / VERIFICATION
  EXPECT_TRUE(plugin.supports_type(ros2_type_name));
}

TYPED_TEST(DefaultConverterPluginTest, TestSchemas) {
  // SETUP
  using Ros2Type = TypeParam;
  using ReSimType = decltype(convert_from_ros2(std::declval<Ros2Type>()));
  const char *ros2_type_name = rosidl_generator_traits::name<Ros2Type>();

  // ACTION
  auto schema_info = plugin.get_schema(ros2_type_name);

  // VERIFICATION
  EXPECT_EQ(schema_info.name, resim_type_name<ReSimType>());
  EXPECT_EQ(schema_info.encoding, "protobuf");

  const std::string expected_data{
      dependency_file_descriptor_set(*ReSimType::GetDescriptor())};
  ASSERT_EQ(schema_info.data.size(), expected_data.size());
  EXPECT_EQ(
      0,
      std::memcmp(
          schema_info.data.data(),
          expected_data.data(),
          schema_info.data.size()));
}

// These functions and overloads are needed to customize the behavior for
// getting random test elements and verifying equality for particular types.
namespace {

// By default, we use random_element() to get test elements.
template <typename ReSimType>
ReSimType get_test_element(InOut<std::mt19937> rng) {
  return random_element<ReSimType>(rng);
}

// For OrientedBox, we don't want the frame IDs
template <>
geometry::proto::OrientedBoxSE3
get_test_element<geometry::proto::OrientedBoxSE3>(InOut<std::mt19937> rng) {
  auto random_box = random_element<geometry::proto::OrientedBoxSE3>(rng);
  // We don't use frame IDs when converting to/from ROS2.
  constexpr int DIMS = 3;
  random_box.mutable_reference_from_box()
      ->mutable_into()
      ->mutable_id()
      ->set_data(transforms::Frame<DIMS>::null_frame().id().to_string());
  random_box.mutable_reference_from_box()
      ->mutable_from()
      ->mutable_id()
      ->set_data(transforms::Frame<DIMS>::null_frame().id().to_string());
  return random_box;
}

// For SE3, we don't want the frame IDs
template <>
transforms::proto::SE3 get_test_element<transforms::proto::SE3>(
    InOut<std::mt19937> rng) {
  auto pose = random_element<transforms::proto::SE3>(rng);
  // We don't use frame IDs when converting to/from ROS2.
  constexpr int DIMS = 3;
  pose.mutable_into()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());
  pose.mutable_from()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());
  return pose;
}

// By default, we use convert_to_ros to convert resim types to ros.
template <typename Ros2Type, typename ReSimType>
Ros2Type convert(const ReSimType &x) {
  return convert_to_ros2(x);
}

// For transforms, we have a special function since there's already an overload
// of convert_to_ros2() for SE3s which returns a geometry_msgs::msgP::Pose
// instead.
// TODO(mikebauer) Come up with a general strategy for dealing with multiple
// ROS2 types corresponding to SE3
template <>
geometry_msgs::msg::Transform convert<geometry_msgs::msg::Transform>(
    const transforms::proto::SE3 &x) {
  return convert_to_ros2_transform(x);
}

// Bring this into this namespace since it will get shadowed by the other
// verify_equality overloads that are in this namespace.
bool verify_equality(
    const google::protobuf::Timestamp &a,
    const google::protobuf::Timestamp &b) {
  return resim::verify_equality(a, b);
}

}  // namespace

TYPED_TEST(DefaultConverterPluginTest, TestConvert) {
  // SETUP
  using Ros2Type = TypeParam;
  using ReSimType = decltype(convert_from_ros2(std::declval<Ros2Type>()));
  const char *ros2_type_name = rosidl_generator_traits::name<Ros2Type>();

  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};
  const auto test_message = get_test_element<ReSimType>(InOut{rng});
  const auto ros2_message = convert<Ros2Type>(test_message);

  rclcpp::Serialization<Ros2Type> serialization;
  rclcpp::SerializedMessage serialized_message;
  serialization.serialize_message(&ros2_message, &serialized_message);

  // ACTION
  auto converted = plugin.convert(ros2_type_name, serialized_message);

  // VERIFICATION
  ReSimType deserialized;
  ASSERT_TRUE(deserialized.ParseFromArray(converted.data(), converted.size()));
  EXPECT_TRUE(verify_equality(test_message, deserialized));
}

TYPED_TEST(DefaultConverterPluginTest, TestConvertBadMessage) {
  // SETUP
  using Ros2Type = TypeParam;
  const char *ros2_type_name = rosidl_generator_traits::name<Ros2Type>();

  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};

  rclcpp::SerializedMessage serialized_message;

  // ACTION / VERIFICATION
  EXPECT_THROW(
      plugin.convert(ros2_type_name, serialized_message),
      AssertException);
}

}  // namespace resim::ros2
