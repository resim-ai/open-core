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

#include "resim/msg/converter_plugin.hh"
#include "resim/msg/fuzz_helpers.hh"
#include "resim/msg/odometry.pb.h"
#include "resim/msg/odometry_from_ros2.hh"
#include "resim/msg/transform.pb.h"
#include "resim/msg/transform_from_ros2.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/proto/dependency_file_descriptor_set.hh"

namespace resim::msg {

using ReSimTypes = ::testing::Types<TransformArray, Odometry>;

template <typename T>
struct DefaultConverterPluginTest : public ::testing::Test {
  static void SetUpTestSuite() {
    static bool registered = false;
    if (not registered) {
      std::atexit(google::protobuf::ShutdownProtobufLibrary);
      registered = true;
    }
  }
};

TYPED_TEST_SUITE(DefaultConverterPluginTest, ReSimTypes);

template <typename T>
std::string resim_type_name() {
  return T::GetDescriptor()->full_name();
}

TYPED_TEST(DefaultConverterPluginTest, TestSupports) {
  // SETUP
  ConverterPlugin plugin{"resim/msg/default_converter_plugin.so"};

  using Ros2Type = decltype(convert_to_ros2(std::declval<TypeParam>()));
  const char *ros2_type_name = rosidl_generator_traits::name<Ros2Type>();

  // ACTION / VERIFICATION
  EXPECT_TRUE(plugin.supports_type(ros2_type_name));
}

TYPED_TEST(DefaultConverterPluginTest, TestSchemas) {
  // SETUP
  ConverterPlugin plugin{"resim/msg/default_converter_plugin.so"};

  using ReSimType = TypeParam;
  using Ros2Type = decltype(convert_to_ros2(std::declval<TypeParam>()));
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

TYPED_TEST(DefaultConverterPluginTest, TestConvert) {
  // SETUP
  ConverterPlugin plugin{"resim/msg/default_converter_plugin.so"};

  using ReSimType = TypeParam;
  using Ros2Type = decltype(convert_to_ros2(std::declval<TypeParam>()));
  const char *ros2_type_name = rosidl_generator_traits::name<Ros2Type>();

  constexpr std::size_t SEED = 913U;
  std::mt19937 rng{SEED};
  const auto test_message = random_element<ReSimType>(InOut{rng});
  const Ros2Type ros2_message = convert_to_ros2(test_message);

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

}  // namespace resim::msg
