// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/converter_plugin.hh"

#include <dlfcn.h>
#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <string_view>

#include "resim/assert/assert.hh"

namespace resim::msg {

TEST(ConverterPluginTest, TestConstructor) {
  // ACTION / VERIFICATION
  EXPECT_NO_THROW(
      ConverterPlugin{"resim/msg/testing/good_test_converter_plugin.so"});
  EXPECT_THROW(ConverterPlugin{"no_such_plugin.so"}, AssertException);
}

TEST(ConverterPluginTest, TestDestructor) {
  // Make sure we don't free a null handle;
  ConverterPlugin plugin{"resim/msg/testing/good_test_converter_plugin.so"};

  dlclose(plugin.handle_);
  plugin.handle_ = nullptr;
}

TEST(ConverterPluginTest, TestSupportsType) {
  // SETUP
  const ConverterPlugin plugin{
      "resim/msg/testing/good_test_converter_plugin.so"};

  // ACTION / VERIFICATION
  EXPECT_TRUE(plugin.supports_type("convertible_message_type"));
  EXPECT_FALSE(plugin.supports_type("inconvertible_message_type"));
}

TEST(ConverterPluginTest, TestGetSchema) {
  // SETUP
  const ConverterPlugin plugin{
      "resim/msg/testing/good_test_converter_plugin.so"};

  // ACITON / VERIFICATION
  const auto schema_info = plugin.get_schema("convertible_message_type");
  EXPECT_THROW(
      plugin.get_schema("inconvertible_message_type"),
      AssertException);

  constexpr std::string_view EXPECTED_SCHEMA =
      "The schema for converted_message_type";

  EXPECT_EQ(schema_info.name, "converted_message_type");
  EXPECT_EQ(schema_info.encoding, "protobuf");
  ASSERT_EQ(EXPECTED_SCHEMA.size(), schema_info.data.size());
  EXPECT_EQ(
      0,
      std::memcmp(
          schema_info.data.data(),
          EXPECTED_SCHEMA.data(),
          EXPECTED_SCHEMA.size()));
}

TEST(ConverterPluginTest, TestConvert) {
  // SETUP
  const ConverterPlugin plugin{
      "resim/msg/testing/good_test_converter_plugin.so"};

  constexpr std::string_view TEST_MESSAGE = "My message content!";
  rclcpp::SerializedMessage ros2_message{TEST_MESSAGE.size()};
  ros2_message.get_rcl_serialized_message().buffer_length = TEST_MESSAGE.size();
  std::memcpy(
      ros2_message.get_rcl_serialized_message().buffer,
      TEST_MESSAGE.data(),
      TEST_MESSAGE.size());

  rclcpp::SerializedMessage invalid_ros2_message;
  invalid_ros2_message.get_rcl_serialized_message().buffer = nullptr;

  // ACTION
  const auto converted =
      plugin.convert("convertible_message_type", ros2_message);

  EXPECT_THROW(
      plugin.convert("unconvertible_message_type", ros2_message),
      AssertException);

  EXPECT_THROW(
      plugin.convert("convertible_message_type", invalid_ros2_message),
      AssertException);

  // VERIFICATION
  const std::string expected_response =
      "I received: " + std::string(TEST_MESSAGE);
  ASSERT_EQ(expected_response.size(), converted.size());
  EXPECT_EQ(
      0,
      std::memcmp(
          expected_response.data(),
          converted.data(),
          expected_response.size()));
}

}  // namespace resim::msg
