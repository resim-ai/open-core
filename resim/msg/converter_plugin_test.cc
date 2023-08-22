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

TEST(ConverterPluginTest, TestConverterPlugin) {
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
  const auto maybe_converted =
      plugin.try_convert("convertible_message_type", ros2_message);

  const auto maybe_not_converted =
      plugin.try_convert("unconvertible_message_type", ros2_message);

  EXPECT_THROW(
      plugin.try_convert("convertible_message_type", invalid_ros2_message),
      AssertException);

  // VERIFICATION
  ASSERT_TRUE(maybe_converted.has_value());
  EXPECT_FALSE(maybe_not_converted.has_value());

  const std::string expected_response =
      "I received: " + std::string(TEST_MESSAGE);
  ASSERT_EQ(expected_response.size(), maybe_converted->size());
  EXPECT_EQ(
      0,
      std::memcmp(
          expected_response.data(),
          maybe_converted->get_rcl_serialized_message().buffer,
          expected_response.size()));
}

TEST(ConverterPluginTest, TestConverterBadPlugin) {
  // SETUP / ACTION / VERIFICATION
  EXPECT_THROW(
      ConverterPlugin("resim/msg/testing/bad_test_converter_plugin.so"),
      AssertException);

  // COVERAGE
  // The below code tests the behavior of the badly named converter function.
  void *handle =
      dlopen("resim/msg/testing/bad_test_converter_plugin.so", RTLD_LAZY);
  using Converter = ReSimConverterPluginStatus (*)(
      const char *,
      const rcutils_uint8_array_t *,
      rcutils_uint8_array_t *);
  const auto bad_converter =
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      reinterpret_cast<const Converter>(
          dlsym(handle, "resim_badly_named_converter"));

  EXPECT_EQ(
      RESIM_CONVERTER_PLUGIN_STATUS_NO_MATCHING_CONVERTER,
      bad_converter(nullptr, nullptr, nullptr));

  dlclose(handle);
}

}  // namespace resim::msg
