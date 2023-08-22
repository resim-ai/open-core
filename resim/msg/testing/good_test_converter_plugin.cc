// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cstdint>
#include <cstring>
#include <rclcpp/serialized_message.hpp>

#include "resim/assert/assert.hh"
#include "resim/msg/converter_plugin_status.h"

// This is a well-behaved test plugin for testing the ConverterPlugin class. See
// ConverterPluginTest for more details.
extern "C" ReSimConverterPluginStatus resim_convert_ros2_to_resim(
    const char *const ros2_message_type,
    const rcutils_uint8_array_t *const ros2_message,
    rcutils_uint8_array_t *const resim_message) {
  try {
    REASSERT(ros2_message_type != nullptr);
    REASSERT(ros2_message != nullptr);
    REASSERT(ros2_message->buffer != nullptr);

    REASSERT(resim_message != nullptr);

    rcl_serialized_message_t &result = *resim_message;
    if (std::strcmp(ros2_message_type, "convertible_message_type") == 0) {
      const std::string converted_message{
          "I received: " +
          std::string{
              // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
              reinterpret_cast<const char *>(ros2_message->buffer),
              ros2_message->buffer_length}};

      const std::size_t converted_size = converted_message.size();
      result.allocator = ros2_message->allocator;
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      result.buffer = reinterpret_cast<uint8_t *>(
          result.allocator.allocate(converted_size, result.allocator.state));
      result.buffer_length = converted_size;
      result.buffer_capacity = converted_size;
      std::memcpy(result.buffer, converted_message.data(), converted_size);
    } else {
      return RESIM_CONVERTER_PLUGIN_STATUS_NO_MATCHING_CONVERTER;
    }
  } catch (...) {
    return RESIM_CONVERTER_PLUGIN_STATUS_ERROR;
  }
  return RESIM_CONVERTER_PLUGIN_STATUS_OK;
}
