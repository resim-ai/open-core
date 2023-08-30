// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cstdint>
#include <cstring>
#include <functional>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <string_view>
#include <unordered_map>

#include "resim/assert/assert.hh"
#include "resim/msg/converter_plugin_types.h"
#include "resim/msg/header_from_ros2.hh"
#include "resim/msg/odometry_from_ros2.hh"
#include "resim/msg/pose_from_ros2.hh"
#include "resim/msg/time_from_ros2.hh"
#include "resim/msg/transform_from_ros2.hh"
#include "resim/utils/proto/dependency_file_descriptor_set.hh"

namespace resim::msg {

struct ConverterFunctions {
  using Converter = std::function<
      void(const rcutils_uint8_array_t *, rcutils_uint8_array_t *)>;
  using SchemaGetter = std::function<void(ReSimConverterSchemaInfo *)>;

  Converter converter;
  SchemaGetter schema_getter;
};

template <typename Ros2Type>
void convert_message(
    const rcutils_uint8_array_t *const ros2_message,
    rcutils_uint8_array_t *const resim_message) {
  REASSERT(ros2_message != nullptr);
  REASSERT(ros2_message->buffer != nullptr);
  REASSERT(resim_message != nullptr);

  rclcpp::Serialization<Ros2Type> serialization;

  Ros2Type deserialized;
  rclcpp::SerializedMessage message{*ros2_message};
  serialization.deserialize_message(&message, &deserialized);
  const auto converted_message = convert_from_ros2(deserialized);

  const std::size_t converted_size = converted_message.ByteSizeLong();

  rcl_serialized_message_t &result = *resim_message;
  result.allocator = ros2_message->allocator;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  result.buffer = reinterpret_cast<uint8_t *>(
      result.allocator.allocate(converted_size, result.allocator.state));
  result.buffer_length = converted_size;
  result.buffer_capacity = converted_size;
  converted_message.SerializeToArray(result.buffer, converted_size);
}

template <typename Ros2Type>
void generate_type_schema(ReSimConverterSchemaInfo *schema_info) {
  using ReSimType = decltype(convert_from_ros2(std::declval<Ros2Type>()));
  REASSERT(schema_info != nullptr);
  const auto allocator = rcl_get_default_allocator();

  const auto &descriptor = *ReSimType::GetDescriptor();
  const std::string name = descriptor.full_name();
  const std::string data = dependency_file_descriptor_set(descriptor);

  const auto copy_string_to_uint8_array =
      [&allocator](const std::string &string, rcutils_uint8_array_t &array) {
        array.allocator = allocator;
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        array.buffer = reinterpret_cast<uint8_t *>(
            allocator.allocate(string.size(), allocator.state));
        REASSERT(array.buffer != nullptr);
        array.buffer_length = string.size();
        array.buffer_capacity = string.size();
        std::memcpy(array.buffer, string.data(), string.size());
      };

  copy_string_to_uint8_array(name, schema_info->name);
  schema_info->encoding = "protobuf";
  copy_string_to_uint8_array(data, schema_info->data);
}

static const std::unordered_map<std::string, ConverterFunctions>
    converters_map = {
        {"tf2_msgs/msg/TFMessage",
         {convert_message<tf2_msgs::msg::TFMessage>,
          generate_type_schema<tf2_msgs::msg::TFMessage>}},
        {"nav_msgs/msg/Odometry",
         {convert_message<nav_msgs::msg::Odometry>,
          generate_type_schema<nav_msgs::msg::Odometry>}},
};

extern "C" bool resim_convert_supports_ros2_type(
    const char *ros2_message_type) {
  return converters_map.contains(ros2_message_type);
}

extern "C" ReSimConverterPluginStatus resim_convert_ros2_to_resim(
    const char *const ros2_message_type,
    const rcutils_uint8_array_t *const ros2_message,
    rcutils_uint8_array_t *const resim_message) {
  try {
    converters_map.at(ros2_message_type).converter(ros2_message, resim_message);
  } catch (...) {
    return RESIM_CONVERTER_PLUGIN_STATUS_ERROR;
  }
  return RESIM_CONVERTER_PLUGIN_STATUS_OK;
}

extern "C" ReSimConverterPluginStatus resim_convert_get_resim_schema(
    const char *const ros2_message_type,
    ReSimConverterSchemaInfo *schema_info) {
  converters_map.at(ros2_message_type).schema_getter(schema_info);
  return RESIM_CONVERTER_PLUGIN_STATUS_OK;
}

}  // namespace resim::msg
