// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cstdint>
#include <cstring>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <string_view>
#include <unordered_map>

#include "resim/assert/assert.hh"
#include "resim/msg/converter_plugin_types.h"

namespace resim {

struct ConverterFunctions {
  using Converter =
      void (*)(const rcutils_uint8_array_t *, rcutils_uint8_array_t *);
  using SchemaGetter = void (*)(ReSimConverterSchemaInfo *);

  Converter converter;
  SchemaGetter schema_getter;
};

void convert_convertible_message_type(
    const rcutils_uint8_array_t *const ros2_message,
    rcutils_uint8_array_t *const resim_message) {
  REASSERT(ros2_message != nullptr);
  REASSERT(ros2_message->buffer != nullptr);
  REASSERT(resim_message != nullptr);

  const std::string converted_message{
      "I received: " +
      std::string{// NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
                  reinterpret_cast<const char *>(ros2_message->buffer),
                  ros2_message->buffer_length}};

  const std::size_t converted_size = converted_message.size();
  rcl_serialized_message_t &result = *resim_message;
  result.allocator = ros2_message->allocator;
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  result.buffer = reinterpret_cast<uint8_t *>(
      result.allocator.allocate(converted_size, result.allocator.state));
  result.buffer_length = converted_size;
  result.buffer_capacity = converted_size;
  std::memcpy(result.buffer, converted_message.data(), converted_size);
}

void convertible_message_type_schema(ReSimConverterSchemaInfo *schema_info) {
  REASSERT(schema_info != nullptr);
  constexpr std::string_view SCHEMA = "The schema for converted_message_type";

  schema_info->name = "converted_message_type";
  schema_info->encoding = "protobuf";
  auto &data = schema_info->data;
  data.allocator = rcl_get_default_allocator();
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  data.buffer = reinterpret_cast<uint8_t *>(
      data.allocator.allocate(SCHEMA.size(), data.allocator.state));
  data.buffer_length = SCHEMA.size();
  data.buffer_capacity = SCHEMA.size();
  std::memcpy(data.buffer, SCHEMA.data(), SCHEMA.size());
}

static const std::unordered_map<std::string, ConverterFunctions>
    converters_map = {
        {"convertible_message_type",
         {convert_convertible_message_type, convertible_message_type_schema}},
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
  try {
    converters_map.at(ros2_message_type).schema_getter(schema_info);
  } catch (...) {
    return RESIM_CONVERTER_PLUGIN_STATUS_ERROR;
  }
  return RESIM_CONVERTER_PLUGIN_STATUS_OK;
}

}  // namespace resim
