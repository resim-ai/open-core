// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>

#include "resim/ros2/converter_plugin_helpers.hh"
#include "resim/ros2/converter_plugin_types.h"
#include "resim/ros2/odometry_from_ros2.hh"
#include "resim/ros2/transform_from_ros2.hh"

namespace resim::ros2 {

namespace {

class ConverterFunctor {
 public:
  template <typename Ros2Type>
  auto operator()(const Ros2Type &ros2_msg) {
    return convert_from_ros2(ros2_msg);
  }
};

template <typename Ros2Type>
auto convert_message(
    const rcutils_uint8_array_t *const ros2_message,
    rcutils_uint8_array_t *const resim_message) {
  return resim::ros2::convert_message<Ros2Type, ConverterFunctor>(
      ros2_message,
      resim_message);
}

template <typename Ros2Type>
auto generate_type_schema(ReSimConverterSchemaInfo *schema_info) {
  return resim::ros2::generate_type_schema<Ros2Type, ConverterFunctor>(
      schema_info);
}

// A helper struct for holding a Converter and SchemaGetter together in our map
// of ROS2 types to converters (see below)
struct ConverterFunctions {
  using Converter = std::function<
      void(const rcutils_uint8_array_t *, rcutils_uint8_array_t *)>;
  using SchemaGetter = std::function<void(ReSimConverterSchemaInfo *)>;

  Converter converter;
  SchemaGetter schema_getter;
};

// This map holds onto all of the functions we need to get the schema and
// convert for each given ROS2 message.
const std::unordered_map<std::string, ConverterFunctions> converters_map = {
    {"nav_msgs/msg/Odometry",
     {convert_message<nav_msgs::msg::Odometry>,
      generate_type_schema<nav_msgs::msg::Odometry>}},
    {"tf2_msgs/msg/TFMessage",
     {convert_message<tf2_msgs::msg::TFMessage>,
      generate_type_schema<tf2_msgs::msg::TFMessage>}},
};

}  // namespace

// Implement the actual functions for this plugin:

extern "C" bool resim_convert_supports_ros2_type(
    const char *ros2_message_type) {
  if (std::string(ros2_message_type) == "std_msgs/msg/UInt8MultiArray") {
    return true;
  }
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
  if (std::string(ros2_message_type) == "std_msgs/msg/UInt8MultiArray") {
    converters_map.cbegin()->second.schema_getter(schema_info);
    return RESIM_CONVERTER_PLUGIN_STATUS_OK;
  }
  converters_map.at(ros2_message_type).schema_getter(schema_info);
  return RESIM_CONVERTER_PLUGIN_STATUS_OK;
}

}  // namespace resim::ros2
