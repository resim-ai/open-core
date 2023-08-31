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
#include <string>
#include <string_view>
#include <unordered_map>

#include "resim/assert/assert.hh"
#include "resim/msg/converter_plugin_types.h"
#include "resim/msg/detection_from_ros2.hh"
#include "resim/msg/header_from_ros2.hh"
#include "resim/msg/navsat_from_ros2.hh"
#include "resim/msg/odometry_from_ros2.hh"
#include "resim/msg/oriented_box_from_ros2.hh"
#include "resim/msg/pose_from_ros2.hh"
#include "resim/msg/time_from_ros2.hh"
#include "resim/msg/transform_from_ros2.hh"
#include "resim/utils/proto/dependency_file_descriptor_set.hh"

namespace resim::msg {

namespace {

// A helper struct for holding a Converter and SchemaGetter together in our map
// of ROS2 types to converters (see below)
struct ConverterFunctions {
  using Converter = std::function<
      void(const rcutils_uint8_array_t *, rcutils_uint8_array_t *)>;
  using SchemaGetter = std::function<void(ReSimConverterSchemaInfo *)>;

  Converter converter;
  SchemaGetter schema_getter;
};

// This is the primary function used to convert a given serialized Ros2Type to
// a serialized ReSimType.
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

// This is the primary function used to get the output schema info for a given
// Ros2Type.
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

// This map holds onto all of the functions we need to get the schema and
// convert for each given ROS2 message.
const std::unordered_map<std::string, ConverterFunctions> converters_map = {
    {"builtin_interfaces/msg/Time",
     {convert_message<builtin_interfaces::msg::Time>,
      generate_type_schema<builtin_interfaces::msg::Time>}},
    {"geometry_msgs/msg/Pose",
     {convert_message<geometry_msgs::msg::Pose>,
      generate_type_schema<geometry_msgs::msg::Pose>}},
    {"geometry_msgs/msg/PoseWithCovariance",
     {convert_message<geometry_msgs::msg::PoseWithCovariance>,
      generate_type_schema<geometry_msgs::msg::PoseWithCovariance>}},
    {"geometry_msgs/msg/Transform",
     {convert_message<geometry_msgs::msg::Transform>,
      generate_type_schema<geometry_msgs::msg::Transform>}},
    {"geometry_msgs/msg/TransformStamped",
     {convert_message<geometry_msgs::msg::TransformStamped>,
      generate_type_schema<geometry_msgs::msg::TransformStamped>}},
    {"geometry_msgs/msg/Twist",
     {convert_message<geometry_msgs::msg::Twist>,
      generate_type_schema<geometry_msgs::msg::Twist>}},
    {"geometry_msgs/msg/TwistWithCovariance",
     {convert_message<geometry_msgs::msg::TwistWithCovariance>,
      generate_type_schema<geometry_msgs::msg::TwistWithCovariance>}},
    {"nav_msgs/msg/Odometry",
     {convert_message<nav_msgs::msg::Odometry>,
      generate_type_schema<nav_msgs::msg::Odometry>}},
    {"sensor_msgs/msg/NavSatFix",
     {convert_message<sensor_msgs::msg::NavSatFix>,
      generate_type_schema<sensor_msgs::msg::NavSatFix>}},
    {"std_msgs/msg/Header",
     {convert_message<std_msgs::msg::Header>,
      generate_type_schema<std_msgs::msg::Header>}},
    {"tf2_msgs/msg/TFMessage",
     {convert_message<tf2_msgs::msg::TFMessage>,
      generate_type_schema<tf2_msgs::msg::TFMessage>}},
    {"vision_msgs/msg/BoundingBox3D",
     {convert_message<vision_msgs::msg::BoundingBox3D>,
      generate_type_schema<vision_msgs::msg::BoundingBox3D>}},
    {"vision_msgs/msg/Detection3D",
     {convert_message<vision_msgs::msg::Detection3D>,
      generate_type_schema<vision_msgs::msg::Detection3D>}},

};

}  // namespace

// Implement the actual functions for this plugin:

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
