// Copyright 2023 ReSim, Inc.
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
#include "resim/ros2/detection_from_ros2.hh"
#include "resim/ros2/header_from_ros2.hh"
#include "resim/ros2/navsat_from_ros2.hh"
#include "resim/ros2/odometry_from_ros2.hh"
#include "resim/ros2/oriented_box_from_ros2.hh"
#include "resim/ros2/pose_from_ros2.hh"
#include "resim/ros2/primitives_from_ros2.hh"
#include "resim/ros2/time_from_ros2.hh"
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
    {"vision_msgs/msg/Detection3DArray",
     {convert_message<vision_msgs::msg::Detection3DArray>,
      generate_type_schema<vision_msgs::msg::Detection3DArray>}},
    {"vision_msgs/msg/BoundingBox2D",
     {convert_message<vision_msgs::msg::BoundingBox2D>,
      generate_type_schema<vision_msgs::msg::BoundingBox2D>}},
    {"vision_msgs/msg/Detection2D",
     {convert_message<vision_msgs::msg::Detection2D>,
      generate_type_schema<vision_msgs::msg::Detection2D>}},
    {"vision_msgs/msg/Detection2DArray",
     {convert_message<vision_msgs::msg::Detection2DArray>,
      generate_type_schema<vision_msgs::msg::Detection2DArray>}},
    {"std_msgs/msg/Bool",
     {convert_message<std_msgs::msg::Bool>,
      generate_type_schema<std_msgs::msg::Bool>}},
    {"std_msgs/msg/Byte",
     {convert_message<std_msgs::msg::Byte>,
      generate_type_schema<std_msgs::msg::Byte>}},
    {"std_msgs/msg/Char",
     {convert_message<std_msgs::msg::Char>,
      generate_type_schema<std_msgs::msg::Char>}},
    {"std_msgs/msg/Empty",
     {convert_message<std_msgs::msg::Empty>,
      generate_type_schema<std_msgs::msg::Empty>}},
    {"std_msgs/msg/Float32",
     {convert_message<std_msgs::msg::Float32>,
      generate_type_schema<std_msgs::msg::Float32>}},
    {"std_msgs/msg/Float64",
     {convert_message<std_msgs::msg::Float64>,
      generate_type_schema<std_msgs::msg::Float64>}},
    {"std_msgs/msg/Int16",
     {convert_message<std_msgs::msg::Int16>,
      generate_type_schema<std_msgs::msg::Int16>}},
    {"std_msgs/msg/Int32",
     {convert_message<std_msgs::msg::Int32>,
      generate_type_schema<std_msgs::msg::Int32>}},
    {"std_msgs/msg/Int64",
     {convert_message<std_msgs::msg::Int64>,
      generate_type_schema<std_msgs::msg::Int64>}},
    {"std_msgs/msg/Int8",
     {convert_message<std_msgs::msg::Int8>,
      generate_type_schema<std_msgs::msg::Int8>}},
    {"std_msgs/msg/String",
     {convert_message<std_msgs::msg::String>,
      generate_type_schema<std_msgs::msg::String>}},
    {"std_msgs/msg/UInt16",
     {convert_message<std_msgs::msg::UInt16>,
      generate_type_schema<std_msgs::msg::UInt16>}},
    {"std_msgs/msg/UInt32",
     {convert_message<std_msgs::msg::UInt32>,
      generate_type_schema<std_msgs::msg::UInt32>}},
    {"std_msgs/msg/UInt64",
     {convert_message<std_msgs::msg::UInt64>,
      generate_type_schema<std_msgs::msg::UInt64>}},
    {"std_msgs/msg/UInt8",
     {convert_message<std_msgs::msg::UInt8>,
      generate_type_schema<std_msgs::msg::UInt8>}},

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

}  // namespace resim::ros2
