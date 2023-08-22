// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <filesystem>
#include <optional>
#include <rclcpp/serialized_message.hpp>

#include "resim/msg/converter_plugin_status.h"

namespace resim::msg {

// This class provides an interface for loading plugins which convert serialized
// ROS2 messages to their resim equivalents. These plugins are expected to
// contain a single function with the signature:
//
// ReSimConverterPluginStatus resim_convert_ros2_to_resim(
//     const char *ros2_message_type,
//     const rcutils_uint8_array_t *ros2_message,
//     rcutils_uint8_array_t *resim_message);
//
// The arguments for this function are defined like so:
//
// ros2_message_type [input]: This is a C string containing the name of the ros2
//                            message type we're trying to convert (e.g.
//                            tf2_msgs::msg::TFMessage). If this plugin knows
//                            how to convert this type of message, it will do so
//                            and return RESIM_CONVERTER_PLUGIN_STATUS_OK.
//                            Otherwise, it will immediately return
//                            RESIM_CONVERTER_PLUGIN_STATUS_NO_MATCHING_CONVERTER.
//
// ros2_message [input]: A pointer to a ROS-style byte array containing the
//                       serialized ros2 message to convert.
//
// resim_message [output]: A pointer to a ROS-style byte array to put the
//                         converted serialized ReSim type into.
//
// returns: RESIM_CONVERTER_PLUGIN_STATUS_OK if the conversion was successful,
// RESIM_CONVERTER_PLUGIN_STATUS_NO_MATCHING_CONVERTER if this plugin doesn't
// know how to convert this ros2_message_type, or
// RESIM_CONVERTER_PLUGIN_STATUS_ERROR if some sort of error occurs.
struct ConverterPlugin {
  // Constructor. This function loads the plugin from the given path and stores
  // it in this object.
  explicit ConverterPlugin(const std::filesystem::path &plugin_path);
  ConverterPlugin(const ConverterPlugin &) = default;
  ConverterPlugin(ConverterPlugin &&) = default;
  ConverterPlugin &operator=(const ConverterPlugin &) = default;
  ConverterPlugin &operator=(ConverterPlugin &&) = default;
  ~ConverterPlugin();

  // Attempt to convert a ros2 message with the contained plugin.
  // @param[in] ros2_message_type: The message type of the ros2_message.
  // @param[in] ros2_message: The message to convert.
  // @returns The converted message if successful or nullopt if this converter
  //          doesn't know how to handle this type of message.
  // @throws If the contained converter plugin returns
  // RESIM_CONVERTER_PLUGIN_STATUS_ERROR
  std::optional<rclcpp::SerializedMessage> try_convert(
      std::string_view ros2_message_type,
      const rclcpp::SerializedMessage &ros2_message) const;

 private:
  static constexpr auto CONVERTER_NAME = "resim_convert_ros2_to_resim";
  using Converter = ReSimConverterPluginStatus (*)(
      const char *,
      const rcutils_uint8_array_t *,
      rcutils_uint8_array_t *);
  Converter converter_;
  void *handle_;
};

}  // namespace resim::msg
