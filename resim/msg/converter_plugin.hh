// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <cstddef>
#include <filesystem>
#include <rclcpp/serialized_message.hpp>
#include <string_view>
#include <vector>

#include "resim/msg/converter_plugin_types.h"

namespace resim::msg {

// This class provides an interface for loading plugins which convert serialized
// ROS2 messages to their resim equivalents. These plugins are expected to
// contain three functions with the signatures:
//
//
// // This function tells whether this plugin supports converting the given
// // message
// // type.
// // @param[in] ros2_message_type - The message type we want to try to convert
// // @returns Whether or not we support converting it
// bool resim_convert_supports_ros2_type(const char *ros2_message_type);
//
//
// // This function performs the message conversion and returns an ERROR status
// // if this plugin doesn't know how to do it.
// // @param[in] ros2_message_type - The message type we want to try to convert
// // @param[in] ros2_message - A pointer to a ROS-style byte array containing
// //                           a serialized ros2 message to convert.
// // @param[out] resim_message - A pointer to a ROS-stype byte array to put our
// //                             converted serialized ReSim type into. Note
// //                             that this is really just a byte array. We
// //                             simply use the ROS type for the sake of
// //                             convenience and memory safety. These bytes
// //                             should be ready to log into an MCAP file.
// // @returns OK status if successful, or an ERROR otherwise.
// ReSimConverterPluginStatus resim_convert_ros2_to_resim(
//     const char *ros2_message_type,
//     const rcutils_uint8_array_t *ros2_message,
//     rcutils_uint8_array_t *resim_message);
//
//
// // This function returns the schema info for the type that this plugin
// // converts ros2_message_type to. This schema info is designed so we can
// // write the result to an MCAP file and so it should fit one of the profiles
// // here: https://mcap.dev/spec/registry#schema-encodings
// // @param[in] ros2_message_type - We want the schema info for the type we
// //                                convert to *from* this type.
// // @param[out] schema_info - The schema info for the type converted to.
// // @returns OK status if successful, or an ERROR otherwise.
// ReSimConverterPluginStatus resim_convert_get_resim_schema(
//     const char *const ros2_message_type,
//     ReSimConverterSchemaInfo *schema_info);
//
class ConverterPlugin {
 public:
  // Schema info we can use to write the converted messages to MCAP. Should fit
  // one of the profiles here: https://mcap.dev/spec/registry#schema-encodings
  struct SchemaInfo {
    std::string name;
    std::string encoding;
    std::vector<std::byte> data;
  };

  // Constructor. This function loads the plugin from the given path and stores
  // it in this object.
  explicit ConverterPlugin(const std::filesystem::path &plugin_path);
  ConverterPlugin(const ConverterPlugin &) = default;
  ConverterPlugin(ConverterPlugin &&) = default;
  ConverterPlugin &operator=(const ConverterPlugin &) = default;
  ConverterPlugin &operator=(ConverterPlugin &&) = default;
  ~ConverterPlugin();

  // Determine whether this plugin supports converting a particular type.
  // @param[in] ros2_message_type - The type we want to convert.
  bool supports_type(std::string_view ros2_message_type) const;

  // Convert a ros2 message with the contained plugin.
  // @param[in] ros2_message_type: The message type of the ros2_message.
  // @param[in] ros2_message: The message to convert.
  // @returns The converted message if successful or nullopt if this
  // converter
  //          doesn't know how to handle this type of message.
  // @throws If the contained converter plugin returns
  // RESIM_CONVERTER_PLUGIN_STATUS_ERROR
  std::vector<std::byte> convert(
      std::string_view ros2_message_type,
      const rclcpp::SerializedMessage &ros2_message) const;

  // Get the schema info for the type we want to convert to.
  // @param[in] ros2_message_type - The type we want to convert.
  SchemaInfo get_schema(std::string_view ros2_message_type) const;

 private:
  static constexpr auto SUPPORTER_NAME = "resim_convert_supports_ros2_type";
  static constexpr auto CONVERTER_NAME = "resim_convert_ros2_to_resim";
  static constexpr auto GET_SCHEMA_NAME = "resim_convert_get_resim_schema";

  using SupportsType = bool (*)(const char *);
  using Converter = ReSimConverterPluginStatus (*)(
      const char *,
      const rcutils_uint8_array_t *,
      rcutils_uint8_array_t *);
  using SchemaGetter =
      ReSimConverterPluginStatus (*)(const char *, ReSimConverterSchemaInfo *);

  SupportsType supports_type_;
  Converter converter_;
  SchemaGetter schema_getter_;
  void *handle_;
};

}  // namespace resim::msg
