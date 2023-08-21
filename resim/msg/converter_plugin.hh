
#pragma once

#include <optional>
#include <rclcpp/serialized_message.hpp>
#include <string>

#include "resim/msg/converter_plugin_status.h"

namespace resim::msg {

struct ConverterPlugin {
  explicit ConverterPlugin(const std::string &plugin_path);
  ConverterPlugin(const ConverterPlugin &) = default;
  ConverterPlugin(ConverterPlugin &&) = default;
  ConverterPlugin &operator=(const ConverterPlugin &) = default;
  ConverterPlugin &operator=(ConverterPlugin &&) = default;
  ~ConverterPlugin();

  std::optional<rclcpp::SerializedMessage> try_convert(
      const char *ros2_message_type,
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
