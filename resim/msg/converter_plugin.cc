// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/converter_plugin.hh"

#include <dlfcn.h>

#include <utility>

#include "resim/assert/assert.hh"

namespace resim::msg {

ConverterPlugin::ConverterPlugin(const std::filesystem::path &plugin_path)
    : handle_(dlopen(plugin_path.c_str(), RTLD_LAZY)) {
  REASSERT(handle_ != nullptr, "Failed to load plugin!");

  converter_ =
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      reinterpret_cast<const Converter>(dlsym(handle_, CONVERTER_NAME));
  REASSERT(converter_ != nullptr, "Failed to load_plugin!");
}

ConverterPlugin::~ConverterPlugin() { dlclose(handle_); }

std::optional<rclcpp::SerializedMessage> ConverterPlugin::try_convert(
    const std::string_view ros2_message_type,
    const rclcpp::SerializedMessage &ros2_message) const {
  rcl_serialized_message_t result;
  const auto status = converter_(
      ros2_message_type.data(),
      &ros2_message.get_rcl_serialized_message(),
      &result);
  REASSERT(
      status != RESIM_CONVERTER_PLUGIN_STATUS_ERROR,
      "Error encountered on conversion!");
  if (status == RESIM_CONVERTER_PLUGIN_STATUS_NO_MATCHING_CONVERTER) {
    return std::nullopt;
  }

  // result must be moved since the lvalue constructor of SerializedMessage will
  // do a deep copy resulting in a memory leak. We want to make sure ownershp is
  // transferred.
  // NOLINTNEXTLINE(performance-move-const-arg)
  return rclcpp::SerializedMessage{std::move(result)};
}

}  // namespace resim::msg
