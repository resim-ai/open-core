
#include "resim/msg/converter_plugin.hh"

#include <dlfcn.h>

#include <utility>

#include "resim/assert/assert.hh"

namespace resim::msg {

ConverterPlugin::ConverterPlugin(const std::string &plugin_path)
    : handle_(dlopen(plugin_path.c_str(), RTLD_LAZY)) {
  // Clear dlerror()
  dlerror();

  const char *maybe_error = dlerror();
  const auto maybe_error_string = [&maybe_error]() {
    return std::string(maybe_error != nullptr ? maybe_error : "");
  };
  REASSERT(
      handle_ != nullptr,
      "Failed to load plugin: " + maybe_error_string());

  // Clear dlerror()
  dlerror();
  converter_ =
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      reinterpret_cast<const Converter>(dlsym(handle_, CONVERTER_NAME));
  maybe_error = dlerror();
  REASSERT(
      converter_ != nullptr and maybe_error == nullptr,
      "Failed to load plugin: " + maybe_error_string());
}

ConverterPlugin::~ConverterPlugin() {
  if (handle_ != nullptr) {
    dlclose(handle_);
  }
}

std::optional<rclcpp::SerializedMessage> ConverterPlugin::try_convert(
    const char *ros2_message_type,
    const rclcpp::SerializedMessage &ros2_message) const {
  rcl_serialized_message_t result;
  const auto status = converter_(
      ros2_message_type,
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
