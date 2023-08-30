// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/converter_plugin.hh"

#include <dlfcn.h>

#include <span>
#include <utility>

#include "resim/assert/assert.hh"

namespace resim::msg {

ConverterPlugin::ConverterPlugin(const std::filesystem::path &plugin_path) {
  // Clear dlerror()
  dlerror();
  const auto check_dlerror = []() {
    const char *maybe_error = dlerror();
    REASSERT(
        maybe_error == nullptr,
        "Failed to load plugin! " +
            std::string(maybe_error ? maybe_error : ""));
  };

  // We need to clear dlerror() first.
  // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
  handle_ = dlopen(plugin_path.c_str(), RTLD_LAZY);
  check_dlerror();
  REASSERT(handle_ != nullptr);

  converter_ =
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      reinterpret_cast<const ConverterPtr>(dlsym(handle_, CONVERTER_NAME_));
  check_dlerror();
  REASSERT(converter_ != nullptr);

  supports_type_ =
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      reinterpret_cast<const SupportsTypePtr>(dlsym(handle_, SUPPORTER_NAME_));
  check_dlerror();
  REASSERT(supports_type_ != nullptr);

  schema_getter_ =
      // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
      reinterpret_cast<const SchemaGetterPtr>(dlsym(handle_, GET_SCHEMA_NAME_));
  check_dlerror();
  REASSERT(schema_getter_ != nullptr);
}

ConverterPlugin::~ConverterPlugin() {
  if (handle_ != nullptr) {
    dlclose(handle_);
  }
}

bool ConverterPlugin::supports_type(std::string_view ros2_message_type) const {
  REASSERT(supports_type_ != nullptr);
  return supports_type_(ros2_message_type.data());
}

std::vector<std::byte> ConverterPlugin::convert(
    const std::string_view ros2_message_type,
    const rclcpp::SerializedMessage &ros2_message) const {
  REASSERT(converter_ != nullptr);
  REASSERT(
      supports_type(ros2_message_type),
      "Can't convert unsupported message type!");

  rcl_serialized_message_t resim_message;
  const auto status = converter_(
      ros2_message_type.data(),
      &ros2_message.get_rcl_serialized_message(),
      &resim_message);
  REASSERT(
      status != RESIM_CONVERTER_PLUGIN_STATUS_ERROR,
      "Error encountered on conversion!");

  std::vector<std::byte> result;
  result.reserve(resim_message.buffer_length);
  const std::span buffer{resim_message.buffer, resim_message.buffer_length};
  for (const auto s : buffer) {
    result.push_back(static_cast<std::byte>(s));
  }
  resim_message.allocator.deallocate(
      resim_message.buffer,
      resim_message.allocator.state);
  return result;
}

ConverterPlugin::SchemaInfo ConverterPlugin::get_schema(
    std::string_view ros2_message_type) const {
  REASSERT(schema_getter_ != nullptr);
  REASSERT(
      supports_type(ros2_message_type),
      "Can't convert unsupported message type!");
  ReSimConverterSchemaInfo schema_info;
  const auto status = schema_getter_(ros2_message_type.data(), &schema_info);
  REASSERT(
      status != RESIM_CONVERTER_PLUGIN_STATUS_ERROR,
      "Error encountered on schema get!");

  SchemaInfo result{
      .name = std::string(
          // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
          reinterpret_cast<const char *>(schema_info.name.buffer),
          schema_info.name.buffer_length),
      .encoding = schema_info.encoding,
  };
  const std::span buffer{
      schema_info.data.buffer,
      schema_info.data.buffer_length};
  result.data.reserve(schema_info.data.buffer_length);
  for (const auto s : buffer) {
    result.data.push_back(static_cast<std::byte>(s));
  }
  schema_info.data.allocator.deallocate(
      schema_info.name.buffer,
      schema_info.name.allocator.state);
  schema_info.data.allocator.deallocate(
      schema_info.data.buffer,
      schema_info.data.allocator.state);
  return result;
}

}  // namespace resim::msg
