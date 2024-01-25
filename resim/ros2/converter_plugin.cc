// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/ros2/converter_plugin.hh"

#include <dlfcn.h>
#include <glog/logging.h>

#include <span>
#include <utility>

#include "resim/assert/assert.hh"

namespace resim::ros2 {

namespace {

// Simple helper to make sure we initialize the buffer with nullptr. T
// can be rmw_serialized_message_t or rcutils_uint8_array_t.
template <typename T>
T make_clean_serialized_message() {
  return T{
      .buffer = nullptr,
      .buffer_length = 0U,
      .buffer_capacity = 0U,
      .allocator =
          rcutils_allocator_t{
              .allocate = nullptr,
              .deallocate = nullptr,
              .reallocate = nullptr,
              .zero_allocate = nullptr,
              .state = nullptr,
          },
  };
}

// Simple RAII class to make sure we clean up this message if it's been alloc'd
template <typename T>
class ReSimMessageRAII {
 public:
  explicit ReSimMessageRAII(T &message) : message_{message} {}
  ReSimMessageRAII(const ReSimMessageRAII &) = delete;
  ReSimMessageRAII(ReSimMessageRAII &&) = delete;
  ReSimMessageRAII &operator=(const ReSimMessageRAII &) = delete;
  ReSimMessageRAII &operator=(ReSimMessageRAII &&) = delete;

  ~ReSimMessageRAII() {
    if (message_.buffer != nullptr) {
      message_.allocator.deallocate(message_.buffer, message_.allocator.state);
    }
  }

 private:
  T &message_;
};

}  // namespace

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

std::optional<std::vector<std::byte>> ConverterPlugin::convert(
    const std::string_view ros2_message_type,
    const rclcpp::SerializedMessage &ros2_message) const {
  REASSERT(converter_ != nullptr);
  REASSERT(
      supports_type(ros2_message_type),
      "Can't convert unsupported message type!");

  auto resim_message{make_clean_serialized_message<rcl_serialized_message_t>()};
  ReSimMessageRAII raii{resim_message};
  const auto status = converter_(
      ros2_message_type.data(),
      &ros2_message.get_rcl_serialized_message(),
      &resim_message);
  if (status == RESIM_CONVERTER_PLUGIN_STATUS_ERROR) {
    LOG(WARNING) << "Error encountered on conversion! Skipping Message...";
    return std::nullopt;
  }

  std::vector<std::byte> result;
  result.reserve(resim_message.buffer_length);
  const std::span buffer{resim_message.buffer, resim_message.buffer_length};
  for (const auto s : buffer) {
    result.push_back(static_cast<std::byte>(s));
  }
  return result;
}

ConverterPlugin::SchemaInfo ConverterPlugin::get_schema(
    std::string_view ros2_message_type) const {
  REASSERT(schema_getter_ != nullptr);
  REASSERT(
      supports_type(ros2_message_type),
      "Can't convert unsupported message type!");
  ReSimConverterSchemaInfo schema_info{
      .name = make_clean_serialized_message<rcutils_uint8_array_t>(),
      .encoding = nullptr,
      .data = make_clean_serialized_message<rcutils_uint8_array_t>(),
  };
  ReSimMessageRAII name_raii{schema_info.name};
  ReSimMessageRAII data_raii{schema_info.data};
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
  return result;
}

}  // namespace resim::ros2
