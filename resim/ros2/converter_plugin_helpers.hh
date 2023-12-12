// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// converter_plugin_helpers.hh
//
// This file contains some function templates that can be be used to create
// converter plugins for custom message types. In particular, because such
// converters are typically defined in other namespaces, we allow users to pass
// in a ConverterFunctor equipped with whatever converters they would like. For
// usage examples, see default_converter_plugin.cc

#include <cstring>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <string>

#include "resim/assert/assert.hh"
#include "resim/ros2/converter_plugin_types.h"
#include "resim/utils/proto/dependency_file_descriptor_set.hh"

namespace resim::ros2 {

// Convert a serialized ros2 message to a serialized resim message.
// @tparam[in] Ros2Type: The message type of the ros2_message.
// @tparam[in] ConverterFunctor: A functor with the needed converter for this
//             type.
// @param[in] ros2_message: The serialized message to convert.
// @param[out] resim_message: The converted resim message.
template <typename Ros2Type, typename ConverterFunctor>
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
  const auto converted_message = ConverterFunctor()(deserialized);

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

// Get the schema info for the type we want to convert to.
// @tparam[in] Ros2Type: The message type of the ros2_message.
// @tparam[in] ConverterFunctor: A functor with the needed converter for this
//             type.
// @param[out] schema_info: The schema info for the type we're converting to.
template <typename Ros2Type, typename ConverterFunctor>
void generate_type_schema(ReSimConverterSchemaInfo *schema_info) {
  using ReSimType = decltype(ConverterFunctor()(std::declval<Ros2Type>()));
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

}  // namespace resim::ros2
