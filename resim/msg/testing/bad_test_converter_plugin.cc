
#include <rclcpp/serialized_message.hpp>

#include "resim/msg/converter_plugin_status.h"

// This is a badly-named test plugin for testing the ConverterPlugin class.
// Because the function has the wrong name, we shouldn't be able to construct
// a ConverterPlugin with it.
extern "C" ReSimConverterPluginStatus resim_badly_named_converter(
    const char *const ros2_message_type,
    const rcutils_uint8_array_t *const ros2_message,
    rcutils_uint8_array_t *const resim_message) {
  return RESIM_CONVERTER_PLUGIN_STATUS_NO_MATCHING_CONVERTER;
}
