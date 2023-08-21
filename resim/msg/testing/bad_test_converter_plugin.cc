
#include <rclcpp/serialized_message.hpp>

#include "resim/msg/converter_plugin_status.h"

extern "C" ReSimConverterPluginStatus resim_badly_named_converter(
    const char *const ros2_message_type,
    const rcutils_uint8_array_t *const ros2_message,
    rcutils_uint8_array_t *const resim_message) {
  return RESIM_CONVERTER_PLUGIN_STATUS_NO_MATCHING_CONVERTER;
}
