
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Status return type for converter plugins. See resim/msg/converter_plugin.hh
// for more details.
typedef enum {
  RESIM_CONVERTER_PLUGIN_STATUS_OK = 0,
  RESIM_CONVERTER_PLUGIN_STATUS_NO_MATCHING_CONVERTER,
  RESIM_CONVERTER_PLUGIN_STATUS_ERROR,
} ReSimConverterPluginStatus;

#ifdef __cplusplus
}  // extern "C"
#endif
