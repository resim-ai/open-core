// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <rcutils/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// Status return type for converter plugins. See resim/ros2/converter_plugin.hh
// for more details.
typedef enum {
  RESIM_CONVERTER_PLUGIN_STATUS_OK = 0,
  RESIM_CONVERTER_PLUGIN_STATUS_ERROR,
} ReSimConverterPluginStatus;

// Schema info we can use to write the converted messages to MCAP. Should fit
// one of the profiles here: https://mcap.dev/spec/registry#schema-encodings
typedef struct {
  rcutils_uint8_array_t name;
  const char *encoding;
  rcutils_uint8_array_t data;
} ReSimConverterSchemaInfo;

#ifdef __cplusplus
}  // extern "C"
#endif
