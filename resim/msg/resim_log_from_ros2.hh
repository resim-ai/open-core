// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <filesystem>

namespace resim::msg {

// This function converts the ROS2 log at the given path to a ReSim log using
// the converter plugin at the given plugin path.
// @param[in] plugin_path - The path of the resim::msg::ConverterPlugin plug-in
//                          shared object to use for the conversion.
// @param[in] input_log - The input ROS2 bag to convert.
// @param[in] output_log - The location to put the ReSim MCAP log.
void resim_log_from_ros2(
    const std::filesystem::path &plugin_path,
    const std::filesystem::path &input_log,
    const std::filesystem::path &output_log);

}  // namespace resim::msg
