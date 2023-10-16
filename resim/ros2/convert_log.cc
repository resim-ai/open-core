// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cstdlib>
#include <cxxopts.hpp>
#include <filesystem>
#include <iostream>

#include "resim/ros2/resim_log_from_ros2.hh"

namespace resim::ros2 {

int convert_log(const int argc, char **const argv) {
  cxxopts::Options options{"convert_log", "Convert ROS2 Logs to ReSim Format."};
  constexpr auto DEFAULT_PLUGIN_PATH = "resim/ros2/default_converter_plugin.so";
  // clang-format off
  options.add_options()
    ("l,log", "Log location (required)", cxxopts::value<std::string>())
    ("o,output", "Output location (required)", cxxopts::value<std::string>())
    ("p,plugin", "Converter plugin location", cxxopts::value<std::string>()->default_value(DEFAULT_PLUGIN_PATH))
    ("h,help", "Print usage")
  ;
  // clang-format on
  auto options_result = options.parse(argc, argv);

  if (options_result.count("help") > 0U or options_result.count("log") == 0U or
      options_result.count("output") == 0U) {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  const std::filesystem::path input_path{
      options_result["log"].as<std::string>()};
  const std::filesystem::path output_path{
      options_result["output"].as<std::string>()};
  const std::filesystem::path plugin_path{
      options_result["plugin"].as<std::string>()};

  resim_log_from_ros2(plugin_path, input_path, output_path);
  return EXIT_SUCCESS;
}

}  // namespace resim::ros2

int main(int argc, char **argv) { return resim::ros2::convert_log(argc, argv); }
