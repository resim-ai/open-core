// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/ros2/resim_log_from_ros2.hh"

#include <google/protobuf/message.h>
#include <gtest/gtest.h>

#include "resim/ros2/resim_log_from_ros2_test_helpers.hh"
#include "resim/testing/test_directory.hh"

namespace resim::ros2 {

TEST(ResimLogFromRos2Test, TestConversion) {
  // SETUP
  // Make sure we clean up protobuf:
  std::atexit(google::protobuf::ShutdownProtobufLibrary);

  const testing::TestDirectoryRAII test_directory;
  const std::filesystem::path input_path{test_directory.test_file_path()};
  const std::filesystem::path output_path{
      test_directory.test_file_path("mcap")};
  constexpr auto PLUGIN_PATH =
      "resim/ros2/testing/resim_log_from_ros2_test_converter_plugin.so";

  populate_log(input_path);

  // ACTION
  resim_log_from_ros2(PLUGIN_PATH, input_path, output_path);

  // VERIFICATION
  verify_log_contents(output_path);
}

}  // namespace resim::ros2
