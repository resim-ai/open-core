// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <filesystem>

namespace resim::ros2 {

// Helper function to populate a ROS2 log with arbitrary test messages on our
// specified channels.
void populate_log(const std::filesystem::path &log_path);

// Helper function to confirm that the ReSim log at the given path contains the
// same messages as the test messages added by populate_log().
void verify_log_contents(const std::filesystem::path &log_path);

}  // namespace resim::ros2
