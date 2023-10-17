// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "resim/ros2/resim_log_from_ros2.hh"

namespace resim::ros2 {
namespace py = pybind11;

// A simple pybinding of Eigen::Quaterniond intended to facilitate the
// construction of SO3 objects. We leave out functionality like inversion and
// composition because users should use SO3 for thess.
PYBIND11_MODULE(resim_log_from_ros2_python, m) {
  m.def(
      "resim_log_from_ros2",
      [](const std::string &plugin_path,
         const std::string &input_log_path,
         const std::string &output_log_path) {
        return resim_log_from_ros2(
            plugin_path,
            input_log_path,
            output_log_path);
      });
}

}  // namespace resim::ros2
