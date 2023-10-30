// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iostream>

#include "resim/ros2/resim_log_from_ros2_test_helpers.hh"

namespace resim::ros2 {
namespace py = pybind11;

PYBIND11_MODULE(resim_log_from_ros2_test_helpers_python, m) {
  m.def("populate_log", [](const std::string &log_path) {
    return populate_log(log_path);
  });
  m.def("verify_log_contents", [](const std::string &log_path) {
    return verify_log_contents(log_path);
  });
}

}  // namespace resim::ros2
