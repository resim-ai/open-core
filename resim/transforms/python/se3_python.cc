// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>
#include <utility>

#include "resim/transforms/se3.hh"

namespace resim::transforms {
namespace py = pybind11;

// A simple pybinding of SE3
// TODO(mikebauer) Add frames and other operators
PYBIND11_MODULE(se3_python, m) {
  py::class_<SE3>(m, "SE3")
      .def_readonly_static("DIMS", &SE3::DIMS)
      .def_readonly_static("DOF", &SE3::DOF)
      .def(py::init<>())
      .def("log", &SE3::log)
      .def("translation", &SE3::translation)
      .def("exp", [](const SE3::TangentVector &arg) { return SE3::exp(arg); })
      .def("inverse", &SE3::inverse)
      .def(py::self * py::self)
      .def(py::self * Eigen::Vector3d());
}

}  // namespace resim::transforms
