// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <utility>

#include "resim/transforms/se3.hh"

namespace resim::transforms {
namespace py = pybind11;

PYBIND11_MODULE(se3_python, m) {
  py::class_<SE3>(m, "SE3")
      .def_readonly_static("DIMS", &SE3::DIMS)
      .def_readonly_static("DOF", &SE3::DOF)
      .def(py::init<>())
      .def("log", &SE3::log)
      .def("exp", [](const SE3::TangentVector &arg) { return SE3::exp(arg); });
}

}  // namespace resim::transforms
