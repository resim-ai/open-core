// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>

namespace resim::transforms {
namespace py = pybind11;

// A simple pybinding of Eigen::Quaterniond intended to facilitate the
// construction of SO3 objects. We leave out functionality like inversion and
// composition because users should use SO3 for thess.
PYBIND11_MODULE(quaternion, m) {
  py::class_<Eigen::Quaterniond>(m, "Quaternion")
      .def(py::init<const Eigen::Vector4d &>(), py::arg("vector"))
      .def(
          py::init<double, double, double, double>(),
          py::arg("w"),
          py::arg("x"),
          py::arg("y"),
          py::arg("z"))
      .def("w", py::overload_cast<>(&Eigen::Quaterniond::w, py::const_))
      .def("x", py::overload_cast<>(&Eigen::Quaterniond::x, py::const_))
      .def("y", py::overload_cast<>(&Eigen::Quaterniond::y, py::const_))
      .def("z", py::overload_cast<>(&Eigen::Quaterniond::z, py::const_));
}

}  // namespace resim::transforms
