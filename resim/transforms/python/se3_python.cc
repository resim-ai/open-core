// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>

#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::transforms {
namespace py = pybind11;

// A simple pybinding of SE3
// TODO(mikebauer) Add frames
PYBIND11_MODULE(se3_python, m) {
  py::module_::import("resim.transforms.python.so3_python");

  py::class_<SE3>(m, "SE3")
      // static members
      .def_readonly_static("DIMS", &SE3::DIMS)
      .def_readonly_static("DOF", &SE3::DOF)
      // static methods
      .def_static("identity", &SE3::identity<>)
      .def_static("exp", &SE3::exp<>, py::arg("alg"))
      // class constructors
      .def(py::init<>())
      .def(py::init<SO3>(), py::arg("rotation"))
      .def(py::init<Eigen::Vector3d>(), py::arg("translation"))
      .def(
          py::init<SO3, Eigen::Vector3d>(),
          py::arg("rotation"),
          py::arg("translation"))
      // operator overloads
      .def(py::self * py::self)
      .def(py::self * Eigen::Vector3d())
      // class methods
      .def(
          "rotate",
          py::overload_cast<const Eigen::Vector3d &>(&SE3::rotate, py::const_),
          py::arg("source_vector"))
      .def("inverse", &SE3::inverse)
      .def("arc_length", &SE3::arc_length)
      .def(
          "interp",
          py::overload_cast<double>(&SE3::interp, py::const_),
          py::arg("fraction"))
      .def("log", &SE3::log)
      .def(
          "is_approx",
          &SE3::is_approx,
          py::arg("other"),
          py::arg("precision") = math::DEFAULT_PRECISION)
      .def("rotation", &SE3::rotation)
      .def("translation", &SE3::translation);
}

}  // namespace resim::transforms
