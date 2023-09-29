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
  py::class_<SE3>(m, "SE3")
      .def_readonly_static("DIMS", &SE3::DIMS)
      .def_readonly_static("DOF", &SE3::DOF)
      .def(py::init<>())
      .def(py::init<SO3>())
      .def(py::init<Eigen::Vector3d>())
      .def("identity", &SE3::identity<>)
      .def(py::self * py::self)
      .def(py::self * Eigen::Vector3d())
      .def(
          "rotate",
          py::overload_cast<const Eigen::Vector3d &>(&SE3::rotate, py::const_))
      .def("inverse", &SE3::inverse)
      .def("arc_length", &SE3::arc_length)
      .def("interp", py::overload_cast<double>(&SE3::interp, py::const_))
      .def("exp", &SE3::exp<>)
      .def("exp_diff", &SE3::exp_diff)
      .def("log", &SE3::log)
      .def("adjoint", py::overload_cast<>(&SE3::adjoint, py::const_))
      .def_static(
          "algebra_adjoint",
          py::overload_cast<const SE3::TangentVector &>(&SE3::adjoint))
      .def(
          "adjoint_times",
          py::overload_cast<const SE3::TangentVector &>(
              &SE3::adjoint_times,
              py::const_))
      .def_static(
          "algebra_adjoint_times",
          py::overload_cast<
              const SE3::TangentVector &,
              const SE3::TangentVector &>(&SE3::adjoint_times))
      .def("is_approx", &SE3::is_approx)
      .def("is_approx_transform", &SE3::is_approx_transform)
      .def("rotation", &SE3::rotation)
      .def("translation", &SE3::translation);
}

}  // namespace resim::transforms
