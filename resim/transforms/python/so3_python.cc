// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>

#include "resim/transforms/so3.hh"

namespace resim::transforms {
namespace py = pybind11;

// A simple pybinding of SO3
// TODO(mikebauer) Add frames
PYBIND11_MODULE(so3_python, m) {
  py::class_<SO3>(m, "SO3")
      .def_readonly_static("DIMS", &SO3::DIMS)
      .def_readonly_static("DOF", &SO3::DOF)
      .def(py::init<>())
      .def(py::init<const Eigen::Quaterniond &>())
      .def("identity", &SO3::identity<>)
      .def(py::self * py::self)
      .def(py::self * Eigen::Vector3d())
      .def(
          "rotate",
          py::overload_cast<const Eigen::Vector3d &>(&SO3::rotate, py::const_))
      .def("inverse", &SO3::inverse)
      .def("interp", py::overload_cast<double>(&SO3::interp, py::const_))
      .def("exp", &SO3::exp<>)
      .def("exp_diff", &SO3::exp_diff)
      .def("log", &SO3::log)
      .def("adjoint", py::overload_cast<>(&SO3::adjoint, py::const_))
      .def_static(
          "algebra_adjoint",
          py::overload_cast<const SO3::TangentVector &>(&SO3::adjoint))
      .def(
          "adjoint_times",
          py::overload_cast<const SO3::TangentVector &>(
              &SO3::adjoint_times,
              py::const_))
      .def_static(
          "algebra_adjoint_times",
          py::overload_cast<
              const SO3::TangentVector &,
              const SO3::TangentVector &>(&SO3::adjoint_times))
      .def("is_approx", &SO3::is_approx)
      .def("is_approx_transform", &SO3::is_approx_transform)
      .def("rotation_matrix", &SO3::rotation_matrix)
      .def("quaternion", &SO3::quaternion);
}

}  // namespace resim::transforms
