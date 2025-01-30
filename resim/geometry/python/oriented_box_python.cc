// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>

#include "resim/geometry/oriented_box.hh"
#include "resim/transforms/se3.hh"

namespace resim::geometry {
namespace py = pybind11;

using transforms::SE3;

PYBIND11_MODULE(oriented_box_python, m) {
  py::module_::import("resim.transforms.python.se3_python");

  py::class_<OrientedBox<SE3>>(m, "OrientedBox")
      .def(
          py::init<SE3, Eigen::Vector3d>(),
          py::kw_only(),
          py::arg("reference_from_box"),
          py::arg("extents"))
      .def("reference_from_box", &OrientedBox<SE3>::reference_from_box)
      .def("extents", &OrientedBox<SE3>::extents)
      .def("set_reference_from_box", &OrientedBox<SE3>::set_reference_from_box)
      .def("set_extents", &OrientedBox<SE3>::set_extents);
}
}  // namespace resim::geometry
