// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/pybind11.h>

#include "resim/geometry/box_box_distance.hh"
#include "resim/transforms/se3.hh"

namespace resim::geometry {
namespace py = pybind11;

using transforms::SE3;

PYBIND11_MODULE(box_box_distance_python, m) {
  py::module_::import("resim.geometry.python.oriented_box_python");

  m.def(
      "box_box_distance",
      &box_box_distance<SE3>,
      py::arg("box_1"),
      py::arg("box_2"));
}
}  // namespace resim::geometry
