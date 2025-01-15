// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>

#include "resim/geometry/polygon_distance.hh"

namespace resim::geometry {
namespace py = pybind11;

PYBIND11_MODULE(polygon_distance_python, m) {
  m.def(
      "polygon_distance",
      &polygon_distance,
      py::arg("polygon_a"),
      py::arg("polygon_b"));
}

}  // namespace resim::geometry
