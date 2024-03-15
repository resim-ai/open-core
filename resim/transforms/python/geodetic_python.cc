// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <Eigen/Dense>

#include "au/au.hh"
#include "au/units/degrees.hh"
#include "au/units/feet.hh"
#include "resim/transforms/geodetic.hh"

namespace resim::transforms {
namespace py = pybind11;

// A simple pybinding of SE3
// TODO(mikebauer) Add frames
PYBIND11_MODULE(geodetic_python, m) {
  py::class_<Geodetic>(m, "Geodetic")
      .def(py::init<>())
      .def(
          "latitude_deg",
          [](const Geodetic &g) { return g.latitude.in(au::degrees); })
      .def(
          "longitude_deg",
          [](const Geodetic &g) { return g.longitude.in(au::degrees); })
      .def(
          "altitude_m",
          [](const Geodetic &g) { return g.altitude.in(au::meters); })
      .def(
          "altitude_ft",
          [](const Geodetic &g) { return g.altitude.in(au::feet); })
      .def(
          "set_latitude_deg",
          [](Geodetic &g, const double x) { g.latitude = au::degrees(x); })
      .def(
          "set_longitude_deg",
          [](Geodetic &g, const double x) { g.longitude = au::degrees(x); })
      .def(
          "set_altitude_m",
          [](Geodetic &g, const double x) { g.altitude = au::meters(x); })
      .def("set_altitude_ft", [](Geodetic &g, const double x) {
        g.altitude = au::feet(x);
      });

  py::class_<GeodeticWithRotation>(m, "GeodeticWithRotation")
      .def_readwrite("geodetic", &GeodeticWithRotation::geodetic)
      .def_readwrite("rotation", &GeodeticWithRotation::rotation);

  m.def(
      "ecef_position_from_geodetic",
      static_cast<Eigen::Vector3d (*)(const Geodetic &)>(
          &ecef_position_from_geodetic));

  m.def("geodetic_from_ecef_position", &geodetic_from_ecef_position);

  m.def(
      "ecef_from_body_from_geodetic_with_rotation",
      &ecef_from_body_from_geodetic_with_rotation);

  m.def(
      "geodetic_with_rotation_from_ecef_from_body",
      &geodetic_with_rotation_from_ecef_from_body);
}

}  // namespace resim::transforms
