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
#include "resim/transforms/so3.hh"

namespace resim::transforms {
namespace py = pybind11;

// A simple pybinding of Geodetics
// TODO(mikebauer) Add frames
PYBIND11_MODULE(geodetic_python, m) {
  py::module_::import("resim.transforms.python.se3_python");

  py::class_<Geodetic>(m, "Geodetic")
      .def(py::init<>())
      .def(
          py::init([](const double latitude_deg,
                      const double longitude_deg,
                      const double altitude_m) {
            return Geodetic{
                .latitude = au::degrees(latitude_deg),
                .longitude = au::degrees(longitude_deg),
                .altitude = au::meters(altitude_m),
            };
          }),
          py::kw_only(),
          py::arg("latitude_deg"),
          py::arg("longitude_deg"),
          py::arg("altitude_m"))
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
          [](Geodetic &g, const double x) { g.latitude = au::degrees(x); },
          py::arg("latitude_deg"))
      .def(
          "set_longitude_deg",
          [](Geodetic &g, const double x) { g.longitude = au::degrees(x); },
          py::arg("longitude_deg"))
      .def(
          "set_altitude_m",
          [](Geodetic &g, const double x) { g.altitude = au::meters(x); },
          py::arg("altitude_m"))
      .def(
          "set_altitude_ft",
          [](Geodetic &g, const double x) { g.altitude = au::feet(x); },
          py::arg("altitude_ft"));

  py::class_<GeodeticWithRotation>(m, "GeodeticWithRotation")
      .def(py::init<>())
      .def(
          py::init([](const Geodetic &geodetic, const SO3 &rotation) {
            return GeodeticWithRotation{
                .geodetic = geodetic,
                .rotation = rotation,
            };
          }),
          py::kw_only(),
          py::arg("geodetic"),
          py::arg("rotation"))
      .def_readwrite("geodetic", &GeodeticWithRotation::geodetic)
      .def_readwrite("rotation", &GeodeticWithRotation::rotation);

  m.def(
      "ecef_position_from_geodetic",
      static_cast<Eigen::Vector3d (*)(const Geodetic &)>(
          &ecef_position_from_geodetic),
      py::arg("geodetic"));

  m.def(
      "geodetic_from_ecef_position",
      &geodetic_from_ecef_position,
      py::arg("ecef_position"));

  m.def(
      "ecef_from_body_from_geodetic_with_rotation",
      &ecef_from_body_from_geodetic_with_rotation,
      py::arg("geodetic_with_rotation"));

  m.def(
      "geodetic_with_rotation_from_ecef_from_body",
      &geodetic_with_rotation_from_ecef_from_body,
      py::arg("ecef_from_body"));
}

}  // namespace resim::transforms
