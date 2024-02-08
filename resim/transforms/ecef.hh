

#pragma once

#include <Eigen/Dense>

#include "au/au.hh"
#include "au/units/degrees.hh"
#include "au/units/meters.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::transforms {

struct LatLonAlt {
  au::QuantityD<au::Degrees> latitude;
  au::QuantityD<au::Degrees> longitude;
  au::QuantityD<au::Meters> altitude;
};

Eigen::Vector3d ecef_position_from_lat_lon_alt(const LatLonAlt &lat_lon_alt);

Eigen::Vector3d ecef_position_from_lat_lon_alt(
    const Eigen::Vector3d &lat_lon_alt,
    NullableReference<Eigen::Matrix3d> jacobian);

LatLonAlt lat_lon_alt_from_ecef_position(const Eigen::Vector3d &ecef);

struct LatLonAltWithRotation {
  LatLonAlt lat_lon_alt;
  SO3 rotation;
};

SE3 ecef_from_body(const LatLonAltWithRotation &lat_lon_alt_with_rotation);

LatLonAltWithRotation lat_lon_alt_with_rotation_from_ecef_from_body(
    const LatLonAltWithRotation &lat_lon_alt_with_rotation);

}  // namespace resim::transforms
