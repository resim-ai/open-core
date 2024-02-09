

#pragma once

#include <Eigen/Dense>

#include "au/au.hh"
#include "au/units/degrees.hh"
#include "au/units/meters.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::transforms {

struct Geodetic {
  au::QuantityD<au::Degrees> latitude;
  au::QuantityD<au::Degrees> longitude;
  au::QuantityD<au::Meters> altitude;
};

Eigen::Vector3d ecef_position_from_geodetic(const Geodetic &geodetic);

Eigen::Vector3d ecef_position_from_geodetic(
    const Eigen::Vector3d &geodetic,
    NullableReference<Eigen::Matrix3d> jacobian);

// Iteratively finds the latitude, longitude, and ecef
Geodetic geodetic_from_ecef_position(const Eigen::Vector3d &ecef);

struct GeodeticWithRotation {
  Geodetic geodetic;
  SO3 rotation;
};

SE3 ecef_from_body(const GeodeticWithRotation &geodetic_with_rotation);

GeodeticWithRotation geodetic_with_rotation_from_ecef_from_body(
    const SE3 &ecef_from_body);

}  // namespace resim::transforms
