// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>

#include "au/au.hh"
#include "au/units/degrees.hh"
#include "au/units/meters.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::transforms {

// A struct representing a point in Geodetic coordinates
struct Geodetic {
  au::QuantityD<au::Degrees> latitude;
  au::QuantityD<au::Degrees> longitude;
  au::QuantityD<au::Meters> altitude;
};

// A struct representing a pose in Geodetic coordinates. The rotation is a
// rotation relative to the (x, y, z) = (east, north, up) frame. One must
// accordingly be careful when dealing with headings. For a vehicle frame with
// its x axis forwards, the rotation will *not* be the identity when its heading
// is 0 degrees. The rotation will be (0, 0, M_PI_2) in angle-axis form in such
// a case.
struct GeodeticWithRotation {
  Geodetic geodetic;
  SO3 rotation;
};

// Convert a geodetic point to ecef.
// @param[in] geodetic - The point to convert.
// @returns The ecef coordinates of this point **in meters**.
Eigen::Vector3d ecef_position_from_geodetic(const Geodetic &geodetic);

// Convert a geodetic point to ecef with a Jacobian. Prefer the above function
// to this one because this one is less unit safe.
// @param[in] geodetic - The geodetic point as a vector of:
//                       [lat (radians), long (radians), alt (meters)]
// @param[inout] jacobian - A reference to the jacobian of this function.
//                          Populated if present.
// @returns The ecef coordinates of this point **in meters**.
Eigen::Vector3d ecef_position_from_geodetic(
    const Eigen::Vector3d &geodetic,
    NullableReference<Eigen::Matrix3d> jacobian);

// Iteratively finds the geodetic coordinates for a given ecef position **in
// meters**.
// @param[in] - The ecef position in meters.
// @returns The geodetic coordinates of this point.
// @throws AssertException if the algorithm fails to converge.
Geodetic geodetic_from_ecef_position(const Eigen::Vector3d &ecef_position);

// Convert a geodetic (with rotation) to an ecef_from_body transform.
SE3 ecef_from_body_from_geodetic_with_rotation(
    const GeodeticWithRotation &geodetic_with_rotation);

// Concvert an ecef_from_body SE3 pose to a GeodeticWithRotation.
GeodeticWithRotation geodetic_with_rotation_from_ecef_from_body(
    const SE3 &ecef_from_body);

}  // namespace resim::transforms
