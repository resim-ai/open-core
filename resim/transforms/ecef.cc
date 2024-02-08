
#include "resim/transforms/ecef.hh"

#include <cmath>
#include <iostream>  // TODO

#include "au/math.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::transforms {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec2 = Eigen::Vector2d;
using Mat2 = Eigen::Matrix2d;

// WGS 84 from https://en.wikipedia.org/wiki/World_Geodetic_System#WGS84
constexpr au::QuantityD<au::Meters> SEMI_MAJOR_AXIS{au::meters(6378137.0)};
// constexpr double INVERSE_FLATTENING{298.257223563 };
constexpr double INVERSE_FLATTENING{298.257222100882711243};
constexpr double ECCENTRICITY_SQ{
    (2 - 1 / INVERSE_FLATTENING) / INVERSE_FLATTENING};
constexpr double ONE_MINUS_FLATTENING_SQUARED =
    (1 - 1. / INVERSE_FLATTENING) * (1 - 1. / INVERSE_FLATTENING);

Eigen::Vector3d ecef_position_from_lat_lon_alt(
    const Eigen::Vector3d &lat_lon_alt,
    NullableReference<Eigen::Matrix3d> jacobian) {
  const double latitude = lat_lon_alt(0);
  const double longitude = lat_lon_alt(1);
  const double altitude = lat_lon_alt(2);

  const double clat = cos(latitude);
  const double slat = sin(latitude);
  const double s2lat = slat * slat;

  const double clong = cos(longitude);
  const double slong = sin(longitude);

  const double prime_vertical_radius{
      (SEMI_MAJOR_AXIS.in(au::meters) / sqrt(1 - ECCENTRICITY_SQ * s2lat))};

  const double x = (prime_vertical_radius + altitude) * clat * clong;
  const double y = (prime_vertical_radius + altitude) * clat * slong;
  const double z =
      ((1. - ECCENTRICITY_SQ) * prime_vertical_radius + altitude) * slat;

  if (jacobian.has_value()) {
    const double dprime_vertical_radius_dlatitude{
        prime_vertical_radius * ECCENTRICITY_SQ * slat * clat /
        (1 - ECCENTRICITY_SQ * s2lat)};

    (*jacobian)(0, 0) = dprime_vertical_radius_dlatitude * clat * clong -
                        (prime_vertical_radius + altitude) * slat * clong;
    (*jacobian)(1, 0) = dprime_vertical_radius_dlatitude * clat * slong -
                        (prime_vertical_radius + altitude) * slat * slong;
    (*jacobian)(2, 0) =
        (1 - ECCENTRICITY_SQ) * dprime_vertical_radius_dlatitude * slat +
        ((1 - ECCENTRICITY_SQ) * prime_vertical_radius + altitude) * clat;

    (*jacobian)(0, 1) = -(prime_vertical_radius + altitude) * clat * slong;
    (*jacobian)(1, 1) = (prime_vertical_radius + altitude) * clat * clong;
    (*jacobian)(2, 1) = 0.0;

    (*jacobian)(0, 2) = clat * clong;
    (*jacobian)(1, 2) = clat * slong;
    (*jacobian)(2, 2) = slat;
  }

  return Eigen::Vector3d{
      x,
      y,
      z,
  };
}

Eigen::Vector3d ecef_position_from_lat_lon_alt(const LatLonAlt &lat_lon_alt) {
  return ecef_position_from_lat_lon_alt(
      Vec3{
          lat_lon_alt.latitude.in(au::radians),
          lat_lon_alt.longitude.in(au::radians),
          lat_lon_alt.altitude.in(au::meters),
      },
      null_reference<Mat3>);
}

LatLonAlt lat_lon_alt_from_ecef_position(const Eigen::Vector3d &ecef_position) {
  // Guess:
  const double p = ecef_position.head<2>().norm();
  Vec3 lat_lon_alt{
      std::atan2(ecef_position.z(), p),
      std::atan2(ecef_position.y(), ecef_position.x()),
      ecef_position.norm() - SEMI_MAJOR_AXIS.in(au::meters)};

  // Technically, iterating is 3d is unnecessary (can just use p = sqrt(x^2 +
  // y^2)), but using the 3d model makes the code more reusable.
  bool converged = false;
  Mat3 jacobian;
  while (not converged) {
    const Vec3 error{
        ecef_position_from_lat_lon_alt(
            lat_lon_alt,
            NullableReference{jacobian}) -
        ecef_position};
    if (error.norm() < 1e-6) {
      converged = true;
    }
    lat_lon_alt = lat_lon_alt - jacobian.inverse() * error;
  }
  return LatLonAlt{
      .latitude = au::radians(lat_lon_alt(0)),
      .longitude = au::radians(lat_lon_alt(1)),
      .altitude = au::meters(lat_lon_alt(2)),
  };
}

SE3 ecef_from_body(const LatLonAltWithRotation &lat_lon_alt_with_rotation) {
  const Vec3 translation{
      ecef_position_from_lat_lon_alt(lat_lon_alt_with_rotation.lat_lon_alt)};

  // Build axes for our geographic Cartesian coordinates
  const double latitude =
      lat_lon_alt_with_rotation.lat_lon_alt.latitude.in(au::radians);
  const double longitude =
      lat_lon_alt_with_rotation.lat_lon_alt.longitude.in(au::radians);
  const double altitude =
      lat_lon_alt_with_rotation.lat_lon_alt.altitude.in(au::meters);

  const double clat = cos(latitude);
  const double slat = sin(latitude);
  const double s2lat = slat * slat;

  const double clong = cos(longitude);
  const double slong = sin(longitude);

  const Vec3 x_axis{
      Vec3{-translation.y(), translation.x(), 0.0}.normalized()};  // Faces East
  const Vec3 z_axis{Vec3{clat * clong, clat * slong, slat}.normalized()};
  const Vec3 y_axis{z_axis.cross(x_axis)};
  const Mat3 rot_mat{(Mat3() << x_axis, y_axis, z_axis).finished()};
  return SE3{SO3{rot_mat} * lat_lon_alt_with_rotation.rotation, translation};
}

LatLonAltWithRotation lat_lon_alt_with_rotation_from_ecef_from_body(
    const LatLonAltWithRotation &lat_lon_alt_with_rotation) {}

}  // namespace resim::transforms
