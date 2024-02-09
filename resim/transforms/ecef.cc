
#include "resim/transforms/ecef.hh"

#include <cmath>

#include "au/math.hh"
#include "resim/math/newton_solver.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::transforms {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec2 = Eigen::Vector2d;
using Mat2 = Eigen::Matrix2d;

namespace {

// WGS 84 from https://en.wikipedia.org/wiki/World_Geodetic_System#WGS84
constexpr au::QuantityD<au::Meters> SEMI_MAJOR_AXIS{au::meters(6378137.0)};
constexpr double INVERSE_FLATTENING{298.257223563};
constexpr double ECCENTRICITY_SQ{
    (2 - 1 / INVERSE_FLATTENING) / INVERSE_FLATTENING};

SO3 get_geographic_cartesian_rotation(
    const Eigen::Vector3d &ecef_position,
    const au::QuantityD<au::Degrees> latitude,
    const au::QuantityD<au::Degrees> longitude) {
  const double clat = cos(latitude);
  const double slat = sin(latitude);
  const double s2lat = slat * slat;

  const double clong = cos(longitude);
  const double slong = sin(longitude);

  const Vec3 x_axis{Vec3{-ecef_position.y(), ecef_position.x(), 0.0}
                        .normalized()};  // Faces East
  const Vec3 z_axis{Vec3{clat * clong, clat * slong, slat}.normalized()};
  const Vec3 y_axis{z_axis.cross(x_axis)};
  const Mat3 rot_mat{(Mat3() << x_axis, y_axis, z_axis).finished()};
  return SO3{rot_mat};
}

}  // namespace

Eigen::Vector3d ecef_position_from_geodetic(
    const Eigen::Vector3d &geodetic,
    NullableReference<Eigen::Matrix3d> jacobian) {
  const double latitude = geodetic(0);
  const double longitude = geodetic(1);
  const double altitude = geodetic(2);

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

Eigen::Vector3d ecef_position_from_geodetic(const Geodetic &geodetic) {
  return ecef_position_from_geodetic(
      Vec3{
          geodetic.latitude.in(au::radians),
          geodetic.longitude.in(au::radians),
          geodetic.altitude.in(au::meters),
      },
      null_reference<Mat3>);
}

Geodetic geodetic_from_ecef_position(const Eigen::Vector3d &ecef_position) {
  // Guess:
  const double p = ecef_position.head<2>().norm();
  const Vec3 geodetic_guess{
      std::atan2(ecef_position.z(), p),
      std::atan2(ecef_position.y(), ecef_position.x()),
      ecef_position.norm() - SEMI_MAJOR_AXIS.in(au::meters)};

  // Technically, iterating is 3d is unnecessary (can just use p = sqrt(x^2
  // + y^2)), but using the 3d model makes the code more reusable.
  constexpr int MAX_ITERATIONS = 20;
  constexpr double TOLERANCE = 1e-6;  // Microns
  const auto geodetic_sv{math::newton_solve(
      [&](const Vec3 &geodetic, NullableReference<Mat3> jacobian) -> Vec3 {
        return ecef_position_from_geodetic(geodetic, jacobian) - ecef_position;
      },
      geodetic_guess,
      MAX_ITERATIONS,
      TOLERANCE)};

  REASSERT(geodetic_sv.ok(), geodetic_sv.status().what());
  const Vec3 &geodetic{geodetic_sv.value()};

  return Geodetic{
      .latitude = au::radians(geodetic(0)),
      .longitude = au::radians(geodetic(1)),
      .altitude = au::meters(geodetic(2)),
  };
}

SE3 ecef_from_body(const GeodeticWithRotation &geodetic_with_rotation) {
  const Vec3 translation{
      ecef_position_from_geodetic(geodetic_with_rotation.geodetic)};
  const auto latitude = geodetic_with_rotation.geodetic.latitude;
  const auto longitude = geodetic_with_rotation.geodetic.longitude;
  return SE3{
      get_geographic_cartesian_rotation(translation, latitude, longitude) *
          geodetic_with_rotation.rotation,
      translation};
}

GeodeticWithRotation geodetic_with_rotation_from_ecef_from_body(
    const SE3 &ecef_from_body) {
  const Geodetic geodetic{
      geodetic_from_ecef_position(ecef_from_body.translation())};
  return GeodeticWithRotation{
      .geodetic = geodetic,
      .rotation = get_geographic_cartesian_rotation(
                      ecef_from_body.translation(),
                      geodetic.latitude,
                      geodetic.longitude)
                      .inverse() *
                  ecef_from_body.rotation(),
  };
}

}  // namespace resim::transforms
