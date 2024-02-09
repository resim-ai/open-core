// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/transforms/geodetic.hh"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "au/units/feet.hh"
#include "au/units/meters.hh"
#include "au/units/radians.hh"
#include "resim/math/is_approx.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::transforms {
// NOLINTBEGIN(readability-magic-numbers)

namespace {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

// WGS 84 from https://en.wikipedia.org/wiki/World_Geodetic_System#WGS84
constexpr au::QuantityD<au::Meters> SEMI_MAJOR_AXIS{au::meters(6378137.0)};
constexpr double INVERSE_FLATTENING{298.257223563};
constexpr double ECCENTRICITY_SQ{
    (2 - 1 / INVERSE_FLATTENING) / INVERSE_FLATTENING};

}  // namespace

// Test converting from geodetic with a few known points and edge cases.
TEST(EcefTest, EcefFromGeodetic) {
  // SETUP
  std::vector<Geodetic> test_points = {
      {
          .latitude = au::degrees(90.0),
          .longitude = au::degrees(132.0),
          .altitude = au::meters(0.0),
      },
      {
          .latitude = au::degrees(-90.0),
          .longitude = au::degrees(27.0),
          .altitude = au::meters(0.0),
      },
      {
          .latitude = au::degrees(180.0),  // Wrap around
          .longitude = au::degrees(0.0),
          .altitude = au::meters(0.0),
      },
      {
          .latitude = au::degrees(0.0),  // Wrap around
          .longitude = au::degrees(180.0),
          .altitude = au::meters(0.0),
      },
      {
          .latitude = au::degrees(0.0),  // Wrap around
          .longitude = au::degrees(-180.0),
          .altitude = au::meters(0.0),
      },
      {
          .latitude = au::degrees(34.1375),
          .longitude = au::degrees(-118.125),
          .altitude = au::feet(863.0),
      },
      {
          .latitude = au::degrees(37.4046944444),
          .longitude = au::degrees(-122.07775),
          .altitude = au::feet(105.0),
      },
      {
          .latitude = au::degrees(51.17886192471981),
          .longitude = au::degrees(-1.8261832097530848),
          .altitude = au::feet(330.0),
      },
  };
  // Known results copied from the internet.
  const double minor_axis =
      SEMI_MAJOR_AXIS.in(au::meters)*sqrt(1 - ECCENTRICITY_SQ);
  const std::vector<Vec3> expected_ecef_positions{
      {0.0, 0.0, minor_axis},
      {0.0, 0.0, -minor_axis},
      {-SEMI_MAJOR_AXIS.in(au::meters), 0.0, 0.0},
      {-SEMI_MAJOR_AXIS.in(au::meters), 0.0, 0.0},
      {-SEMI_MAJOR_AXIS.in(au::meters), 0.0, 0.0},
      {-2491299.7367, -4660893.9818, 3559228.4123},
      {-2694042.8748, -4298379.4806, 3853186.4169},
      {4004575.2952, -127680.7061, 4946121.2891},
  };

  for (int ii = 0; ii < test_points.size(); ++ii) {
    // ACTION
    const Vec3 ecef_position{ecef_position_from_geodetic(test_points.at(ii))};

    // VERIFICATION
    constexpr double TOLERANCE =
        1e-10;  // We only know 10 significant digits for our expected result.
    EXPECT_TRUE(math::is_approx(
        ecef_position,
        expected_ecef_positions.at(ii),
        TOLERANCE));
  }
}

// Test converting to geodetic coordinates and back
TEST(EcefTest, TestRoundTrip) {
  // SETUP
  constexpr size_t SEED = 913U;
  std::mt19937 rng{SEED};
  std::uniform_real_distribution<double> dist{
      -SEMI_MAJOR_AXIS.in(au::meters),
      SEMI_MAJOR_AXIS.in(au::meters)};

  std::vector<Vec3> test_ecef_positions;
  constexpr int NUM_RANDOM_POINTS = 10;
  test_ecef_positions.reserve(NUM_RANDOM_POINTS);
  for (int ii = 0; ii < NUM_RANDOM_POINTS; ++ii) {
    test_ecef_positions.push_back(
        testing::random_vector<Eigen::Vector3d>(rng, dist));
  }

  // Edge cases
  test_ecef_positions.emplace_back(Vec3::Zero());  // Center of the earth
  test_ecef_positions.emplace_back(
      Vec3::UnitZ() * SEMI_MAJOR_AXIS.in(au::meters));  // Above the North Pole
  test_ecef_positions.emplace_back(
      -Vec3::UnitX() *
      SEMI_MAJOR_AXIS.in(
          au::meters));  // Pacific Ocean on the international date line

  constexpr double DISTANCE_TO_MOON_M = 384467000.0;
  test_ecef_positions.emplace_back(
      Vec3::Ones().normalized() * DISTANCE_TO_MOON_M);

  for (const auto &test_ecef_position : test_ecef_positions) {
    // ACTION
    const Geodetic geodetic{geodetic_from_ecef_position(test_ecef_position)};

    const Eigen::Vector3d result_position{
        ecef_position_from_geodetic(geodetic)};

    // VERIFICATION
    constexpr double TOLERANCE = 1e-10;
    EXPECT_TRUE(
        math::is_approx(test_ecef_position, result_position, TOLERANCE));
  }
}

// Test the Jacobian for our conversion with finite differences
TEST(EcefTest, TestEcefFromGeodeticJacobian) {
  // SETUP
  constexpr size_t SEED = 92U;
  std::mt19937 rng{SEED};
  std::uniform_real_distribution<double> lat_dist{-M_PI_2, M_PI_2};
  std::uniform_real_distribution<double> long_dist{-M_PI, M_PI};
  std::uniform_real_distribution<double> alt_dist{-100, 100000};

  constexpr int NUM_RANDOM_POINTS = 10;
  std::vector<Eigen::Vector3d> test_points;
  test_points.reserve(NUM_RANDOM_POINTS);
  for (int ii = 0; ii < NUM_RANDOM_POINTS; ++ii) {
    test_points.emplace_back(lat_dist(rng), long_dist(rng), alt_dist(rng));
  };

  // Edge cases
  test_points.emplace_back(M_PI_2, M_PI, 10.0);
  test_points.emplace_back(-M_PI_2, M_PI_2, 20.0);
  test_points.emplace_back(0.0, -M_PI, 30.0);
  test_points.emplace_back(
      0.0,
      0.0,
      -SEMI_MAJOR_AXIS.in(au::meters));  // Center of the Earth

  for (const auto &geodetic : test_points) {
    // ACTION
    Mat3 jacobian;
    const Vec3 ecef_position{
        ecef_position_from_geodetic(geodetic, NullableReference{jacobian})};

    // VERIFICATION
    constexpr double EPSILON = 5e-9;
    Mat3 finite_differenced_jacobian;
    for (int ii = 0; ii < geodetic.size(); ++ii) {
      const double local_eps = EPSILON * std::max(1.0, std::fabs(geodetic(ii)));
      const Vec3 perturbed_geodetic{geodetic + local_eps * Vec3::Unit(ii)};

      finite_differenced_jacobian.col(ii) = (ecef_position_from_geodetic(
                                                 perturbed_geodetic,
                                                 null_reference<Mat3>) -
                                             ecef_position) /
                                            local_eps;
    }
    constexpr double TOLERANCE = 1e-6;
    EXPECT_TRUE(
        math::is_approx(jacobian, finite_differenced_jacobian, TOLERANCE));
  }
}

// Test converting whole poses (with rotation) to geodetic and back.
TEST(EcefTest, TestPoseRoundTrip) {
  // SETUP
  constexpr size_t SEED = 3U;
  std::mt19937 rng{SEED};
  std::uniform_real_distribution<double> lat_dist{-M_PI_2, M_PI_2};
  std::uniform_real_distribution<double> long_dist{-M_PI, M_PI};
  std::uniform_real_distribution<double> alt_dist{-100, 100000};

  constexpr int NUM_RANDOM_POINTS = 10;
  for (int ii = 0; ii < NUM_RANDOM_POINTS; ++ii) {
    const GeodeticWithRotation random_pose{
        Geodetic{
            .latitude = au::radians(lat_dist(rng)),
            .longitude = au::radians(long_dist(rng)),
            .altitude = au::meters(long_dist(rng)),
        },
        SO3::exp(testing::random_vector<SO3::TangentVector>(rng)),
    };

    // ACTION
    const SE3 ecef_from_frame{
        ecef_from_body_from_geodetic_with_rotation(random_pose)};

    // VERIFICATION
    // Verify the properties of the local geographic Cartesian Frame
    // The rotation of the local geogrpahic Cartesian frame
    const SO3 geographic_cartesian_frame_rotation{
        ecef_from_frame.rotation() * random_pose.rotation.inverse()};
    // Check the z axis direction with the latitude
    const Vec3 z_axis{geographic_cartesian_frame_rotation * Vec3::UnitZ()};
    EXPECT_TRUE(math::is_approx(
        std::atan2(z_axis.z(), z_axis.head<2>().norm()),
        random_pose.geodetic.latitude.in(au::radians)));
    // Check the x axis points east
    const Vec3 x_axis{geographic_cartesian_frame_rotation * Vec3::UnitX()};
    EXPECT_TRUE(math::is_approx(
        x_axis.cross(Vec3::UnitZ()),
        Vec3{
            cos(random_pose.geodetic.longitude),
            sin(random_pose.geodetic.longitude),
            0.0}));

    // ACTION
    const GeodeticWithRotation round_tripped{
        geodetic_with_rotation_from_ecef_from_body(ecef_from_frame)};

    // VERIFICATION
    EXPECT_TRUE(math::is_approx(
        round_tripped.geodetic.latitude.in(au::degrees),
        random_pose.geodetic.latitude.in(au::degrees)));

    EXPECT_TRUE(math::is_approx(
        round_tripped.geodetic.longitude.in(au::degrees),
        random_pose.geodetic.longitude.in(au::degrees)));

    constexpr double TOLERANCE = 1e-7;
    EXPECT_TRUE(math::is_approx(
        round_tripped.geodetic.altitude.in(au::meters),
        random_pose.geodetic.altitude.in(au::meters),
        TOLERANCE));
  }
}

// NOLINTEND(readability-magic-numbers)
}  // namespace resim::transforms
