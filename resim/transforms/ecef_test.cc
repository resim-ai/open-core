
#include "resim/transforms/ecef.hh"

#include <gtest/gtest.h>

#include <iomanip>

#include "au/units/feet.hh"

namespace resim::transforms {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

TEST(EcefTest, TestLatLonAltFromEcefPosition) {
  LatLonAlt lat_lon_alt{
      .latitude = au::degrees(37.4046944444),
      .longitude = au::degrees(-122.07775),
      .altitude = au::feet(1000000.0),
  };
  std::cout << std::setprecision(16) << std::endl;

  Vec3 position{ecef_position_from_lat_lon_alt(lat_lon_alt)};

  //  std::cout << position << std::endl;

  LatLonAlt round_tripped{lat_lon_alt_from_ecef_position(position)};
  //  std::cout << round_tripped.latitude.in(au::degrees) << std::endl;

  LatLonAltWithRotation x;
  x.lat_lon_alt = lat_lon_alt;
  ecef_from_body(x);
}

TEST(EcefTest, TestLatLonAltFromEcefPositionJacobian) {
  std::vector<Eigen::Vector3d> test_points{
      Vec3{37.4046944444, -122.07775, 30.0},
      Vec3{M_PI_2, M_PI, 10000.0},
  };

  for (const auto &lat_lon_alt : test_points) {
    Mat3 jacobian;
    const Vec3 ecef_position{ecef_position_from_lat_lon_alt(
        lat_lon_alt,
        NullableReference{jacobian})};

    constexpr double EPSILON = 5e-9;
    Mat3 finite_differenced_jacobian;
    for (int ii = 0; ii < lat_lon_alt.size(); ++ii) {
      const double local_eps =
          EPSILON * std::max(1.0, std::fabs(lat_lon_alt(ii)));
      const Vec3 perturbed_lat_lon_alt{
          lat_lon_alt + local_eps * Vec3::Unit(ii)};

      finite_differenced_jacobian.col(ii) = (ecef_position_from_lat_lon_alt(
                                                 perturbed_lat_lon_alt,
                                                 null_reference<Mat3>) -
                                             ecef_position) /
                                            local_eps;
    }
    //    std::cout <<
    //    "-----------------------------------------------------------"
    //                 "---------------------"
    //              << std::endl;
    //    std::cout << jacobian << std::endl << std::endl;
    //    std::cout << finite_differenced_jacobian << std::endl << std::endl;
    //
    //    std::cout << (jacobian - finite_differenced_jacobian).array() /
    //                     jacobian.array().abs().cwiseMax(1.)
    //              << std::endl;
  }
}

}  // namespace resim::transforms
