//
// gjk_distance_subalgorithm.hh
//
// This file defines the distance subalgorithm as used by the GJK algorithm. In
// particular, this is the distance subroutine described by Section V of the
// following paper:
// https://drive.google.com/file/d/1A8j4b-Wknf6eAz8zh4YMKNLowfTuxXWD/view
//
#pragma once

#include <Eigen/Dense>
#include <vector>

namespace resim::geometry::convex_helpers {

// A representation of a simplex
template <int DIM>
using Simplex = std::vector<Eigen::Matrix<double, DIM, 1U>>;

// The result of the distance subalgorithm containing the closest point to the
// origin in a simplex and the smallest subsimplex containing it.
template <int DIM>
struct DistanceResult {
  Eigen::Matrix<double, DIM, 1> closest_point{
      Eigen::Matrix<double, DIM, 1U>::Zero()};
  Simplex<DIM> simplex;
};

namespace testing {
// Make it possible to optionally force the backup procedure to be used in order
// to make sure it has test coverage.
enum class Algorithm {
  MAIN = 0,
  BACKUP,
};
}  // namespace testing

// The distance subalgorithm.
// @param[in] simplex - The simplex to find the closest point to the origin
//                      from.
// @param[in] force_backup - Force the employment of the backup procedure for
//                           testing/debugging purposes.
// @returns - The closest point in the given simplex to the origin in a simplex
//            and the smallest subsimplex containing it.
template <int DIM>
DistanceResult<DIM> distance_subalgorithm(
    const Simplex<DIM> &simplex,
    testing::Algorithm force_backup = testing::Algorithm::MAIN);

}  // namespace resim::geometry::convex_helpers
