
#pragma once

#include <Eigen/Dense>

namespace resim::geometry {

// A struct representing a polynomial in 2D. Not necessarily oriented.
struct Polygon {
  std::vector<Eigen::Vector2d> vertices;
};

// Computes the oriented area of a given polygon. I.e. this is negative if the
// polygon has clockwise winding order.
// @param[in] The polygon to compute the area of
// @returns the oriented area of the given polygon.
double compute_area(const Polygon &polygon);

}  // namespace resim::geometry
