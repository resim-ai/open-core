// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <Eigen/Dense>
#include <vector>

namespace resim::geometry {

// Compute the minimum distance between two polygons.
//
// If either polygon has less than 3 elements, it's treated as a line
// segment or a point. The polygons are expressed as sequences of
// points representing their outer boundaries. Currently, polygon_a
// and polygon_b must be simple and convex if they have more than
// three elements and this is enforced. Orientation is not enforced.
// TODO(michael) support non-convex polynomials.
// @param[in] polygon_a - The first polygon.
// @param[in] polygon_b - The second polygon.
// @returns min ||a - b|| for any (a, b) in polygon_a x polygon_b.
double polygon_distance(
    const std::vector<Eigen::Vector2d> &polygon_a,
    const std::vector<Eigen::Vector2d> &polygon_b);

}  // namespace resim::geometry
