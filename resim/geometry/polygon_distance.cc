// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/geometry/polygon_distance.hh"

#include "resim/assert/assert.hh"
#include "resim/geometry/gjk_algorithm.hh"
#include "resim/geometry/polygon_utils.hh"

namespace resim::geometry {

namespace {
using Vec2 = Eigen::Vector2d;

// TODO(michael) Handle non-convex polygons
void assert_convex(const std::vector<Eigen::Vector2d> &polygon) {
  double sign = 0.;
  const size_t num_vertices = polygon.size();
  REASSERT(num_vertices > 2U, "Polygons must have at least three vertices.");

  for (size_t ii = 0; ii < num_vertices; ++ii) {
    const Vec2 prev_edge{polygon.at(ii) - polygon.at((ii - 1U) % num_vertices)};
    const Vec2 next_edge{polygon.at((ii + 1U) % num_vertices) - polygon.at(ii)};
    const double cross =
        prev_edge(0) * next_edge(1) - prev_edge(1) * next_edge(0);
    if (cross == 0.0) {
      continue;
    }
    const double cross_sign = cross > 0.0 ? 1.0 : -1.0;
    if (sign == 0.) {
      sign = cross_sign;
    } else {
      REASSERT(sign == cross_sign, "Polygon is non-convex!");
    }
  }
}

}  // namespace

double polygon_distance(
    const std::vector<Eigen::Vector2d> &polygon_a,
    const std::vector<Eigen::Vector2d> &polygon_b) {
  if (polygon_a.size() > 2U) {
    REASSERT(not self_intersecting(polygon_a), "Self intersection detected!");
    assert_convex(polygon_a);
  }
  if (polygon_b.size() > 2U) {
    REASSERT(not self_intersecting(polygon_b), "Self intersection detected!");
    assert_convex(polygon_b);
  }

  const auto support_a = [&polygon_a](const Vec2 &v) {
    return *std::max_element(
        polygon_a.cbegin(),
        polygon_a.cend(),
        [&v](const Vec2 &a, const Vec2 &b) { return a.dot(v) < b.dot(v); });
  };
  const auto support_b = [&polygon_b](const Vec2 &v) {
    return *std::max_element(
        polygon_b.cbegin(),
        polygon_b.cend(),
        [&v](const Vec2 &a, const Vec2 &b) { return a.dot(v) < b.dot(v); });
  };

  constexpr int DIM = 2;
  const auto maybe_distance = gjk_algorithm<DIM>(support_a, support_b);

  REASSERT(maybe_distance.has_value());
  return *maybe_distance;
}

}  // namespace resim::geometry
