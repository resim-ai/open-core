// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/geometry/polygon_utils.hh"

#include "resim/assert/assert.hh"
#include "resim/math/clamp.hh"

// TODO
#include <iostream>

namespace resim::geometry {

namespace {

using Vec2 = Eigen::Vector2d;
using Mat2 = Eigen::Matrix2d;
constexpr int DIM = 2;

double cross(const Vec2 &a, const Vec2 &b) { return a(0) * b(1) - a(1) * b(0); }

bool ray_segment_intersection(
    const Eigen::Vector2d &ray_start,
    const Eigen::Vector2d &ray_dir,
    const Eigen::Vector2d &segment_0,
    const Eigen::Vector2d &segment_1) {
  REASSERT(not ray_dir.isZero());
  REASSERT(not segment_1.isApprox(segment_0));
  const Vec2 segment_dir{segment_1 - segment_0};

  // ray_start + ray_dir * t == segment_0 + segment_dir * s
  const Vec2 ray_perp{-ray_dir(1), ray_dir(0)};
  if (ray_perp.dot(segment_dir) == 0.0) {
    return false;
  }

  const double s =
      -ray_perp.dot(segment_0 - ray_start) / ray_perp.dot(segment_dir);

  if (s < 0. or s > 1.) {
    return false;
  }
  return ray_dir.dot(segment_0 - ray_start + segment_dir * s) >= 0.0;
}

}  // namespace

bool segment_intersection(
    const Eigen::Vector2d &a0,
    const Eigen::Vector2d &a1,
    const Eigen::Vector2d &b0,
    const Eigen::Vector2d &b1) {
  const Vec2 a0a1{a1 - a0};
  const Vec2 b0b1{b1 - b0};
  return cross(a0a1, b0 - a1) * cross(a0a1, b1 - a1) < 0. and
         cross(b0b1, a0 - b1) * cross(b0b1, a1 - b1) < 0.;
}

double point_line_segment_distance(
    const Eigen::Vector2d &point,
    const Eigen::Vector2d &segment_start,
    const Eigen::Vector2d &segment_end) {
  if (segment_start.isApprox(segment_end)) {
    return (point - segment_start).norm();
  }

  // segment(t) = segment_start + (segment_end - segment_start) * t for t in [0,
  // 1]
  const Vec2 dir{segment_end - segment_start};
  double t = dir.dot(point - segment_start) / dir.squaredNorm();
  t = math::clamp(t, 0., 1.);

  return (segment_start + (segment_end - segment_start) * t - point).norm();
}

bool is_self_intersecting(const std::vector<Eigen::Vector2d> &polygon) {
  const size_t num_edges = polygon.size();
  REASSERT(num_edges > 2U, "Polygons must have at least three edges");

  for (size_t ii = 0U; ii < num_edges; ++ii) {
    // The condition here is a bit complex. We want jj to be an edge
    // we have not already checked, and we want it not to be the edge
    // before the ii-th edge which can only happen if we loop around
    // when ii = 0.
    for (size_t jj = (ii + 2U); jj < num_edges and (jj - ii) < num_edges - 1U;
         ++jj) {
      if (segment_intersection(
              polygon.at(ii),
              polygon.at(ii + 1),
              polygon.at(jj),
              polygon.at((jj + 1) % num_edges /* Will wrap around */))) {
        return true;
      }
    }
  }
  return false;
}

bool point_in_polygon(
    const Eigen::Vector2d &point,
    const std::vector<Eigen::Vector2d> &polygon) {
  const size_t num_edges = polygon.size();
  REASSERT(num_edges > 2U, "Polygons must have at least three edges");
  REASSERT(
      not is_self_intersecting(polygon),
      "Can't detect points in non-simple polygon");
  int intersection_count = 0;
  for (size_t edge_start = 0U; edge_start < num_edges; ++edge_start) {
    const size_t edge_end = (edge_start + 1U) % num_edges;
    if (ray_segment_intersection(
            point,
            Vec2::UnitX(),
            polygon.at(edge_start),
            polygon.at(edge_end))) {
      ++intersection_count;
    }
  }
  return bool(intersection_count % 2);  // Odd counts mean we're inside
}

}  // namespace resim::geometry
