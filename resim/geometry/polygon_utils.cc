// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/geometry/polygon_utils.hh"

#include "resim/assert/assert.hh"

namespace resim::geometry {

namespace {

using Vec2 = Eigen::Vector2d;
using Mat2 = Eigen::Matrix2d;
constexpr int DIM = 2;

}  // namespace

bool edge_intersection(
    const Eigen::Vector2d &a0,
    const Eigen::Vector2d &a1,
    const Eigen::Vector2d &b0,
    const Eigen::Vector2d &b1) {
  const Mat2 edgemat{(Mat2() << (a1 - a0), -(b1 - b0)).finished()};

  const Eigen::JacobiSVD<Mat2> svd{
      edgemat,
      Eigen::ComputeFullV | Eigen::ComputeFullU};

  // Parallel edges
  if (svd.rank() != DIM) {
    return false;
  }

  const Vec2 intersection{svd.solve(b0 - a0)};

  return !(
      (intersection.array() < 0.0).any() or (intersection.array() > 1.0).any());
}

bool self_intersecting(const std::vector<Eigen::Vector2d> &polygon) {
  const size_t num_edges = polygon.size();
  REASSERT(num_edges > 2U, "Polygons must have at least three edges");

  for (size_t ii = 0U; ii < num_edges; ++ii) {
    // The condition here is a bit complex. We want jj to be an edge
    // we have not already checked, and we want it not to be the edge
    // before the ii-th edge which can only happen if we loop around
    // when ii = 0.
    for (size_t jj = (ii + 2U); jj < num_edges and (jj - ii) < num_edges - 1U;
         ++jj) {
      if (edge_intersection(
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

}  // namespace resim::geometry
