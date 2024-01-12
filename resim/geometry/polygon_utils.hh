// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <Eigen/Dense>
#include <vector>

namespace resim::geometry {

bool edge_intersection(
    const Eigen::Vector2d &a0,
    const Eigen::Vector2d &a1,
    const Eigen::Vector2d &b0,
    const Eigen::Vector2d &b1);

bool self_intersecting(const std::vector<Eigen::Vector2d> &polygon);

}  // namespace resim::geometry
