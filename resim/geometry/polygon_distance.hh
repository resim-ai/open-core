// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <Eigen/Dense>
#include <vector>

namespace resim::geometry {

double polygon_distance(
    const std::vector<Eigen::Vector2d> &polygon_a,
    const std::vector<Eigen::Vector2d> &polygon_b);

}
