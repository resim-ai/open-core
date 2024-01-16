// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <Eigen/Dense>
#include <vector>

namespace resim::geometry {

// Checks whether two line segments intersect.
//
// Intersection occurs if the intersection point of the two lines is
// in the open interval of each segment. For example, segments with
// coincident endpoints do not satisfy this function.
bool segment_intersection(
    const Eigen::Vector2d &a0,
    const Eigen::Vector2d &a1,
    const Eigen::Vector2d &b0,
    const Eigen::Vector2d &b1);

// Get the shortest distance between a point and any point along a
// given line segment.
double point_line_segment_distance(
    const Eigen::Vector2d &point,
    const Eigen::Vector2d &segment_start,
    const Eigen::Vector2d &segment_end);

// Determine whether a polygon is self intersecting.
bool is_self_intersecting(const std::vector<Eigen::Vector2d> &polygon);

// Determine whether a given point is inside the given
// polygon. Behavior is undefined for points on the edge of the
// polygon.
bool point_in_polygon(
    const Eigen::Vector2d &point,
    const std::vector<Eigen::Vector2d> &polygon);

}  // namespace resim::geometry
