// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/geometry/boxes_collide.hh"

#include <Eigen/Dense>

#include "resim/assert/assert.hh"
#include "resim/geometry/box_box_distance.hh"
#include "resim/geometry/gjk_algorithm.hh"
#include "resim/transforms/se3.hh"

namespace resim::geometry {

namespace {
using Vec3 = Eigen::Vector3d;
using transforms::LieGroupType;

// This simple helper detects whether the bounding spheres of the
// given bounding boxes collide within the given collision tolerance.
// @param[in] box_1 - The first box to collide.
// @param[in] box_2 - The second box to collide.
// @param[in] collision_tolerance - The threshold to use for determining
//                                  collision.
// @return A boolean saying whether or not the bounding spheres collide.
template <LieGroupType Group>
bool bounding_spheres_collide(
    const OrientedBox<Group> &box_1,
    const OrientedBox<Group> &box_2,
    const double collision_tolerance) {
  const double radius_1 = 0.5 * box_1.extents().norm();
  const double radius_2 = 0.5 * box_2.extents().norm();
  const double center_to_center_distance =
      (box_2.reference_from_box().translation() -
       box_1.reference_from_box().translation())
          .norm();
  return center_to_center_distance < radius_1 + radius_2 + collision_tolerance;
}

}  // namespace

template <LieGroupType Group>
bool boxes_collide(
    const OrientedBox<Group> &box_1,
    const OrientedBox<Group> &box_2,
    const double collision_tolerance) {
  const bool frames_match =
      box_1.reference_from_box().into() == box_2.reference_from_box().into();
  REASSERT(frames_match, "Box frames don't match!");

  if (not bounding_spheres_collide(box_1, box_2, collision_tolerance)) {
    return false;
  }
  return box_box_distance(box_1, box_2) < collision_tolerance;
}

template bool boxes_collide(
    const OrientedBox<transforms::SE3> &box_1,
    const OrientedBox<transforms::SE3> &box_2,
    double collision_tolerance);

}  // namespace resim::geometry
