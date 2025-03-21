// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/geometry/oriented_box.hh"
#include "resim/transforms/liegroup_concepts.hh"

namespace resim::geometry {

// This function determines whether the given two boxes collide as a boolean.
// Due to numerical error, the internal iterative algorithm does not report
// exactly zero when a collision is found, so a non-zero threshold is used to
// determine collision.
// @param[in] box_1 - The first box to collide.
// @param[in] box_2 - The second box to collide.
// @param[in] collision_tolerance - The threshold to use for determining
//                                  collision. This has units of distance which
//                                  match the units in the bounding box
//                                  transformations.
// @returns A boolean saying whether the given boxes collide.
template <transforms::LieGroupType Group>
bool boxes_collide(
    const OrientedBox<Group> &box_1,
    const OrientedBox<Group> &box_2,
    double collision_tolerance = 1e-4);

}  // namespace resim::geometry
