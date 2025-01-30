// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/geometry/oriented_box.hh"
#include "resim/transforms/liegroup_concepts.hh"

namespace resim::geometry {

// This function determines the distance between two oriented boxes
// @param[in] box_1 - The first box.
// @param[in] box_2 - The second box.
// @returns The distance between the boxes.
template <transforms::LieGroupType Group>
double box_box_distance(
    const OrientedBox<Group> &box_1,
    const OrientedBox<Group> &box_2);

}  // namespace resim::geometry
