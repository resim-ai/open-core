// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#pragma once

#include <vector>

#include "resim/visualization/view_primitive.hh"

namespace resim::visualization {

// A collection of view primitives to update the current state of the
// resim::view visualization.
struct ViewUpdate {
  std::vector<ViewPrimitive> primitives;
};

}  // namespace resim::visualization
