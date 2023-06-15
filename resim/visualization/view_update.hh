
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
