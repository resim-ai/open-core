
#pragma once

#include <string>
#include <variant>

#include "resim_core/curves/d_curve.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/uuid.hh"

namespace resim::visualization {

// A struct representing a single visualizable piece of data (e.g. a transform,
// curve, box, etc.)
struct ViewPrimitive {
  UUID id;
  std::variant<transforms::SE3, curves::DCurve<transforms::SE3>> payload;
};

}  // namespace resim::visualization
