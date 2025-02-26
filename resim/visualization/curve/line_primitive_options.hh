#pragma once

#include "resim/visualization/color.hh"

namespace resim::visualization::curve {

// Options to be forwarded to the resulting LinePrimitive
struct LinePrimitiveOptions {
  // See https://foxglove.dev/docs/studio/messages/line-primitive for
  // the meaning of these options
  static constexpr double DEFAULT_THICKNESS = 2.0;
  double thickness = DEFAULT_THICKNESS;
  bool scale_invariant = true;
  Color color{colors::JAVA};
};

struct TrajectoryLinePrimitiveOptions {
  // See https://foxglove.dev/docs/studio/messages/line-primitive for
  // the meaning of these options
  static constexpr double DEFAULT_THICKNESS = 5.0;
  double thickness = DEFAULT_THICKNESS;
  bool scale_invariant = true;
  Color color{colors::FLAMINGO};
};

}  // namespace resim::visualization::curve
