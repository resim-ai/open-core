// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/color.hh"

namespace resim::visualization {

struct LinePrimitiveOptions {
  // See https://foxglove.dev/docs/studio/messages/line-primitive for
  // the meaning of these options
  static constexpr double DEFAULT_THICKNESS = 2.0;
  double thickness = DEFAULT_THICKNESS;
  bool scale_invariant = true;
  Color color{colors::JAVA};
};

}  // namespace resim::visualization
