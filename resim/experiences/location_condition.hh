// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/transforms/se3.hh"
#include "resim/utils/uuid.hh"

namespace resim::experiences {

// A condition that triggers true when the triggering actor reaches the target
// position, within a given tolerance.
struct LocationCondition {
  UUID triggering_actor;
  transforms::SE3 target_position;
  double tolerance_m{};  // default to 0m tolerance
};

}  // namespace resim::experiences
