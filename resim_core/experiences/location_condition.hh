#pragma once

#include "resim_core/transforms/framed_group.hh"
#include "resim_core/utils/uuid.hh"

namespace resim::experiences {

// A condition that triggers true when the triggering actor reaches the target
// position, within a given tolerance.
struct LocationCondition {
  UUID triggering_actor;
  transforms::FSE3 target_position;
  double tolerance_m{};  // default to 0m tolerance
};

}  // namespace resim::experiences