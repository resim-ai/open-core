
#include "resim/planning/drone/dynamics.hh"

namespace resim::planning::drone {

State Dynamics::operator()(
    const State &x,
    const Control &u,
    NullableReference<DynamicsDiffs<State, Control>> diffs) {
  return x;
}

}  // namespace resim::planning::drone
