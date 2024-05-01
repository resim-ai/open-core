
#pragma once

#include "resim/planning/drone/control.hh"
#include "resim/planning/drone/state.hh"
#include "resim/planning/dynamics.hh"

namespace resim::planning::drone {

class Dynamics {
 public:
  Dynamics() = default;
  explicit Dynamics(
      const double dt,
      const double gravitational_acceleration_mpss)
      : dt_{dt},
        gravitational_acceleration_mpss_{gravitational_acceleration_mpss} {}

  State operator()(
      const State &x,
      const Control &u,
      NullableReference<DynamicsDiffs<State, Control>> diffs);

 private:
  double dt_;
  double gravitational_acceleration_mpss_;
};

}  // namespace resim::planning::drone
