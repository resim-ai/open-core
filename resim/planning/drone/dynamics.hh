// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/planning/drone/control.hh"
#include "resim/planning/drone/state.hh"
#include "resim/planning/dynamics.hh"

namespace resim::planning::drone {

// Dynamics for a simple quadcopter drone. These dynamics do not represent full
// Newtonian dynamics but rather they allow the vertical acceleration due to
// thrust and body angular accelration to be directly commanded as
// controls. Inverse dynamics can be used to compute the torque and thrust force
// required to attain these anyhow, and keeping these dynamics simple is
// important for the performance of our planning algorithm.
class Dynamics {
 public:
  Dynamics(double dt, double gravitational_acceleration_mpss);

  // The dynamics themselves.
  State operator()(
      const State &x,
      const Control &u,
      NullableReference<DynamicsDiffs<State, Control>> diffs) const;

 private:
  double dt_;
  double gravitational_acceleration_mpss_;
};

}  // namespace resim::planning::drone
