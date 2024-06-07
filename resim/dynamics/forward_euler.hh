// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/dynamics/dynamics.hh"
#include "resim/dynamics/integrator.hh"
#include "resim/time/timestamp.hh"

namespace resim::dynamics {

//
// This class implements the integrator interface using the forward Euler method
// (https://en.wikipedia.org/wiki/Euler_method). This is a first-order
// explicit integration scheme.
//
template <StateType State>
class ForwardEuler final : public Integrator<State> {
 public:
  using Base = Integrator<State>;
  using CompleteDynamics = typename Base::CompleteDynamics;

  State operator()(
      time::Duration dt,
      const State &state,
      time::Timestamp time,
      const CompleteDynamics &complete_dynamics) override;
};

template <StateType State>
State ForwardEuler<State>::operator()(
    time::Duration dt,
    const State &state,
    time::Timestamp time,
    const CompleteDynamics &complete_dynamics) {
  using Delta = typename Base::Delta;
  const Delta d_state_dt{complete_dynamics(state, time, null_reference)};
  const double dt_s = time::as_seconds(dt);

  // Forward Euler :)
  return state + dt_s * d_state_dt;
}

}  // namespace resim::dynamics
