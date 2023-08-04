// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

////////////////////////////////////////////////////////////////////////////////
// oscillator.hh                                                              //
////////////////////////////////////////////////////////////////////////////////
//
// This file defines a controlled dynamic system that behaves like a harmonic
// oscillator for testing purposes. The system here is defined like so:
//
//                              x''(t) = u(x, t)
//                             u(x, t) = -kx
//
// Since most integrators work with first-order systems, we describe the system
// state as displacment `x` and velocity `v = x'`. In other words:
//
//
//                       (d/dt)[x v]^T = [v u(x,t)]^T
//                             u(x, t) = -kx
//
// We define OscillatorState to represent [x v]^T, OscillatorDynamics to define
// the first equaiton above, and OscillatorController to represent the second
// equation above.
//
#pragma once

#include <Eigen/Dense>

#include "resim/dynamics/controller.hh"
#include "resim/dynamics/dynamics.hh"

namespace resim::dynamics::testing {

// The state of an oscillator as described above.
struct OscillatorState {
  static constexpr int DOF = 2;
  double displacement = 0.;
  double velocity = 0.;
};

// Add an addition operator so we can add deltas to the
// OscillatorState. delta(0) is the change in displacement and delta(1) is the
// change in velocity.
// @param[in] state - The state we're adding to.
// @param[in] delta - The change to add to our state.
OscillatorState operator+(
    const OscillatorState &state,
    const Eigen::Vector2d &delta);

// The dimension of our control vector, as described above.
constexpr int OSCILLATOR_CONTROL_DIM = 1;

// The dynamics of our oscillator as described above.
class OscillatorDynamics
    : public Dynamics<OscillatorState, OSCILLATOR_CONTROL_DIM> {
 public:
  Delta operator()(
      const State &state,
      const Control &control,
      time::Timestamp time,
      NullableReference<Diffs> jacobian) const override;
};

// Our controller as described above, which is accodringly parameterized by a
// stiffness parameter k.
class OscillatorController
    : public Controller<OscillatorState, OSCILLATOR_CONTROL_DIM> {
 public:
  explicit OscillatorController(double stiffness);

  Control operator()(
      const OscillatorState &state,
      time::Timestamp time,
      NullableReference<Jacobian> jacobian) const override;

 private:
  double stiffness_ = 0.;
};

}  // namespace resim::dynamics::testing
