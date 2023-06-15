#pragma once

#include <Eigen/Dense>

#include "resim/transforms/se3.hh"

namespace resim::dynamics::rigid_body {

//
// This class describes the dynamic state of a rigid body. It is distinct from
// actor::state::RigidBodyState in that it does not contain second derivatives
// because these are not part of the state that is integrated for a body under
// Newtonian dynamics. This state is designed to be used with
// dynamics::Dynamics, dynamics::Controller, and dynamics::Integrator.
//
struct State {
  // Double the group DOF because we have position and velocity
  static constexpr int DOF = 2 * transforms::SE3::DOF;

  // A vector type representing the difference between two rigid_body::State's
  using Delta = Eigen::Matrix<double, DOF, 1>;

  transforms::SE3 reference_from_body;
  transforms::SE3::TangentVector d_reference_from_body{
      transforms::SE3::TangentVector::Zero()};
};

// Addition operator required for working with the Integrator framework. We
// define this as the right + operator as defined in this document:
// https://drive.google.com/file/d/1UlI1N63o6abyL03VfbYoXu22CcYTdZ6b/view?usp=sharing
State operator+(const State &state, const State::Delta &delta);
State operator+(const State::Delta &delta, const State &state);

// Subtraction operator, defined such that state_a + (state_b - state_a) ==
// state_b. This means that it is the right - operator defined in the document
// mentioned above.
State::Delta operator-(const State &state_a, const State &state_b);

}  // namespace resim::dynamics::rigid_body
