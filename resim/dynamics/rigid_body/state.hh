// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>

#include "resim/math/vector_partition.hh"
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
  enum DeltaBlockIndices : std::size_t {
    POSE = 0,
    VELOCITY,
  };
  using DeltaPartition =
      math::VectorPartition<transforms::SE3::DOF, transforms::SE3::DOF>;

  static constexpr int DOF = math::VectorPartitionDim<DeltaPartition>::value;

  // A vector type representing the difference between two rigid_body::State's
  using Delta = Eigen::Matrix<double, DOF, 1>;

  // Getter for the part of the delta corresponding to the pose (i.e.
  // reference_from_body).
  static Eigen::VectorBlock<Delta, transforms::SE3::DOF> delta_vector_pose_part(
      Delta &delta);

  static Eigen::VectorBlock<const Delta, transforms::SE3::DOF>
  delta_vector_pose_part(const Delta &delta);

  // Getter for the part of the delta corresponding to the velocity (i.e.
  // d_reference_from_body).
  static Eigen::VectorBlock<Delta, transforms::SE3::DOF>
  delta_vector_velocity_part(Delta &delta);

  static Eigen::VectorBlock<const Delta, transforms::SE3::DOF>
  delta_vector_velocity_part(const Delta &delta);

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
