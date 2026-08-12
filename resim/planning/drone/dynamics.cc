// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/planning/drone/dynamics.hh"

#include "resim/math/vector_partition.hh"

namespace resim::planning::drone {

using transforms::SO3;
using Vec3 = Eigen::Vector3d;
using math::get_block;

Dynamics::Dynamics(
    const double dt,
    const double gravitational_acceleration_mpss)
    : dt_{dt},
      gravitational_acceleration_mpss_{gravitational_acceleration_mpss} {}

State Dynamics::operator()(
    const State &x,
    const Control &u,
    NullableReference<DynamicsDiffs<State, Control>> diffs) const {
  State::Vec dx;

  get_block<State::Partition, State::ROTATION>(dx) = x.angular_velocity;
  get_block<State::Partition, State::POSITION>(dx) = x.velocity;
  get_block<State::Partition, State::ANGULAR_VELOCITY>(dx) =
      u.angular_acceleration;
  get_block<State::Partition, State::VELOCITY>(dx) =
      x.scene_from_body_rotation * Vec3::UnitZ() * u.thrust -
      Vec3::UnitZ() * gravitational_acceleration_mpss_;
  get_block<State::Partition, State::TIME>(dx).x() = 1.0;
  dx = dx * dt_;

  if (diffs) {
    diffs->f_x.setIdentity();
    diffs->f_u.setZero();

    get_block<
        State::Partition,
        State::ROTATION,
        State::Partition,
        State::ROTATION>(diffs->f_x) =
        SO3::exp(-dt_ * x.angular_velocity).adjoint();

    get_block<
        State::Partition,
        State::ROTATION,
        State::Partition,
        State::ANGULAR_VELOCITY>(diffs->f_x) =
        SO3::exp_diff(-dt_ * x.angular_velocity) * dt_;

    get_block<
        State::Partition,
        State::POSITION,
        State::Partition,
        State::VELOCITY>(diffs->f_x) = Eigen::Matrix3d::Identity() * dt_;

    // Dependence of acceleration/thrust on rotation:
    // accel = R * [0 0 t]^T - [0 0 g]^T
    // The first term is the relevant one here:
    // d/dR[ accel ] = d/dR[ R v ]
    //               = d/de[ R ex v ]   # Right tangent space derivative
    //               = d/de[ -R vx e ]  # Antisymmetry of cross product
    //               = -R vx
    // where ex and vx are the cross matrices of e and v (i.e. the matrix ex
    // such that (e x v) == ex v). This is also vx = SO3::adjoint(v)
    get_block<
        State::Partition,
        State::VELOCITY,
        State::Partition,
        State::ROTATION>(diffs->f_x) =
        x.scene_from_body_rotation.rotation_matrix() *
        SO3::adjoint(-dt_ * u.thrust * Vec3::UnitZ());

    get_block<
        State::Partition,
        State::ANGULAR_VELOCITY,
        Control::Partition,
        Control::ANGULAR_ACCELERATION>(diffs->f_u) =
        dt_ * Eigen::Matrix3d::Identity();

    get_block<
        State::Partition,
        State::VELOCITY,
        Control::Partition,
        Control::THRUST>(diffs->f_u) =
        x.scene_from_body_rotation * Vec3::UnitZ() * dt_;
  }

  return x + dx;
}

}  // namespace resim::planning::drone
