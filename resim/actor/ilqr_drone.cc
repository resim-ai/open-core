// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/actor/ilqr_drone.hh"

#include <algorithm>
#include <iostream>

#include "resim/actor/state/rigid_body_state.hh"
#include "resim/math/vector_partition.hh"
#include "resim/planning/cost_building_blocks.hh"
#include "resim/planning/cost_function_registry.hh"
#include "resim/planning/drone/dynamics.hh"
#include "resim/simulator/standard_frames.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/nullable_reference.hh"

namespace resim::actor {

namespace {
constexpr double GRAVITATIONAL_ACCELERATION_MPSS = 9.81;
constexpr double PLANNING_DT_S = 0.1;
constexpr size_t PLANNING_STEPS = 40U;
constexpr double REPLANNING_CADENCE_S = 1.0;

using Vec3 = Eigen::Vector3d;
using transforms::SE3;

using CostFunctionRegistry = planning::
    CostFunctionRegistry<planning::drone::State, planning::drone::Control>;

using math::get_block;

// Helper function which sets up the correct cost function registry
// containing the desired control regularization, goal attainment, and
// velocity regularization cost functions.
// @param[in] goal_position - The goal we're navigating to.
// @param[in] velocity_cost - The penalty weight to apply to non-zero velocity.
CostFunctionRegistry make_cost(Vec3 goal_position, const double velocity_cost) {
  CostFunctionRegistry registry;
  using State = planning::drone::State;
  using Control = planning::drone::Control;
  using planning::ComputeDiffs;

  registry["control_regularization"] =
      [](const State &x,
         NullableReference<const Control> u,
         NullableReference<planning::CostDiffs<State, Control>> diffs) {
        constexpr double REGULARIZATION_WEIGHT = 0.1;
        if (not u.has_value()) {
          return 0.0;
        }
        const auto torque_cost = quadratic_cost(
            u->angular_acceleration,
            Eigen::Matrix3d{
                REGULARIZATION_WEIGHT * Eigen::Matrix3d::Identity()},
            diffs.has_value() ? ComputeDiffs::YES : ComputeDiffs::NO);

        if (diffs.has_value()) {
          get_block<Control::Partition, Control::ANGULAR_ACCELERATION>(
              diffs->cost_u) += *torque_cost.dcost_dx;

          get_block<
              Control::Partition,
              Control::ANGULAR_ACCELERATION,
              Control::Partition,
              Control::ANGULAR_ACCELERATION>(diffs->cost_uu) +=
              *torque_cost.d2cost_dx2;
        }

        const auto thrust_cost = quadratic_cost(
            Eigen::Matrix<double, 1, 1>{
                u->thrust - GRAVITATIONAL_ACCELERATION_MPSS},
            Eigen::Matrix<double, 1, 1>{
                REGULARIZATION_WEIGHT *
                Eigen::Matrix<double, 1, 1>::Identity()},
            diffs.has_value() ? ComputeDiffs::YES : ComputeDiffs::NO);

        if (diffs.has_value()) {
          get_block<Control::Partition, Control::THRUST>(diffs->cost_u) +=
              *thrust_cost.dcost_dx;
          get_block<
              Control::Partition,
              Control::THRUST,
              Control::Partition,
              Control::THRUST>(diffs->cost_uu) += *thrust_cost.d2cost_dx2;
        }
        return torque_cost.cost + thrust_cost.cost;
      };

  registry["goal_cost"] =
      [goal = std::move(goal_position)](
          const State &x,
          NullableReference<const Control> u,
          NullableReference<planning::CostDiffs<State, Control>> diffs) {
        constexpr double VERTICAL_WEIGHT = 50.0;
        constexpr double HORIZONTAL_WEIGHT = 10.0;
        constexpr double THRESHOLD = 2.0;
        auto cost_result = soft_abs_cost<3>(
            x.position - goal,
            Vec3{HORIZONTAL_WEIGHT, HORIZONTAL_WEIGHT, VERTICAL_WEIGHT}
                .asDiagonal(),
            THRESHOLD,
            diffs.has_value() ? ComputeDiffs::YES : ComputeDiffs::NO);

        if (diffs.has_value()) {
          get_block<State::Partition, State::POSITION>(diffs->cost_x) +=
              *cost_result.dcost_dx;
          get_block<
              State::Partition,
              State::POSITION,
              State::Partition,
              State::POSITION>(diffs->cost_xx) += *cost_result.d2cost_dx2;
        }

        return cost_result.cost;
      };

  registry["velocity_cost"] =
      [velocity_cost](
          const State &x,
          NullableReference<const Control> u,
          NullableReference<planning::CostDiffs<State, Control>> diffs) {
        auto cost_result = quadratic_cost<3>(
            x.velocity,
            velocity_cost * Eigen::Matrix3d::Identity(),
            diffs.has_value() ? ComputeDiffs::YES : ComputeDiffs::NO);

        if (diffs.has_value()) {
          get_block<State::Partition, State::VELOCITY>(diffs->cost_x) +=
              *cost_result.dcost_dx;
          get_block<
              State::Partition,
              State::VELOCITY,
              State::Partition,
              State::VELOCITY>(diffs->cost_xx) += *cost_result.d2cost_dx2;
        }
        return cost_result.cost;
      };
  return registry;
}

// Helper to create a stationary initial drone state at the given position.
planning::drone::State make_initial_state(Vec3 position) {
  planning::drone::State result;
  result.scene_from_body_rotation = transforms::SO3::identity();
  result.position = std::move(position);
  result.velocity = Vec3::Zero();
  result.angular_velocity = Vec3::Zero();
  return result;
}

// Helper to create a default drone control trajectory (zero torque and thrust
// to counteract gravity).
std::vector<planning::drone::Control> default_control_trajectory() {
  return std::vector<planning::drone::Control>(
      PLANNING_STEPS,
      planning::drone::Control{
          .angular_acceleration = Vec3::Zero(),
          .thrust = GRAVITATIONAL_ACCELERATION_MPSS});
};

// Helper to create a rigid body state based on drone state and control.
state::RigidBodyState<SE3> rigid_body_state_from_drone_state(
    const planning::drone::State &drone_state,
    const planning::drone::Control &control,
    const ActorId &id) {
  state::RigidBodyState<SE3> state;

  // Pose
  SE3 pose{drone_state.scene_from_body_rotation, drone_state.position};
  pose.set_frames(simulator::SCENE_FRAME, transforms::Frame<3>{id});
  state.set_ref_from_body(pose);

  // First derivative
  state.set_body_angular_velocity_radps(drone_state.angular_velocity);
  state.set_body_linear_velocity_mps(
      drone_state.scene_from_body_rotation.inverse() * drone_state.velocity);

  // Second derivative
  state.set_body_angular_acceleration_radpss(control.angular_acceleration);
  state.set_body_linear_acceleration_mpss(
      Vec3::UnitZ() * control.thrust -
      drone_state.scene_from_body_rotation.inverse() * Vec3::UnitZ() *
          GRAVITATIONAL_ACCELERATION_MPSS);
  return state;
}
}  // namespace

ILQRDrone::ILQRDrone(
    ActorId id,
    Vec3 initial_position,
    Vec3 goal_position,
    double velocity_cost)
    : Actor{id},
      state_{make_initial_state(std::move(initial_position))},
      ilqr_{
          PLANNING_STEPS,
          planning::drone::Dynamics{
              PLANNING_DT_S,
              GRAVITATIONAL_ACCELERATION_MPSS},
          make_cost(std::move(goal_position), velocity_cost)},
      control_trajectory_{
          default_control_trajectory(),
          default_control_trajectory()} {}

void ILQRDrone::replan() {
  // If we have planned previously, copy the part of the control trajectory
  // which is still relevant
  if (last_plan_time_.has_value()) {
    auto idx = static_cast<size_t>(std::floor(
        time::as_seconds(current_time_ - *last_plan_time_) / PLANNING_DT_S));
    auto new_start_it = std::next(
        control_trajectory_.current().cbegin(),
        static_cast<int>(idx));
    auto start_of_remaining = std::copy(
        new_start_it,
        control_trajectory_.current().cend(),
        control_trajectory_.mutable_next().begin());

    // Fill the remainder with default values
    for (auto it = start_of_remaining;
         it != control_trajectory_.mutable_next().end();
         ++it) {
      *it = planning::drone::Control{
          .angular_acceleration = Vec3::Zero(),
          .thrust = GRAVITATIONAL_ACCELERATION_MPSS};
    }
    control_trajectory_.swap();
  }
  last_plan_time_ = current_time_;
  constexpr int MAX_ITERATIONS = 100;
  auto result = ilqr_.optimize_controls(
      state_,
      control_trajectory_.current(),
      MAX_ITERATIONS,
      MAX_ITERATIONS);
  control_trajectory_.mutable_next() = std::move(result.controls);
  control_trajectory_.swap();
}

planning::drone::Control ILQRDrone::get_current_control() const {
  if (not last_plan_time_.has_value()) {
    return Control{
        .angular_acceleration = Vec3::Zero(),
        .thrust = GRAVITATIONAL_ACCELERATION_MPSS,
    };
  }
  auto idx = static_cast<size_t>(std::floor(
      time::as_seconds(current_time_ - *last_plan_time_) / PLANNING_DT_S));
  REASSERT(idx < PLANNING_STEPS);
  return control_trajectory_.current().at(idx);
}

void ILQRDrone::simulate_forward(time::Timestamp time) {
  if (not is_spawned_) {
    is_spawned_ = true;
    current_time_ = time;
    return;
  }

  if (not last_plan_time_.has_value() or
      (current_time_ - *last_plan_time_) >
          time::as_duration(REPLANNING_CADENCE_S)) {
    replan();
  }

  const double dt_s = time::as_seconds(time - current_time_);
  const planning::drone::Dynamics dynamics{
      dt_s,
      GRAVITATIONAL_ACCELERATION_MPSS};
  state_ = dynamics(state_, get_current_control(), null_reference);
  current_time_ = time;
}

bool ILQRDrone::is_spawned() const { return is_spawned_; }

state::ObservableState ILQRDrone::observable_state() const {
  state::ObservableState state{
      .id = id(),
      .is_spawned = is_spawned(),
      .time_of_validity = current_time(),
  };
  if (state.is_spawned) {
    state.state =
        rigid_body_state_from_drone_state(state_, get_current_control(), id());
  }
  return state;
};

time::Timestamp ILQRDrone::current_time() const { return current_time_; }

}  // namespace resim::actor
