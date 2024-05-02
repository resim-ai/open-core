// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/actor/ilqr_drone.hh"

#include <glog/logging.h>

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
constexpr size_t PLANNING_STEPS = 40u;
constexpr double REPLANNING_CADENCE_S = 1.0;

using CostFunctionRegistry = planning::
    CostFunctionRegistry<planning::drone::State, planning::drone::Control>;

using math::get_block;

CostFunctionRegistry make_cost(
    Eigen::Vector3d goal_position,
    const double velocity_cost) {
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
        constexpr double WEIGHT = 10.0;
        constexpr double THRESHOLD = 2.0;
        auto cost_result = soft_abs_cost<3>(
            x.position - goal,
            WEIGHT * Eigen::Matrix3d::Identity(),
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

planning::drone::State make_initial_state(Eigen::Vector3d position) {
  planning::drone::State result;
  result.scene_from_body_rotation = transforms::SO3::identity();
  result.position = std::move(position);
  result.velocity = Eigen::Vector3d::Zero();
  result.angular_velocity = Eigen::Vector3d::Zero();
  return result;
}

std::vector<planning::drone::Control> default_control_trajectory() {
  return std::vector<planning::drone::Control>(
      PLANNING_STEPS,
      planning::drone::Control{
          .angular_acceleration = Eigen::Vector3d::Zero(),
          .thrust = GRAVITATIONAL_ACCELERATION_MPSS});
};

}  // namespace

ILQRDrone::ILQRDrone(
    ActorId id,
    Eigen::Vector3d initial_position,
    Eigen::Vector3d goal_position,
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

void ILQRDrone::replan(const time::Timestamp time) {
  // If we have planned previously, copy the part of the control trajectory
  // which is still relevant
  if (last_plan_time_.has_value()) {
    size_t idx = static_cast<size_t>(
        std::floor(time::as_seconds(time - *last_plan_time_) / PLANNING_DT_S));
    auto new_start_it = std::next(control_trajectory_.current().cbegin(), idx);
    auto start_of_remaining = std::copy(
        new_start_it,
        control_trajectory_.current().cend(),
        control_trajectory_.mutable_next().begin());

    // Fill the remainder with default values
    for (auto it = start_of_remaining;
         it != control_trajectory_.mutable_next().end();
         ++it) {
      *it = planning::drone::Control{
          .angular_acceleration = Eigen::Vector3d::Zero(),
          .thrust = GRAVITATIONAL_ACCELERATION_MPSS};
    }
    control_trajectory_.swap();
  }
  last_plan_time_ = time;
  constexpr int MAX_ITERATIONS = 100;
  auto result = ilqr_.optimize_controls(
      state_,
      control_trajectory_.current(),
      MAX_ITERATIONS,
      MAX_ITERATIONS);
  LOG_IF(WARNING, not result.converged)
      << "Failed planner convergence in iLQR drone actor!";
  control_trajectory_.mutable_next() = std::move(result.controls);
  control_trajectory_.swap();
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
    replan(current_time_);
  }

  size_t idx = static_cast<size_t>(std::floor(
      time::as_seconds(current_time_ - *last_plan_time_) / PLANNING_DT_S));
  REASSERT(idx < PLANNING_STEPS);
  const double dt_s = time::as_seconds(time - current_time_);
  current_time_ = time;
  const planning::drone::Dynamics dynamics{
      dt_s,
      GRAVITATIONAL_ACCELERATION_MPSS};
  state_ =
      dynamics(state_, control_trajectory_.current().at(idx), null_reference);
}

bool ILQRDrone::is_spawned() const { return is_spawned_; }

state::ObservableState ILQRDrone::observable_state() const {
  state::ObservableState state{
      .id = id(),
      .is_spawned = is_spawned(),
      .time_of_validity = current_time(),
  };

  transforms::SE3 pose{state_.scene_from_body_rotation, state_.position};
  pose.set_frames(simulator::SCENE_FRAME, transforms::Frame<3>{id()});
  if (state.is_spawned) {
    state.state = state::RigidBodyState<transforms::SE3>{pose};
  }
  return state;
};

}  // namespace resim::actor
