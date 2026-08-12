// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <optional>

#include "resim/actor/actor.hh"
#include "resim/actor/actor_id.hh"
#include "resim/planning/drone/control.hh"
#include "resim/planning/drone/state.hh"
#include "resim/planning/ilqr.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/double_buffer.hh"
#include "state/observable_state.hh"

namespace resim::actor {

// This actor spawns immediately at the start of the simulation and
// uses model predictive control with a simple vertical thrust +
// torque drone model to navigate from its given initial position to
// the given goal position.
class ILQRDrone : public Actor {
 public:
  // Constructor
  // @param[in] id - The id for this actor.
  // @param[in] initial_position - The place to spawn the actor with no
  //                               velocity.
  // @param[in] goal_position - The place the actor is trying to get to.
  // @param[in] velocity_cost - The weight for a quadratic cost on translational
  //                            velocity.
  explicit ILQRDrone(
      ActorId id,
      Eigen::Vector3d initial_position,
      Eigen::Vector3d goal_position,
      double velocity_cost);

  // Simulate the actor forward to the given time.
  void simulate_forward(time::Timestamp time) override;

  // Get the current actor state
  state::ObservableState observable_state() const override;

  // Get the current time
  time::Timestamp current_time() const override;

  void observe_states(
      const std::vector<state::ObservableState> &states) override;

 private:
  using State = planning::drone::State;
  using Control = planning::drone::Control;

  // Getter for whether we are spawned.
  bool is_spawned() const;

  // Helper to re-generate the plan starting at the current time
  void replan();

  // Helper to get the current control based on the control buffer and
  // the last replanning time.
  Control get_current_control() const;

  bool is_spawned_ = false;
  time::Timestamp current_time_;

  State state_;
  std::vector<state::ObservableState> avoidance_states_;
  planning::ILQR<State, Control> ilqr_;
  std::vector<State> state_trajectory_;
  DoubleBuffer<std::vector<Control>> control_trajectory_;
  std::optional<time::Timestamp> last_plan_time_;
};

};  // namespace resim::actor
