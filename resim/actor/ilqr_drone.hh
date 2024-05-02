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

namespace resim::actor {

class ILQRDrone : public Actor {
 public:
  explicit ILQRDrone(
      ActorId id,
      Eigen::Vector3d initial_position,
      Eigen::Vector3d goal_position,
      double velocity_cost);

  void simulate_forward(time::Timestamp time) override;

  state::ObservableState observable_state() const override;

  time::Timestamp current_time() const override { return current_time_; }

 private:
  using State = planning::drone::State;
  using Control = planning::drone::Control;
  bool is_spawned() const;

  void replan(time::Timestamp time);

  bool is_spawned_ = false;
  time::Timestamp current_time_;

  State state_;
  planning::ILQR<State, Control> ilqr_;
  DoubleBuffer<std::vector<Control>> control_trajectory_;
  std::optional<time::Timestamp> last_plan_time_;
};

};  // namespace resim::actor
