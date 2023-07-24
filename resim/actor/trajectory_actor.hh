#pragma once

#include <optional>

#include "resim/actor/actor.hh"
#include "resim/actor/actor_id.hh"
#include "resim/actor/state/trajectory.hh"
#include "resim/time/timestamp.hh"

namespace resim::actor {

// An actor which follows a given Trajectory. It spawns at the start of the
// given Trajectory, follows the trajectory, and despawns at its end.
class TrajectoryActor : public Actor {
 public:
  explicit TrajectoryActor(ActorId id, state::Trajectory trajectory);

  void simulate_forward(time::Timestamp time) override;

  state::ObservableState observable_state() const override;

  time::Timestamp current_time() const override;

 private:
  bool is_spawned() const;

  time::Timestamp current_time_;
  state::Trajectory trajectory_;
};

};  // namespace resim::actor
