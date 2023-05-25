#pragma once

#include <optional>

#include "resim_core/actor/actor.hh"
#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/experiences/geometry.hh"
#include "resim_core/time/timestamp.hh"

namespace resim::actor {

// An actor which follows a given Trajectory. It spawns at the start of the
// given Trajectory, follows the trajectory, and despawns at its end.
class TrajectoryActor : public Actor {
 public:
  explicit TrajectoryActor(
      ActorId id,
      state::Trajectory trajectory,
      std::optional<experiences::Geometry> geometry = std::nullopt);

  void simulate_forward(time::Timestamp time) override;

  state::ObservableState observable_state() const override;

  Geometry geometry() const override;

  time::Timestamp current_time() const override;

 private:
  bool is_spawned() const;

  time::Timestamp current_time_;
  state::Trajectory trajectory_;

  // This is mutable since it's not really part of the actor's state. For
  // instance, if we want to snapshot and replay a sim from a particular
  // timestamp, we want to preserve the actor's state, but we always want to
  // publish the geometry again.
  // TODO(https://app.asana.com/0/1204498029712344/1204673427233182/f) Remove
  // this once we have a long term geometries strategy.
  mutable bool has_published_geometry_ = false;
  std::optional<experiences::Geometry> geometry_;
};

};  // namespace resim::actor
