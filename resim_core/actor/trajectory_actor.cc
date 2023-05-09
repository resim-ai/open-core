#include "resim_core/actor/trajectory_actor.hh"

#include <utility>

#include "resim_core/actor/geometry.hh"
#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/framed_group.hh"

namespace resim::actor {

TrajectoryActor::TrajectoryActor(const ActorId id, state::Trajectory trajectory)
    : Actor{id},
      trajectory_{std::move(trajectory)} {}

void TrajectoryActor::simulate_forward(const time::Timestamp time) {
  current_time_ = time;
}

state::ObservableState TrajectoryActor::observable_state() const {
  const bool time_in_bounds = current_time() >= trajectory_.start_time() and
                              current_time() <= trajectory_.end_time();
  state::ObservableState state{
      .id = id(),
      .is_spawned = time_in_bounds,
      .time_of_validity = current_time(),
  };
  if (time_in_bounds) {
    state.state = trajectory_.point_at(current_time_);
  }
  return state;
}

Geometry TrajectoryActor::geometry() const {
  return Geometry{
      .frame = trajectory_.body_frame(),
      .time_of_validity = current_time(),
      // TODO(michael) Add geometries to this actor
      .visible_geometry = Geometry::NoUpdate(),
  };
}

time::Timestamp TrajectoryActor::current_time() const { return current_time_; }

}  // namespace resim::actor
