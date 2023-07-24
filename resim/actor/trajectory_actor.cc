#include "resim/actor/trajectory_actor.hh"

#include <utility>

#include "resim/actor/state/observable_state.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/match.hh"

namespace resim::actor {

TrajectoryActor::TrajectoryActor(const ActorId id, state::Trajectory trajectory)
    : Actor{id},
      trajectory_{std::move(trajectory)} {}

void TrajectoryActor::simulate_forward(const time::Timestamp time) {
  current_time_ = time;
}

state::ObservableState TrajectoryActor::observable_state() const {
  state::ObservableState state{
      .id = id(),
      .is_spawned = is_spawned(),
      .time_of_validity = current_time(),
  };
  if (state.is_spawned) {
    state.state = trajectory_.point_at(current_time_);
  }
  return state;
}

time::Timestamp TrajectoryActor::current_time() const { return current_time_; }

bool TrajectoryActor::is_spawned() const {
  return current_time() >= trajectory_.start_time() and
         current_time() <= trajectory_.end_time();
}

}  // namespace resim::actor
