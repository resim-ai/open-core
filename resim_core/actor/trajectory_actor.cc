#include "resim_core/actor/trajectory_actor.hh"

#include <utility>

#include "resim_core/actor/geometry.hh"
#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/utils/match.hh"

namespace resim::actor {

TrajectoryActor::TrajectoryActor(
    const ActorId id,
    state::Trajectory trajectory,
    std::optional<experiences::Geometry> geometry)
    : Actor{id},
      trajectory_{std::move(trajectory)},
      geometry_{std::move(geometry)} {}

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

Geometry TrajectoryActor::geometry() const {
  Geometry result{
      .frame = trajectory_.body_frame(),
      .time_of_validity = current_time(),
  };
  const bool spawned = is_spawned();
  if (geometry_.has_value()) {
    if (not has_published_geometry_ and spawned) {
      match(geometry_->model, [&](const geometry::Wireframe &wireframe) {
        result.visible_geometry = wireframe;
        has_published_geometry_ = true;
      });
    } else if (has_published_geometry_ and not spawned) {
      result.visible_geometry = Geometry::Clear{};
      has_published_geometry_ = false;
    }
  }
  // NoUpdate is the default
  return result;
}

time::Timestamp TrajectoryActor::current_time() const { return current_time_; }

bool TrajectoryActor::is_spawned() const {
  return current_time() >= trajectory_.start_time() and
         current_time() <= trajectory_.end_time();
}

}  // namespace resim::actor
