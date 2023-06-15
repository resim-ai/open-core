#include "resim/actor/test_actor.hh"

#include <fmt/core.h>

#include "resim/actor/actor_id.hh"
#include "resim/actor/geometry.hh"
#include "resim/assert/assert.hh"
#include "resim/transforms/liegroup_test_helpers.hh"
#include "resim/transforms/se3.hh"

namespace resim::actor {
namespace {
using Frame = transforms::Frame<transforms::SE3::DIMS>;
}
TestActor::TestActor(const ActorId id) : Actor{id} {}

void TestActor::set_state(const state::ObservableState &state) {
  state_ = state;
}

void TestActor::set_geometry(const Geometry &geometry) { geometry_ = geometry; }

void TestActor::simulate_forward(const time::Timestamp time) {
  simulate_forward_(time);
  REASSERT(
      time == current_time(),
      fmt::format(
          "Time mismatch: {} vs {}",
          time.time_since_epoch().count(),
          current_time().time_since_epoch().count()));
};

state::ObservableState TestActor::observable_state() const { return state_; }

Geometry TestActor::geometry() const { return geometry_; }

void TestActor::set_simulate_forward(
    std::function<void(const time::Timestamp)> sim_forward) {
  simulate_forward_ = std::move(sim_forward);
}

time::Timestamp TestActor::current_time() const {
  return state_.time_of_validity;
}

std::vector<std::pair<state::ObservableState, Geometry>>
get_test_actor_components(const time::Timestamp time) {
  std::vector<transforms::SE3> test_poses{
      transforms::make_test_group_elements<transforms::SE3>()};

  std::vector<std::pair<state::ObservableState, Geometry>> test_components;
  test_components.reserve(test_poses.size());
  for (auto &pose : test_poses) {
    // Make sure the pose is relative to scene
    pose.set_frames(simulator::SCENE_FRAME, Frame::new_frame());
    test_components.emplace_back(
        state::ObservableState{
            .id = UUID::new_uuid(),
            .is_spawned = true,
            .time_of_validity = time,
            .state = state::RigidBodyState<transforms::SE3>{pose},
        },
        Geometry{
            .frame = pose.from(),
            .time_of_validity = time,
            .visible_geometry = Geometry::Clear{}});
  }

  return test_components;
}

std::vector<state::ObservableState> get_test_actor_states(
    const time::Timestamp time) {
  std::vector<transforms::SE3> test_poses{
      transforms::make_test_group_elements<transforms::SE3>()};

  std::vector<state::ObservableState> test_states;
  test_states.reserve(test_poses.size());
  for (auto &pose : test_poses) {
    // Make sure the pose is relative to scene
    pose.set_frames(simulator::SCENE_FRAME, Frame::new_frame());
    test_states.push_back(state::ObservableState{
        .id = UUID::new_uuid(),
        .is_spawned = true,
        .time_of_validity = time,
        .state = state::RigidBodyState<transforms::SE3>{pose},
    });
  }

  return test_states;
}

std::vector<Geometry> get_test_actor_geometries(
    const time::Timestamp time,
    bool inconsistent_times) {
  std::vector<Geometry> geometries;
  constexpr int NUM_GEOMETRIES = 5;
  geometries.reserve(NUM_GEOMETRIES);
  for (int ii = 0; ii < NUM_GEOMETRIES; ++ii) {
    geometries.push_back(Geometry{
        .frame = Frame::new_frame(),
        .time_of_validity =
            inconsistent_times ? (time + std::chrono::nanoseconds(ii)) : time,
        .visible_geometry = Geometry::Clear{},
    });
  }
  return geometries;
}

}  // namespace resim::actor
