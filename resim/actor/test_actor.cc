// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/actor/test_actor.hh"

#include <fmt/core.h>

#include "resim/actor/actor_id.hh"
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

void TestActor::set_simulate_forward(
    std::function<void(const time::Timestamp)> sim_forward) {
  simulate_forward_ = std::move(sim_forward);
}

time::Timestamp TestActor::current_time() const {
  return state_.time_of_validity;
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

}  // namespace resim::actor
