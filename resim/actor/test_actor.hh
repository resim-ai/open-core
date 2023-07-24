#pragma once

#include <functional>

#include "resim/actor/actor.hh"
#include "resim/actor/actor_id.hh"
#include "resim/actor/state/observable_state.hh"
#include "resim/simulator/standard_frames.hh"
#include "resim/time/timestamp.hh"

namespace resim::actor {

// A trivial actor to use for test purposes it allows users to set the
// observable state it will return when queried and set a callback to be called
// as simulate_forward()
class TestActor : public Actor {
 public:
  explicit TestActor(ActorId id);

  // Set the state that will be returned from observable_state().
  void set_state(const state::ObservableState &state);

  // Set the function that will be called as simulate_forward.
  void set_simulate_forward(
      std::function<void(const time::Timestamp)> sim_forward);

  void simulate_forward(time::Timestamp time) override;

  state::ObservableState observable_state() const override;

  time::Timestamp current_time() const override;

 private:
  state::ObservableState state_;
  std::function<void(const time::Timestamp)> simulate_forward_;
};

// Helper to set up a vector of test actor states
std::vector<state::ObservableState> get_test_actor_states(time::Timestamp time);

state::RigidBodyState<transforms::SE3> make_default_actor_state() {
  return state::RigidBodyState<transforms::SE3>(transforms::SE3::identity(
      simulator::SCENE_FRAME,
      transforms::Frame<transforms::SE3::DIMS>::new_frame()));
}

}  // namespace resim::actor
