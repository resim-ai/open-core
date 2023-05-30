#pragma once

#include <functional>

#include "resim_core/actor/actor.hh"
#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/geometry.hh"
#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/simulator/standard_frames.hh"
#include "resim_core/time/timestamp.hh"

namespace resim::actor {

// A trivial actor to use for test purposes it allows users to set the
// observable state it will return when queried and set a callback to be called
// as simulate_forward()
class TestActor : public Actor {
 public:
  explicit TestActor(ActorId id);

  // Set the state that will be returned from observable_state().
  void set_state(const state::ObservableState &state);

  // Set the geometry of this actor that will be returned from geometry().
  void set_geometry(const Geometry &geometry);

  // Set the function that will be called as simulate_forward.
  void set_simulate_forward(
      std::function<void(const time::Timestamp)> sim_forward);

  void simulate_forward(time::Timestamp time) override;

  state::ObservableState observable_state() const override;

  Geometry geometry() const override;

  time::Timestamp current_time() const override;

 private:
  state::ObservableState state_;
  Geometry geometry_;
  std::function<void(const time::Timestamp)> simulate_forward_;
};

// Helper to set up a vector of test actor states and geometries
std::vector<std::pair<state::ObservableState, Geometry>>
get_test_actor_components(time::Timestamp time);

// Helper to set up a vector of test actor states
std::vector<state::ObservableState> get_test_actor_states(time::Timestamp time);

// Helper to set up a vector of test actor geometries
std::vector<Geometry> get_test_actor_geometries(
    time::Timestamp time,
    bool inconsistent_times = false);

state::RigidBodyState<transforms::FSE3> make_default_actor_state() {
  return state::RigidBodyState<transforms::FSE3>(transforms::FSE3(
      transforms::SE3::identity(),
      simulator::SCENE_FRAME,
      transforms::Frame<transforms::FSE3::DIMS>::new_frame()));
}

}  // namespace resim::actor
