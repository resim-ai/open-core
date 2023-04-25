#include "resim_core/actor/test_actor.hh"

#include "resim_core/assert/assert.hh"

namespace resim::actor {
void TestActor::set_state(const state::ObservableState &state) {
  state_ = state;
}

void TestActor::simulate_forward(const time::Timestamp time) {
  simulate_forward_(time);
  REASSERT(time == current_time());
};

state::ObservableState TestActor::observable_state() const { return state_; }

void TestActor::set_simulate_forward(
    std::function<void(const time::Timestamp)> sim_forward) {
  simulate_forward_ = std::move(sim_forward);
}

time::Timestamp TestActor::current_time() const {
  return state_.time_of_validity;
}

}  // namespace resim::actor
