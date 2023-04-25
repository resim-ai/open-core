
#include "resim_core/actor/actor_unit.hh"

#include <memory>

#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/simulator/standard_topics.hh"

namespace resim::actor {

namespace {
// Simple helper we use to double check that the actor is outputing sensible
// values.
void validate_observable_state(
    const Actor &actor,
    const state::ObservableState &state,
    const time::Timestamp time) {
  REASSERT(actor.id() == state.id, "Invalid actor id!");
  REASSERT(time == state.time_of_validity, "Invalid state time!");
}
}  // namespace

ActorUnit::ActorUnit(
    std::unique_ptr<Actor> actor,
    InOut<simulator::ExecutorBuilder> executor_builder)
    : actor_{std::move(actor)} {
  REASSERT(actor_ != nullptr, "Null actor detected!");
  executor_builder->add_task<time::Timestamp, state::ObservableState>(
      "publish_actor_states",
      simulator::TIME_TOPIC,
      simulator::ACTOR_STATES_TOPIC,
      [&](const time::Timestamp time) {
        actor_->simulate_forward(time);
        state::ObservableState state{actor_->observable_state()};
        validate_observable_state(*actor_, state, time);
        return state;
      });
};

}  // namespace resim::actor
