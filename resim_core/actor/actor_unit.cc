
#include "resim_core/actor/actor_unit.hh"

#include <fmt/core.h>

#include <memory>
#include <string_view>

#include "resim_core/actor/geometry.hh"
#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/simulator/standard_topics.hh"
#include "resim_core/time/timestamp.hh"

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
    : actor_{std::move(actor)},
      simulate_forward_dependency_{fmt::format(
          "simulate_forward_{}",
          (actor_ != nullptr ? actor_->id().to_string() : ""))} {
  REASSERT(actor_ != nullptr, "Null actor detected!");

  executor_builder->add_task<time::Timestamp>(
      "simulate_forward",
      simulator::TIME_TOPIC,
      simulate_forward_dependency_,
      [this](const time::Timestamp time) { actor_->simulate_forward(time); });

  executor_builder->add_task<state::ObservableState>(
      "publish_actor_states",
      simulate_forward_dependency_,
      simulator::ACTOR_STATES_TOPIC,
      [this]() {
        state::ObservableState state{actor_->observable_state()};
        validate_observable_state(*actor_, state, actor_->current_time());
        return state;
      });
  executor_builder->add_task<Geometry>(
      "publish_actor_geometries",
      simulate_forward_dependency_,
      simulator::ACTOR_GEOMETRIES_TOPIC,
      [this]() { return actor_->geometry(); });
};

}  // namespace resim::actor
