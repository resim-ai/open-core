// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/actor/actor_unit.hh"

#include <fmt/core.h>

#include <memory>
#include <string_view>

#include "resim/actor/state/observable_state.hh"
#include "resim/actor/state/proto/observable_state.pb.h"
#include "resim/actor/state/proto/observable_state_to_proto.hh"
#include "resim/assert/assert.hh"
#include "resim/simulator/simulation_unit.hh"
#include "resim/simulator/standard_frames.hh"
#include "resim/simulator/standard_topics.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/mcap_logger.hh"

namespace resim::actor {

namespace {
constexpr auto ACTOR_STATES_LOG_TOPIC = "/actor_states";

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
    std::shared_ptr<LoggerInterface> logger_interface,
    std::unique_ptr<Actor> actor,
    InOut<simulator::ExecutorBuilder> executor_builder)
    : SimulationUnit(std::move(logger_interface)),
      actor_{std::move(actor)},
      simulate_forward_dependency_{fmt::format(
          "simulate_forward_{}",
          (actor_ != nullptr ? actor_->id().to_string() : ""))} {
  REASSERT(actor_ != nullptr, "Null actor passed to actor unit");

  executor_builder->add_task<time::Timestamp>(
      "simulate_forward",
      simulator::TIME_TOPIC,
      simulate_forward_dependency_,
      [this](const time::Timestamp time) {
        REASSERT(actor_ != nullptr, "Null actor in simulate forward.");
        actor_->simulate_forward(time);
      });

  executor_builder->add_task<state::ObservableState>(
      "publish_actor_states",
      simulate_forward_dependency_,
      simulator::ACTOR_STATES_TOPIC,
      [this]() {
        state::ObservableState state{actor_->observable_state()};
        validate_observable_state(*actor_, state, actor_->current_time());
        return state;
      });
};

ActorLoggerUnit::ActorLoggerUnit(
    std::shared_ptr<LoggerInterface> logger_interface,
    InOut<simulator::ExecutorBuilder> executor_builder)
    : SimulationUnit(std::move(logger_interface)) {
  REASSERT(logger() != nullptr, "Null logger passed to actor logger unit");

  // This is a trick to allow us to guarantee that we have an updated time
  // before we process actor states. We don't actually write an actor state,
  // but this must run before things dependent on actor states can read them.
  // This could be simplified once we support depending on multiple
  // dependencies simultaneously.
  executor_builder->add_task<time::Timestamp>(
      "receive_time",
      simulator::TIME_TOPIC,
      simulator::ACTOR_STATES_TOPIC,
      [this](const time::Timestamp time) { latest_time_ = time; });

  executor_builder->add_task<actor::state::ObservableState>(
      "log_actor_states",
      simulator::ACTOR_STATES_TOPIC,
      LOG_ACTOR_STATES_TOPIC,
      [&](const std::vector<actor::state::ObservableState> &actor_states) {
        log_actor_states(actor_states);
      });
};

void ActorLoggerUnit::log_actor_states(
    const std::vector<actor::state::ObservableState> &actor_states) {
  state::proto::ObservableStates states_msg;
  pack(actor_states, &states_msg);

  // This is a no-op if already added:
  logger()->add_proto_channel<state::proto::ObservableStates>(
      ACTOR_STATES_LOG_TOPIC);
  logger()->log_proto(ACTOR_STATES_LOG_TOPIC, latest_time_, states_msg);
}

}  // namespace resim::actor
