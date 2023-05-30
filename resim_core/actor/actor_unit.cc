
#include "resim_core/actor/actor_unit.hh"

#include <fmt/core.h>
#include <foxglove/FrameTransform.pb.h>

#include <memory>
#include <string_view>

#include "resim_core/actor/geometry.hh"
#include "resim_core/actor/state/observable_state.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/simulator/simulation_unit.hh"
#include "resim_core/simulator/standard_frames.hh"
#include "resim_core/simulator/standard_topics.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/utils/mcap_logger.hh"
#include "resim_core/visualization/foxglove/actor_geometry_to_foxglove.hh"
#include "resim_core/visualization/foxglove/frame_transform_to_foxglove.hh"

namespace resim::actor {

namespace {
const std::string FOXGLOVE_GEOMETRIES_TOPIC{"/geometries"};

// Simple helper we use to double check that the actor is outputing sensible
// values.
void validate_observable_state(
    const Actor &actor,
    const state::ObservableState &state,
    const time::Timestamp time) {
  REASSERT(actor.id() == state.id, "Invalid actor id!");
  REASSERT(time == state.time_of_validity, "Invalid state time!");
}

std::string get_transform_name(const transforms::FSE3 &ref_from_body) {
  REASSERT(
      ref_from_body.into() == simulator::SCENE_FRAME,
      "Actors must be relative to scene frame!");
  return fmt::format(
      "/transforms/{}_from_{}",
      simulator::SCENE_FRAME_NAME,
      ref_from_body.from().id().to_string());
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
  executor_builder->add_task<actor::Geometry>(
      "publish_actor_geometries",
      simulate_forward_dependency_,
      simulator::ACTOR_GEOMETRIES_TOPIC,
      [this]() { return actor_->geometry(); });
};

ActorLoggerUnit::ActorLoggerUnit(
    std::shared_ptr<LoggerInterface> logger_interface,
    InOut<simulator::ExecutorBuilder> executor_builder)
    : SimulationUnit(std::move(logger_interface)) {
  REASSERT(logger() != nullptr, "Null logger passed to actor logger unit");

  logger()->add_proto_channel<foxglove::SceneUpdate>(FOXGLOVE_GEOMETRIES_TOPIC);

  executor_builder->add_task<actor::Geometry>(
      "log_actor_geometries",
      simulator::ACTOR_GEOMETRIES_TOPIC,
      LOG_ACTOR_GEOMETRIES_TOPIC,
      [&](const std::vector<actor::Geometry> &actor_geometries) {
        log_geometries_update(actor_geometries);
      });

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
  for (const auto &actor_state : actor_states) {
    if (not actor_state.is_spawned) {
      continue;
    }
    const transforms::FSE3 &scene_from_actor{actor_state.state.ref_from_body()};

    ::foxglove::FrameTransform frame_transform;
    visualization::foxglove::pack_into_foxglove(
        scene_from_actor,
        actor_state.time_of_validity,
        &frame_transform,
        std::string(simulator::SCENE_FRAME_NAME),
        scene_from_actor.from().id().to_string());

    const std::string topic_name{get_transform_name(scene_from_actor)};
    // This is a no-op if already added:
    logger()->add_proto_channel<::foxglove::FrameTransform>(topic_name);
    logger()->log_proto(
        topic_name,
        actor_state.time_of_validity,
        frame_transform);
  }
}

void ActorLoggerUnit::log_geometries_update(
    const std::vector<actor::Geometry> &actor_geometries) const {
  ::foxglove::SceneUpdate scene_update;
  std::optional<time::Timestamp> time;
  for (const auto &geometry : actor_geometries) {
    visualization::foxglove::pack_into_foxglove(geometry, &scene_update);

    if (not time.has_value()) {
      time = geometry.time_of_validity;
    }
    // NOLINTBEGIN(bugprone-unchecked-optional-access)
    REASSERT(
        *time == geometry.time_of_validity,
        "Inconsistent times in actor geometries!");
    // NOLINTEND(bugprone-unchecked-optional-access)
  }

  // This should be logically impossible since log_actor_geometies is only
  // called when we receive a vector of them.
  REASSERT(
      time.has_value(),
      "log_actor_geometries() must be called with non-empty geometries!");
  // NOLINTBEGIN(bugprone-unchecked-optional-access)
  logger()->log_proto(FOXGLOVE_GEOMETRIES_TOPIC, *time, scene_update);
  // NOLINTEND(bugprone-unchecked-optional-access)
}

}  // namespace resim::actor
