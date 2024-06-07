// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/log/visualize_actor_states.hh"

#include <fmt/core.h>
#include <foxglove/FrameTransform.pb.h>
#include <foxglove/SceneEntityDeletion.pb.h>
#include <foxglove/SceneUpdate.pb.h>

#include <string>
#include <vector>

#include "resim/actor/actor_id.hh"
#include "resim/actor/state/observable_state.hh"
#include "resim/actor/state/proto/observable_state.pb.h"
#include "resim/actor/state/proto/observable_state_to_proto.hh"
#include "resim/assert/assert.hh"
#include "resim/experiences/geometry.hh"
#include "resim/simulator/standard_frames.hh"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/match.hh"
#include "resim/visualization/foxglove/frame_transform_to_foxglove.hh"
#include "resim/visualization/foxglove/wireframe_to_foxglove.hh"
#include "resim/visualization/log/visit_topic.hh"

namespace resim::visualization::log {

// Helper to get the right topic name for each actor's FrameTransform.
std::string get_transform_name(const transforms::SE3 &ref_from_body) {
  REASSERT(
      ref_from_body.into() == simulator::SCENE_FRAME,
      "Actors must be relative to scene frame!");
  return fmt::format(
      "/transforms/{}_from_{}",
      simulator::SCENE_FRAME_NAME,
      ref_from_body.from().id().to_string());
}

// Helper which logs frame transforms for all of the given actor states.
void log_frame_transforms(
    const std::vector<actor::state::ObservableState> &actor_states,
    InOut<McapLogger> logger) {
  for (const auto &actor_state : actor_states) {
    if (not actor_state.is_spawned) {
      continue;
    }
    const transforms::SE3 &scene_from_actor{actor_state.state.ref_from_body()};

    ::foxglove::FrameTransform frame_transform;
    foxglove::pack_into_foxglove(
        scene_from_actor,
        actor_state.time_of_validity,
        &frame_transform,
        std::string(simulator::SCENE_FRAME_NAME),
        scene_from_actor.from().id().to_string());

    const std::string topic_name{get_transform_name(scene_from_actor)};
    // This is a no-op if already added:
    logger->add_proto_channel<::foxglove::FrameTransform>(topic_name);
    logger->log_proto(
        topic_name,
        actor_state.time_of_validity,
        frame_transform);
  }
}

// Helper which logs each actor's geometry once it spawns and a deletion for it
// when it despawns.
// @param[in] actor_states - The actor states. Used to find spawning and
//                           despawning events.
// @param[in] actor_geometries - The actor geometries, extracted from the
//                               Experience config and provided by the caller
//                               in a map.
// @param[in] log_time - The time to log the geometries at.
// @param[inout] logger - The logger to write the geometries to.
// @param[inout] visible_actors - A set containing the ids of the
//                                currently-visible actors. Used to track which
//                                actors have spawned but haven't had their
//                                geometries published yet and which have
//                                despawned but haven't had their deletion
//                                published yet.
void log_geometries(
    const std::vector<actor::state::ObservableState> &actor_states,
    const std::unordered_map<actor::ActorId, experiences::Geometry>
        &actor_geometries,
    const time::Timestamp log_time,
    InOut<McapLogger> logger,
    InOut<std::unordered_set<actor::ActorId>> visible_actors) {
  ::foxglove::SceneUpdate scene_update;
  for (const auto &actor_state : actor_states) {
    const transforms::SE3 &scene_from_actor{actor_state.state.ref_from_body()};

    const actor::ActorId &id{actor_state.id};
    const std::string frame_id{scene_from_actor.from().id().to_string()};

    if (not actor_geometries.contains(id)) {
      continue;
    }

    // Spawning
    // TODO(michael) Potentially do this on some slow cadence also so that
    // geometries don't disappear when playing back.
    if (actor_state.is_spawned and not visible_actors->contains(id)) {
      visible_actors->emplace(id);
      const experiences::Geometry &geometry{actor_geometries.at(id)};

      auto &entity = *scene_update.add_entities();

      time::proto::pack(
          actor_state.time_of_validity,
          entity.mutable_timestamp());
      entity.set_frame_id(frame_id);
      entity.set_id(id.to_string());
      constexpr bool DEFAULT_FRAME_LOCKED = true;
      entity.set_frame_locked(DEFAULT_FRAME_LOCKED);

      match(geometry.model, [&](const geometry::Wireframe &wireframe) {
        foxglove::pack_into_foxglove(wireframe, entity.add_lines());
      });
      // Despawning
    } else if (not actor_state.is_spawned and visible_actors->contains(id)) {
      visible_actors->erase(id);

      auto &deletion = *scene_update.add_deletions();
      time::proto::pack(
          actor_state.time_of_validity,
          deletion.mutable_timestamp());
      deletion.set_type(::foxglove::SceneEntityDeletion::MATCHING_ID);
      deletion.set_id(id.to_string());
    }
  }
  logger->add_proto_channel<::foxglove::SceneUpdate>("/geometries");
  logger->log_proto("/geometries", log_time, scene_update);
}

void visualize_actor_states(
    const experiences::Experience &experience,
    InOut<mcap::McapReader> reader,
    InOut<McapLogger> logger) {
  std::unordered_set<actor::ActorId> visible_actors;

  std::unordered_map<actor::ActorId, experiences::Geometry> actor_geometries;

  for (const auto &actor : experience.dynamic_behavior.actors) {
    for (const auto &geometry_ref : actor.geometries) {
      REASSERT(
          experience.geometries.contains(geometry_ref.geometry_id),
          "Geometry not found for actor!");
      REASSERT(
          actor_geometries
              .emplace(
                  actor.id,
                  experience.geometries.at(geometry_ref.geometry_id))
              .second,
          "Duplicate geometry for actor!");
    }
  }

  visit_topic("/actor_states", reader, [&](const mcap::MessageView &view) {
    actor::state::proto::ObservableStates states_msg;
    REASSERT(
        states_msg.ParseFromArray(view.message.data, view.message.dataSize));
    std::vector<actor::state::ObservableState> states{unpack(states_msg)};

    log_frame_transforms(states, logger);
    log_geometries(
        states,
        actor_geometries,
        time::Timestamp{time::Duration{view.message.logTime}},
        InOut{logger},
        InOut{visible_actors});
  });
}

}  // namespace resim::visualization::log
