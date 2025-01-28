// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/actor/factory.hh"

#include "resim/actor/actor_id.hh"
#include "resim/actor/ilqr_drone.hh"
#include "resim/actor/state/trajectory.hh"
#include "resim/actor/trajectory_actor.hh"
#include "resim/assert/assert.hh"
#include "resim/curves/t_curve.hh"
#include "resim/experiences/actor.hh"
#include "resim/experiences/storyboard.hh"
#include "resim/utils/match.hh"
#include "resim/utils/uuid.hh"

namespace resim::actor {

namespace {

// A helper to create a map from actor id to movement model, failing on
// duplicate entries
std::unordered_map<ActorId, experiences::MovementModel> make_movement_model_map(
    const resim::experiences::Storyboard &storyboard) {
  std::unordered_map<ActorId, experiences::MovementModel> map;
  for (const auto &movement_model : storyboard.movement_models) {
    REASSERT(
        map.emplace(movement_model.actor_reference, movement_model).second,
        "Duplicate MovementModel for actor!");
  }
  return map;
}
}  // namespace

std::vector<std::unique_ptr<Actor>> factory(
    experiences::DynamicBehavior &dynamic_behavior2) {
  auto &dynamic_behavior = dynamic_behavior2;
  auto seed_actor = dynamic_behavior.actors.at(0);

  std::unordered_map<ActorId, experiences::MovementModel> movement_model_map{
      make_movement_model_map(dynamic_behavior.storyboard)};

  for (const Eigen::Vector3d &xi : {
           Eigen::Vector3d{1.0, 1.0, 0.0},
           Eigen::Vector3d{-1.0, 1.0, 0.0},
           Eigen::Vector3d{1.0, -1.0, 0.0},
           Eigen::Vector3d{-1.0, -1.0, 0.0},
       }) {
    auto new_actor = seed_actor;
    const auto new_id = UUID::new_uuid();
    new_actor.id = new_id;
    auto new_movement_model = movement_model_map.at(seed_actor.id);
    new_movement_model.actor_reference = new_id;
    std::get<experiences::ILQRDrone>(new_movement_model.model)
        .initial_position += 3.0 * xi;
    dynamic_behavior.actors.push_back(new_actor);
    dynamic_behavior.storyboard.movement_models.push_back(new_movement_model);
  }
  movement_model_map = make_movement_model_map(dynamic_behavior.storyboard);

  std::vector<std::unique_ptr<Actor>> actors;
  for (const auto &actor : dynamic_behavior.actors) {
    REASSERT(
        movement_model_map.contains(actor.id),
        "No MovementModel for actor!");

    REASSERT(
        actor.actor_type != experiences::ActorType::INVALID,
        "Invalid actor type!");
    match(
        movement_model_map.at(actor.id).model,
        [&](const state::Trajectory &trajectory) {
          actors.push_back(std::make_unique<resim::actor::TrajectoryActor>(
              actor.id,
              trajectory));
        },
        [&](const experiences::ILQRDrone &drone) {
          actors.push_back(std::make_unique<actor::ILQRDrone>(
              actor.id,
              drone.initial_position,
              drone.goal_position,
              drone.velocity_cost));
        },
        [](const auto &) { REASSERT(false, "Unsupported MovementModel!"); });
  }
  return actors;
}

}  // namespace resim::actor
