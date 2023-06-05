#include "resim_core/actor/factory.hh"

#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/actor/trajectory_actor.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/experiences/actor.hh"
#include "resim_core/experiences/storyboard.hh"
#include "resim_core/utils/match.hh"

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
    const experiences::DynamicBehavior &dynamic_behavior,
    const std::unordered_map<UUID, experiences::Geometry> &geometries) {
  const std::unordered_map<ActorId, experiences::MovementModel>
      movement_model_map{make_movement_model_map(dynamic_behavior.storyboard)};

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
          std::optional<experiences::Geometry> maybe_geometry;
          REASSERT(
              actor.geometries.size() <= 1U,
              "Cannot yet support multiple geometries per actor!");

          for (const auto &geometry_ref : actor.geometries) {
            REASSERT(
                geometries.contains(geometry_ref.geometry_id),
                "Cannot find referenced geometry!");
            maybe_geometry = geometries.at(geometry_ref.geometry_id);
          }
          actors.push_back(std::make_unique<resim::actor::TrajectoryActor>(
              actor.id,
              trajectory,
              maybe_geometry));
        },
        [](const auto &) { REASSERT(false, "Unsupported MovementModel!"); });
  }
  return actors;
}

}  // namespace resim::actor
