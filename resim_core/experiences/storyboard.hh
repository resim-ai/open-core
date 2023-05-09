#pragma once

#include <variant>
#include <vector>

#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/experiences/ilqr_drone.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/utils/uuid.hh"

namespace resim::experiences {

// A movement model assigns an actor a mechanism to move within the simulation.
// Currently we only support a fixed trajectory model or an iLQR model.
struct MovementModel {
  UUID actor_reference;
  std::variant<actor::state::Trajectory, ILQRDrone> model;
};

// A storyboard is currently a list of movement models.
struct Storyboard {
  std::vector<MovementModel> movement_models;
};

}  // namespace resim::experiences
