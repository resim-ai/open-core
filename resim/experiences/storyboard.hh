// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <variant>
#include <vector>

#include "resim/actor/state/trajectory.hh"
#include "resim/experiences/ilqr_drone.hh"
#include "resim/utils/uuid.hh"

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
