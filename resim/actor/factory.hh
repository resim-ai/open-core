#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "resim/actor/actor.hh"
#include "resim/experiences/dynamic_behavior.hh"
#include "resim/utils/uuid.hh"

namespace resim::actor {

// This function creates a vector of Actors reflecting the actors in the given
// DynamicBehavior.
// @param[in] dynamic_behavior - The dynamic behavior config containing the
//                               actors.
std::vector<std::unique_ptr<Actor>> factory(
    const experiences::DynamicBehavior &dynamic_behavior);

}  // namespace resim::actor
