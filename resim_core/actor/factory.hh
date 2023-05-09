#pragma once

#include <memory>
#include <vector>

#include "resim_core/actor/actor.hh"
#include "resim_core/experiences/dynamic_behavior.hh"

namespace resim::actor {

// This function creates a vector of Actors reflecting the actors in the given
// DynamicBehavior.
// @param[in] dynamic_behavior - The dynamic behavior config containing the
//                               actors.
std::vector<std::unique_ptr<Actor>> factory(
    const experiences::DynamicBehavior &dynamic_behavior);

}  // namespace resim::actor
