// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

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
    experiences::DynamicBehavior &dynamic_behavior);

}  // namespace resim::actor
