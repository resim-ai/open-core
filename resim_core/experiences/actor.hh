#pragma once

#include <string>

#include "resim_core/utils/uuid.hh"

namespace resim::experiences {

enum class ActorType {
  INVALID = 0,
  SYSTEM_UNDER_TEST = 1,
  SIMULATION_ACTOR = 2,
};

struct Actor {
  UUID id;
  std::string name;
  // Inititialized invalid, but should be set by the experience.
  ActorType actor_type = ActorType::INVALID;
};

}  // namespace resim::experiences
