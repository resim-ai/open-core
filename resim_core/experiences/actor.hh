#pragma once

#include <string>
#include <vector>

#include "resim_core/utils/uuid.hh"

namespace resim::experiences {

enum class ActorType {
  INVALID = 0,
  SYSTEM_UNDER_TEST = 1,
  SIMULATION_ACTOR = 2,
};

struct GeometryReference {
  UUID geometry_id;
};

inline bool operator==(const GeometryReference &a, const GeometryReference &b) {
  return a.geometry_id == b.geometry_id;
}

struct Actor {
  UUID id;
  std::string name;
  // Inititialized invalid, but should be set by the experience.
  ActorType actor_type = ActorType::INVALID;
  std::vector<GeometryReference> geometries;
};

}  // namespace resim::experiences
