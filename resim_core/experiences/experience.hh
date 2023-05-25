#pragma once

#include <string>
#include <unordered_map>

#include "resim_core/experiences/dynamic_behavior.hh"
#include "resim_core/experiences/geometry.hh"
#include "resim_core/utils/uuid.hh"

namespace resim::experiences {

struct Revision {
  uint32_t major;
  uint32_t minor;
};

// A header consists of a revision, documenting the schema revision that the
// experience conforms to, a name and description and an optional parent
// experience name, if this is a derived experience.
struct Header {
  Revision revision{};
  std::string name;
  std::string description;
  std::string parent_experience_name;
};

// An experience consists of a header containing the experience metadata, a
// DynamicBehavior struct that encodes the agents and their actions in the
// experience, and a list of geometries that actors may reference by id. An
// experience is often termed a scenario in the autonomous car industry.
struct Experience {
  Header header;
  DynamicBehavior dynamic_behavior;
  std::unordered_map<UUID, Geometry> geometries;
};

}  // namespace resim::experiences
