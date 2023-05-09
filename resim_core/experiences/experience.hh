#pragma once

#include <string>

#include "resim_core/experiences/dynamic_behavior.hh"

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

// An experience consists of a header containing the experience metadata and a
// DynamicBehavior struct that encodes the agents and their actions in the
// experience. An experience is often termed a scenario in the autonomous car
// industry.
struct Experience {
  Header header;
  DynamicBehavior dynamic_behavior;
};

}  // namespace resim::experiences
