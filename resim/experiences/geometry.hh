#pragma once

#include <variant>

#include "resim/geometry/wireframe.hh"
#include "resim/utils/uuid.hh"

namespace resim::experiences {

// This struct describes the configuration for a single visualizable geometry
// that may be referenced by other components of the experience (e.g. actors)
struct Geometry {
  UUID id;
  std::variant<geometry::Wireframe> model;
};

}  // namespace resim::experiences
