// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <string>
#include <unordered_map>

#include "resim/experiences/dynamic_behavior.hh"
#include "resim/experiences/geometry.hh"
#include "resim/utils/uuid.hh"

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
