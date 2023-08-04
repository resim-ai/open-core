// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

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
