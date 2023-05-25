#pragma once

#include "resim_core/geometry/proto/wireframe.pb.h"
#include "resim_core/geometry/wireframe.hh"

namespace resim::geometry::proto {

void pack(const geometry::Wireframe &in, Wireframe *out);

geometry::Wireframe unpack(const Wireframe &in);

}  // namespace resim::geometry::proto
