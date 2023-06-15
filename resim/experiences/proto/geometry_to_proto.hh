#pragma once

#include "resim/experiences/geometry.hh"
#include "resim/experiences/proto/geometry.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::Geometry &in, Geometry *out);

experiences::Geometry unpack(const Geometry &in);

}  // namespace resim::experiences::proto
