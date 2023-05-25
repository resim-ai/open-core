#pragma once

#include "resim_core/experiences/geometry.hh"
#include "resim_core/experiences/proto/geometry.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::Geometry &in, Geometry *out);

experiences::Geometry unpack(const Geometry &in);

}  // namespace resim::experiences::proto
