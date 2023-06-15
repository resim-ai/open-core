
#pragma once

#include "resim/experiences/dynamic_behavior.hh"
#include "resim/experiences/proto/dynamic_behavior.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::DynamicBehavior &in, DynamicBehavior *out);

experiences::DynamicBehavior unpack(const DynamicBehavior &in);

}  // namespace resim::experiences::proto
