#pragma once

#include "resim_core/experiences/location_condition.hh"
#include "resim_core/experiences/proto/location_condition.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::LocationCondition &in, LocationCondition *out);

experiences::LocationCondition unpack(const LocationCondition &in);

}  // namespace resim::experiences::proto
