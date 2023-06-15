#pragma once

#include "resim/experiences/location_condition.hh"
#include "resim/experiences/proto/location_condition.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::LocationCondition &in, LocationCondition *out);

experiences::LocationCondition unpack(const LocationCondition &in);

}  // namespace resim::experiences::proto
