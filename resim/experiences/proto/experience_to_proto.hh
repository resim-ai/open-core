
#pragma once

#include "resim/experiences/experience.hh"
#include "resim/experiences/proto/experience.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::Experience &in, Experience *out);

experiences::Experience unpack(const Experience &in);

}  // namespace resim::experiences::proto
