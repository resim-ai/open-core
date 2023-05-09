
#pragma once

#include "resim_core/experiences/experience.hh"
#include "resim_core/experiences/proto/experience.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::Experience &in, Experience *out);

experiences::Experience unpack(const Experience &in);

}  // namespace resim::experiences::proto
