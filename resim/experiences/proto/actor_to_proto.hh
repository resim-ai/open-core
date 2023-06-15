#pragma once

#include "resim/experiences/actor.hh"
#include "resim/experiences/proto/actor.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::Actor &in, Actor *out);

experiences::Actor unpack(const Actor &in);

}  // namespace resim::experiences::proto
