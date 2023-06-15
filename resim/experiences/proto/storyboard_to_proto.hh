#pragma once

#include "resim/experiences/proto/storyboard.pb.h"
#include "resim/experiences/storyboard.hh"

namespace resim::experiences::proto {

void pack(const experiences::Storyboard &in, Storyboard *out);

experiences::Storyboard unpack(const Storyboard &in);

}  // namespace resim::experiences::proto
