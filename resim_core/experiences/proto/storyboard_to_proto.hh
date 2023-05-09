#pragma once

#include "resim_core/experiences/proto/storyboard.pb.h"
#include "resim_core/experiences/storyboard.hh"

namespace resim::experiences::proto {

void pack(const experiences::Storyboard &in, Storyboard *out);

experiences::Storyboard unpack(const Storyboard &in);

}  // namespace resim::experiences::proto
