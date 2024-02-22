#pragma once

#include <foxglove/Color.pb.h>

#include "resim/visualization/color.hh"

namespace resim::visualization::foxglove {

// Pack the given color into a ::foxglove::Color message.
void pack_into_foxglove(const Color &in, ::foxglove::Color *out);

}  // namespace resim::visualization::foxglove
