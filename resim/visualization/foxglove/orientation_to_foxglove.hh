
#pragma once

#include <foxglove/Quaternion.pb.h>

#include "resim/transforms/so3.hh"

namespace resim::visualization::foxglove {

// Pack an SO3 into a ::foxglove::Quaternion proto message.
void pack_into_foxglove(const transforms::SO3 &in, ::foxglove::Quaternion *out);

}  // namespace resim::visualization::foxglove
