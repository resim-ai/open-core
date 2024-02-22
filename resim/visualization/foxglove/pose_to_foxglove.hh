#pragma once

#include <foxglove/Pose.pb.h>

#include "resim/transforms/se3.hh"

namespace resim::visualization::foxglove {

// Pack an SE3 into a foxglove pose message
void pack_into_foxglove(const transforms::SE3 &in, ::foxglove::Pose *out);

}  // namespace resim::visualization::foxglove
