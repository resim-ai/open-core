#pragma once

#include "resim/transforms/proto/se3.pb.h"
#include "resim/transforms/se3.hh"

namespace resim::transforms::proto {

void pack(const transforms::SE3 &in, SE3 *out);

transforms::SE3 unpack(const SE3 &in);

}  // namespace resim::transforms::proto
