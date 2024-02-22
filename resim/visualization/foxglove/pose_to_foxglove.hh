// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <foxglove/Pose.pb.h>

#include "resim/transforms/se3.hh"

namespace resim::visualization::foxglove {

// Pack an SE3 into a foxglove pose message
void pack_into_foxglove(const transforms::SE3 &in, ::foxglove::Pose *out);

}  // namespace resim::visualization::foxglove
