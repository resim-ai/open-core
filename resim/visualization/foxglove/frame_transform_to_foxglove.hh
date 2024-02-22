// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <foxglove/FrameTransform.pb.h>

#include "resim/time/proto/time_to_proto.hh"
#include "resim/transforms/se3.hh"

namespace resim::visualization::foxglove {

// Pack a framed SE3 into a FrameTransform foxglove message for visualization
// purposes.
// @param[in] in - The SE3 to pack into the FrameTransform.
// @param[in] time - The time to pack.
// @param[out] out - The mesage to pack into.
// @param[in] parent - The parent frame id to use. The into frame id from "in"
//                     is used if this is left empty.
// @param[in] child - The child frame id to use. The from frame id from "in" is
//                    used if this is left empty.
void pack_into_foxglove(
    const transforms::SE3 &in,
    time::Timestamp time,
    ::foxglove::FrameTransform *out,
    const std::string &parent = "",
    const std::string &child = "");

}  // namespace resim::visualization::foxglove
