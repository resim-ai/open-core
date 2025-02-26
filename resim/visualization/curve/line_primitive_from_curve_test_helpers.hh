// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <foxglove/Color.pb.h>
#include <foxglove/Pose.pb.h>

#include "resim/visualization/color.hh"

namespace resim::visualization::curve {
// A simple helper to check that a given color message matches the given Color
// object.
void expect_colors_equal(const ::foxglove::Color &msg, const Color &color);

// A simple helper to check that the given pose is the identity.
void expect_pose_identity(const ::foxglove::Pose &pose);
}  // namespace resim::visualization::curve
