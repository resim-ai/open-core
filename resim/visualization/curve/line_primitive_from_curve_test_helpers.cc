// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/curve/line_primitive_from_curve_test_helpers.hh"

#include <gtest/gtest.h>

namespace resim::visualization::curve {
// A simple helper to check that a given color message matches the given Color
// object.
void expect_colors_equal(const ::foxglove::Color &msg, const Color &color) {
  EXPECT_EQ(msg.r(), color.r);
  EXPECT_EQ(msg.g(), color.g);
  EXPECT_EQ(msg.b(), color.b);
  EXPECT_EQ(msg.a(), color.a);
}

// A simple helper to check that the given pose is the identity.
// NOLINTBEGIN(readability-function-cognitive-complexity)
void expect_pose_identity(const ::foxglove::Pose &pose) {
  EXPECT_EQ(pose.position().x(), 0.0);
  EXPECT_EQ(pose.position().y(), 0.0);
  EXPECT_EQ(pose.position().z(), 0.0);

  EXPECT_EQ(pose.orientation().w(), 1.0);
  EXPECT_EQ(pose.orientation().x(), 0.0);
  EXPECT_EQ(pose.orientation().y(), 0.0);
  EXPECT_EQ(pose.orientation().z(), 0.0);
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim::visualization::curve
