// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/curve/line_primitive_from_curve_test_helpers.hh"

#include <foxglove/Color.pb.h>
#include <foxglove/Pose.pb.h>
#include <gtest/gtest.h>

#include <random>

#include "resim/visualization/color.hh"

namespace resim::visualization::curve {

TEST(LinePrimitiveFromCurveTestHelpersTest, TestExpectColorsEqual) {
  // SETUP
  constexpr int SEED = 93;
  std::mt19937 rng{SEED};
  std::uniform_real_distribution<double> dist{0.0, 1.0};
  ::foxglove::Color c1;

  Color c2;

  c1.set_a(dist(rng));
  c1.set_r(dist(rng));
  c1.set_g(dist(rng));
  c1.set_b(dist(rng));

  c2.a = c1.a();
  c2.r = c1.r();
  c2.g = c1.g();
  c2.b = c1.b();

  // ACTION / VERFICIATION
  expect_colors_equal(c1, c2);
}

TEST(LinePrimitiveFromCurveTestHelpersTest, TestExpect) {
  // SETUP
  ::foxglove::Pose pose;
  pose.mutable_orientation()->set_w(1.0);

  // ACTION / VERIFICATION
  expect_pose_identity(pose);
}

}  // namespace resim::visualization::curve
