// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/test_helpers.hh"

#include <gtest/gtest.h>

#include "resim/curves/t_curve.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves::testing {

using Frame = transforms::Frame<3>;
using transforms::SE3;
using TangentVector = SE3::TangentVector;
using Vec3 = Eigen::Vector3d;
using TwoJetL = curves::TwoJetL<SE3>;

// Sample the curve along its length to verify that it has the right translation
// and orientation along the length.
// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(TestHelpersTest, TestMakeCircleCurve) {
  // SETUP
  const Frame into{Frame::new_frame()};
  const Frame from{Frame::new_frame()};

  // ACTION
  const curves::TCurve<SE3> curve{make_circle_curve(into, from)};

  // VERIFICATION
  EXPECT_EQ(curve.start_time(), 0.0);
  EXPECT_EQ(curve.end_time(), 2. * M_PI);

  const TangentVector expected_velocity{
      -(TangentVector() << Vec3::UnitZ(), Vec3::UnitX()).finished()};

  constexpr int NUM_SAMPLES = 100;
  for (int ii = 0; ii < NUM_SAMPLES; ++ii) {
    const double frac = static_cast<double>(ii) / (NUM_SAMPLES - 1);
    const double time =
        frac * curve.start_time() + (1. - frac) * curve.end_time();

    const TwoJetL point_from_ref{curve.point_at(time)};
    const Vec3 translation{
        point_from_ref.frame_from_ref().inverse().translation()};
    constexpr double TOLERANCE = 1e-7;
    EXPECT_NEAR(translation.norm(), 1.0, TOLERANCE);
    EXPECT_NEAR(translation.x(), std::cos(time), TOLERANCE);
    EXPECT_NEAR(translation.y(), std::sin(time), TOLERANCE);
    EXPECT_NEAR(translation.z(), 0., TOLERANCE);
    const Vec3 direction{
        point_from_ref.frame_from_ref().rotation().inverse() * Vec3::UnitX()};
    EXPECT_NEAR(direction.x(), -std::sin(time), TOLERANCE);
    EXPECT_NEAR(direction.y(), std::cos(time), TOLERANCE);

    EXPECT_TRUE(expected_velocity.isApprox(point_from_ref.d_frame_from_ref()));
    EXPECT_TRUE(point_from_ref.d2_frame_from_ref().isZero());

    EXPECT_EQ(point_from_ref.frame_from_ref().into(), into);
    EXPECT_EQ(point_from_ref.frame_from_ref().from(), from);

    // Check that the ref origin is along the y axis of the frame:
    const Vec3 expected_origin =
        point_from_ref.frame_from_ref().inverse() * Vec3::UnitY();
    EXPECT_TRUE(expected_origin.isZero());
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim::curves::testing
