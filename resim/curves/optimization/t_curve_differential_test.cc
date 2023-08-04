// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/optimization/t_curve_differential.hh"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "resim/assert/assert.hh"
#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/curves/t_curve.hh"
#include "resim/curves/two_jet.hh"
#include "resim/curves/two_jet_test_helpers.hh"
#include "resim/math/is_approx.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves::optimization {

template <typename Group>
class TCurveDifferentialTest : public ::testing::Test {
 protected:
  using TwoJet = TwoJetL<Group>;
  TwoJetTestHelper<TwoJet> test_helper_;
};

using LieGroupTypes = ::testing::Types<transforms::SE3, transforms::SO3>;

TYPED_TEST_SUITE(TCurveDifferentialTest, LieGroupTypes);

TYPED_TEST(TCurveDifferentialTest, TestBadTimes) {
  // SETUP
  using Group = TypeParam;
  using Control = typename TCurve<Group>::Control;

  constexpr double PREV_TIME = 1.0;
  constexpr double NEXT_TIME = 5.0;

  const Control prev{
      .time = PREV_TIME,
      .point = this->test_helper_.make_test_two_jet(),
  };
  const Control next{
      .time = NEXT_TIME,
      .point = this->test_helper_.make_test_two_jet(),
  };

  // ACTION / VERIFICATION
  EXPECT_NO_THROW(point_at<Group>(PREV_TIME, prev, next));
  EXPECT_NO_THROW(point_at<Group>(NEXT_TIME, prev, next));
  EXPECT_THROW(
      point_at<Group>(std::nextafter(PREV_TIME, -INFINITY), prev, next),
      AssertException);
  EXPECT_THROW(
      point_at<Group>(std::nextafter(NEXT_TIME, INFINITY), prev, next),
      AssertException);
}

TYPED_TEST(TCurveDifferentialTest, TestBadFrames) {
  // SETUP
  using Group = TypeParam;
  using TwoJet = typename TestFixture::TwoJet;
  using Control = typename TCurve<Group>::Control;
  using Frame = transforms::Frame<Group::DIMS>;

  constexpr double PREV_TIME = 1.0;
  constexpr double NEXT_TIME = 5.0;
  constexpr double TIME = 3.1;

  const Control prev{
      .time = PREV_TIME,
      .point = this->test_helper_.make_test_two_jet(),
  };
  TwoJet next_point = this->test_helper_.make_test_two_jet();
  auto next_point_group = next_point.frame_from_ref();
  next_point_group.set_frames(Frame::new_frame(), Frame::new_frame());
  next_point.set_frame_from_ref(next_point_group);
  const Control next{
      .time = NEXT_TIME,
      .point = next_point,
  };

  // ACTION / VERIFICATION
  EXPECT_THROW(point_at<Group>(TIME, prev, next), AssertException);
}

TYPED_TEST(TCurveDifferentialTest, TestFiniteDifferences) {
  // SETUP
  using Group = TypeParam;
  using TwoJet = typename TestFixture::TwoJet;
  using Control = typename TCurve<Group>::Control;
  constexpr double PREV_TIME = 1.0;
  constexpr double NEXT_TIME = 5.0;
  constexpr std::array<double, 4> TIMES = {1.01, 2.5, 3.7, 4.99};

  const Control prev{
      .time = PREV_TIME,
      .point = this->test_helper_.make_test_two_jet(),
  };
  const Control next{
      .time = NEXT_TIME,
      .point = this->test_helper_.make_test_two_jet(),
  };

  // ACTION
  for (const auto time : TIMES) {
    const TCurvePointWithDifferential<Group> point_and_diffs =
        point_at<Group>(time, prev, next);

    // VERIFICATION
    // Verify with finite differences:
    const TCurve<Group> curve{prev, next};
    EXPECT_TRUE(curve.point_at(time).is_approx(point_and_diffs.point));

    for (int ii = 0; ii < TWO_JET_DOF<Group>; ++ii) {
      constexpr double EPSILON = 1e-7;
      constexpr double TOLERANCE = 1e-7;
      using Vec = TwoJetTangentVector<Group>;

      // Previous pertubation
      const Control prev_perturbed{
          .time = prev.time,
          .point = accumulate(prev.point, EPSILON * Vec::Unit(ii)),
      };
      const TCurve<Group> prev_perturbed_curve{prev_perturbed, next};
      const TwoJet prev_perturbed_point{prev_perturbed_curve.point_at(time)};

      Vec expected_differential_column{
          difference(prev_perturbed_point, point_and_diffs.point) / EPSILON};

      EXPECT_TRUE(math::is_approx(
          expected_differential_column,
          point_and_diffs.d_prev.col(ii),
          TOLERANCE));

      // Next pertubation
      const Control next_perturbed{
          .time = next.time,
          .point = accumulate(next.point, EPSILON * Vec::Unit(ii)),
      };
      const TCurve<Group> next_perturbed_curve{prev, next_perturbed};
      const TwoJet next_perturbed_point{next_perturbed_curve.point_at(time)};

      expected_differential_column =
          difference(next_perturbed_point, point_and_diffs.point) / EPSILON;

      EXPECT_TRUE(math::is_approx(
          expected_differential_column,
          point_and_diffs.d_next.col(ii),
          TOLERANCE));
    }
  }
}

}  // namespace resim::curves::optimization
