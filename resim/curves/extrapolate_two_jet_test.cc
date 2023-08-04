// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/extrapolate_two_jet.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/curves/two_jet.hh"
#include "resim/curves/two_jet_test_helpers.hh"
#include "resim/math/is_approx.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/liegroup_concepts.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves {

template <transforms::LieGroupType Group>
class ExtrapolateTwoJetTests : public ::testing::Test {
 protected:
  void test_extrapolation(const double dt) {
    // SETUP
    using Tangent = typename Group::TangentVector;

    TwoJetL<Group> two_jet = tj_helper.make_test_two_jet();

    // ACTION
    TwoJetL<Group> extrapolated_two_jet{extrapolate_two_jet(two_jet, dt)};

    // VERIFICATION
    const Tangent actual{(extrapolated_two_jet.frame_from_ref() *
                          two_jet.frame_from_ref().inverse())
                             .log()};
    const Tangent expected{
        dt *
        (two_jet.d_frame_from_ref() + 0.5 * dt * two_jet.d2_frame_from_ref())};

    EXPECT_TRUE(math::is_approx(actual, expected));

    EXPECT_TRUE(
        (extrapolated_two_jet.d_frame_from_ref() - two_jet.d_frame_from_ref())
            .isApprox(dt * two_jet.d2_frame_from_ref()));
    EXPECT_EQ(
        extrapolated_two_jet.d2_frame_from_ref(),
        two_jet.d2_frame_from_ref());

    // Verify that the frame assignment is done correctly.
    EXPECT_EQ(
        extrapolated_two_jet.frame_from_ref().into(),
        two_jet.frame_from_ref().into());
    EXPECT_EQ(
        extrapolated_two_jet.frame_from_ref().from(),
        two_jet.frame_from_ref().from());
  }

  void test_frame_overload(const double dt) {
    // SETUP
    const transforms::Frame<Group::DIMS> frame =
        transforms::Frame<Group::DIMS>::new_frame();

    TwoJetL<Group> two_jet = tj_helper.make_test_two_jet();

    // ACTION
    TwoJetL<Group> extrapolated_two_jet_explicit{
        extrapolate_two_jet(two_jet, dt, frame)};

    // VERIFICATION
    // Verify using the other overload tested above
    TwoJetL<Group> extrapolated_two_jet{extrapolate_two_jet(two_jet, dt)};
    // But now manually set the frame.
    auto two_jet_transform = extrapolated_two_jet.frame_from_ref();
    two_jet_transform.set_frames(frame, two_jet_transform.from());
    extrapolated_two_jet.set_frame_from_ref(two_jet_transform);
    // Compare the two TwoJets.
    EXPECT_TRUE(extrapolated_two_jet.is_approx(extrapolated_two_jet_explicit));

    // Explicitly check the frames.
    EXPECT_EQ(extrapolated_two_jet_explicit.frame_from_ref().into(), frame);
    EXPECT_EQ(
        extrapolated_two_jet_explicit.frame_from_ref().from(),
        two_jet.frame_from_ref().from());
  }

 private:
  static constexpr unsigned SEED = 839U;
  TwoJetTestHelper<TwoJetL<Group>> tj_helper =
      TwoJetTestHelper<TwoJetL<Group>>(SEED);
};

using LieGroupTypes = ::testing::Types<transforms::SE3, transforms::SO3>;
TYPED_TEST_SUITE(ExtrapolateTwoJetTests, LieGroupTypes);

TYPED_TEST(ExtrapolateTwoJetTests, TestExtrapolation) {
  constexpr int NUM_TESTS = 1000;
  for (const double dt : {-0.1, 0.0, 0.1, 0.2, 0.3}) {
    for (int ii = 0; ii < NUM_TESTS; ++ii) {
      this->test_extrapolation(dt);
    }
  }
}

TYPED_TEST(ExtrapolateTwoJetTests, TestFrameOverload) {
  constexpr int NUM_TESTS = 1000;
  for (const double dt : {-0.1, 0.0, 0.1, 0.2, 0.3}) {
    for (int ii = 0; ii < NUM_TESTS; ++ii) {
      this->test_frame_overload(dt);
    }
  }
}

}  // namespace resim::curves
