// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/t_curve_test_helpers.hh"

#include <gtest/gtest.h>

#include "resim/curves/t_curve.hh"
#include "resim/curves/two_jet.hh"
#include "resim/curves/two_jet_test_helpers.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves {

namespace {
// An explicit seed for deterministic generation of test objects.
constexpr unsigned int SEED = 103;

using SE3 = transforms::SE3;
using SO3 = transforms::SO3;
}  // namespace

template <transforms::LieGroupType Group>
class TCurveHelperTests : public ::testing::Test {
 public:
  TCurveTestHelper<Group> t_curve_helper = TCurveTestHelper<Group>(SEED);
  inline static const std::vector<double> EMPTY_TIMES{};
  inline static const std::vector<double> SINGLETON_TIMES{0.0};
  inline static const std::vector<double> THREE_TIMES{0.0, 0.11, 0.67};
};

using LieGroupTypes = ::testing::Types<SO3, SE3>;

TYPED_TEST_SUITE(TCurveHelperTests, LieGroupTypes);

TYPED_TEST(TCurveHelperTests, TwoJetHelperAccessTest) {
  auto tj_helper = TwoJetTestHelper<TwoJetL<TypeParam>>(SEED);
  // Check, essentially, that the seed is generating identical two jets
  ASSERT_TRUE(TestFixture::t_curve_helper.two_jet_helper()
                  .make_test_two_jet()
                  .is_approx(tj_helper.make_test_two_jet()));
}

TYPED_TEST(TCurveHelperTests, ControlPointsTest) {
  const auto test_t_curve =
      TestFixture::t_curve_helper.make_t_curve(TestFixture::THREE_TIMES);
  EXPECT_EQ(test_t_curve.control_pts().size(), TestFixture::THREE_TIMES.size());
  const auto test_singleton_t_curve =
      TestFixture::t_curve_helper.make_t_curve(TestFixture::SINGLETON_TIMES);
  EXPECT_EQ(
      test_singleton_t_curve.control_pts().size(),
      TestFixture::SINGLETON_TIMES.size());
  const auto test_empty_t_curve =
      TestFixture::t_curve_helper.make_t_curve(TestFixture::EMPTY_TIMES);
  EXPECT_EQ(
      test_empty_t_curve.control_pts().size(),
      TestFixture::EMPTY_TIMES.size());
}

template <typename Group>
class FramedTCurveHelperTests : public TCurveHelperTests<Group> {};

using FramedTypes = ::testing::Types<transforms::SE3, transforms::SO3>;

TYPED_TEST_SUITE(FramedTCurveHelperTests, FramedTypes);

TYPED_TEST(FramedTCurveHelperTests, CheckFrames) {
  const auto test_t_curve =
      TestFixture::t_curve_helper.make_t_curve(TestFixture::THREE_TIMES);
  for (const auto &control : test_t_curve.control_pts()) {
    EXPECT_EQ(
        control.point.frame_from_ref().from(),
        TCurveTestHelper<TypeParam>::REF_FRAME);
    EXPECT_EQ(
        control.point.frame_from_ref().into(),
        TCurveTestHelper<TypeParam>::POINT_FRAME);
  }
}

}  // namespace resim::curves
