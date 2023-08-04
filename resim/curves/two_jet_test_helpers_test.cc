// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/two_jet_test_helpers.hh"

#include <gtest/gtest.h>

#include "resim/assert/assert.hh"
#include "resim/curves/two_jet.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves {

// An explicit seed for deterministic generation of test objects.
constexpr unsigned int SEED = 104;

template <typename TwoJet>
class TwoJetTestHelperTests : public ::testing::Test {
 public:
  TwoJetTestHelper<TwoJet> tj_helper = TwoJetTestHelper<TwoJet>(SEED);
};

using TwoJetTypes =
    ::testing::Types<TwoJetL<transforms::SO3>, TwoJetL<transforms::SE3>>;

TYPED_TEST_SUITE(TwoJetTestHelperTests, TwoJetTypes);

TYPED_TEST(TwoJetTestHelperTests, ReturnCounts) {
  const auto test_min_two_jet =
      TestFixture::tj_helper.make_test_two_jet_elements();
  EXPECT_EQ(test_min_two_jet.size(), detail::MIN_TEST_ELEMENTS);

  constexpr unsigned LARGE_COUNT = 51;
  const auto test_large_two_jet =
      TestFixture::tj_helper.make_test_two_jet_elements(LARGE_COUNT);
  EXPECT_EQ(test_large_two_jet.size(), LARGE_COUNT);
}

TYPED_TEST(TwoJetTestHelperTests, TestMakeTwoJetElements) {
  // Build the vector of test elements
  std::vector<TypeParam> test_elements =
      TestFixture::tj_helper.make_test_two_jet_elements();

  // Confirm the vecor is not empty
  EXPECT_FALSE(test_elements.empty());

  // Confirm that all elements are unique
  // First sort
  std::sort(
      test_elements.begin(),
      test_elements.end(),
      [](const TypeParam &a, const TypeParam &b) -> bool {
        //
        return a.d_frame_from_ref().norm() > b.d_frame_from_ref().norm();
      });

  // Then find adjacent duplicates (or not)
  const auto last = std::adjacent_find(
      test_elements.begin(),
      test_elements.end(),
      [](const TypeParam &a, const TypeParam &b) -> bool {
        //
        return a.is_approx(b);
      });
  EXPECT_EQ(last, test_elements.end());
}

template <typename TwoJet>
using TwoJetTestHelperDeathTests = TwoJetTestHelperTests<TwoJet>;
TYPED_TEST_SUITE(TwoJetTestHelperDeathTests, TwoJetTypes);

TYPED_TEST(TwoJetTestHelperDeathTests, TooFewElementsRequested) {
  constexpr unsigned TOO_FEW = detail::MIN_TEST_ELEMENTS - 1;
  EXPECT_THROW(
      {
        const auto test_tj =
            TestFixture::tj_helper.make_test_two_jet_elements(TOO_FEW);
        (void)test_tj;
      },
      AssertException);
}

}  // namespace resim::curves
