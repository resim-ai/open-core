// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#include "resim/curves/optimization/two_jet_tangent_space.hh"

#include <gtest/gtest.h>

#include "resim/curves/two_jet.hh"
#include "resim/curves/two_jet_test_helpers.hh"
#include "resim/transforms/liegroup_concepts.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves::optimization {

using transforms::SE3;
using transforms::SO3;

template <transforms::LieGroupType Group>
class TwoJetTangentSpaceTest : public ::testing::Test {
 protected:
  TwoJetTestHelper<TwoJetL<Group>> test_helper_;
};

using LieGroupTypes = ::testing::Types<SO3, SE3>;
TYPED_TEST_SUITE(TwoJetTangentSpaceTest, LieGroupTypes);

TYPED_TEST(TwoJetTangentSpaceTest, TestAccumulateDifferenceRoundTrip) {
  // SETUP
  using Group = TypeParam;

  constexpr int NUM_TESTS = 10;

  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const TwoJetL<Group> element = this->test_helper_.make_test_two_jet();
    const TwoJetL<Group> other = this->test_helper_.make_test_two_jet();

    // ACTION / VERIFICATION
    EXPECT_TRUE(
        other.is_approx(accumulate(element, difference(other, element))));

    // VERIFICATION
    EXPECT_TRUE(
        difference(other, element).isApprox(-difference(element, other)));
  }
}

}  // namespace resim::curves::optimization
