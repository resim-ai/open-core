// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/proto/two_jet_to_proto.hh"

#include <gtest/gtest.h>

#include <vector>

#include "random"
#include "resim/assert/assert.hh"
#include "resim/curves/proto/two_jet.pb.h"
#include "resim/curves/proto/two_jetl_se3_to_proto.hh"
#include "resim/curves/proto/two_jetl_so3_to_proto.hh"
#include "resim/curves/proto/two_jetr_se3_to_proto.hh"
#include "resim/curves/proto/two_jetr_so3_to_proto.hh"
#include "resim/curves/two_jet.hh"
#include "resim/curves/two_jet_test_helpers.hh"
#include "resim/math/proto/matrix_to_proto.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/proto/se3_to_proto.hh"
#include "resim/transforms/proto/so3_to_proto.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves {

namespace {
using TwoJetLSE3 = curves::TwoJetL<transforms::SE3>;
using TwoJetLSO3 = curves::TwoJetL<transforms::SO3>;

using TwoJetRSE3 = curves::TwoJetR<transforms::SE3>;
using TwoJetRSO3 = curves::TwoJetR<transforms::SO3>;

// For each test below we employ (deterministic) randomly generated TwoJet
// objects. We desire to test a few different generated TwoJets in order to
// confirm the implementation is robust. This number should be more than 1.
// However, it does not need to be hundreds, because we are testing
// fundamental functionality which should not be susceptible to errors of high
// sensitivity. Seven is a (hopefully) lucky guess at the 'right' number.
constexpr unsigned int NUM_TRIES = 7;

// An explicit seed for deterministic generation of test objects.
constexpr unsigned int SEED = 31;
}  // namespace

template <typename Pair>
class TwoJetToProtoTestsBase : public ::testing::Test {
 protected:
  void SetUp() override {
    tj_helper_ = TwoJetTestHelper<typename Pair::first_type>(SEED);
  }
  TwoJetTestHelper<typename Pair::first_type> &tj_helper() {
    return tj_helper_;
  }

 private:
  TwoJetTestHelper<typename Pair::first_type> tj_helper_;
};

template <typename Pair>
class TwoJetToProtoCommonTests : public TwoJetToProtoTestsBase<Pair> {};

using TwoJetTypePairs = ::testing::Types<
    std::pair<TwoJetLSE3, proto::TwoJetL_SE3>,
    std::pair<TwoJetLSO3, proto::TwoJetL_SO3>,
    std::pair<TwoJetRSE3, proto::TwoJetR_SE3>,
    std::pair<TwoJetRSO3, proto::TwoJetR_SO3>>;

TYPED_TEST_SUITE(TwoJetToProtoCommonTests, TwoJetTypePairs);

TYPED_TEST(TwoJetToProtoCommonTests, TestRoundTrip) {
  // SETUP
  using TwoJet = typename TypeParam::first_type;
  using TwoJetMsg = typename TypeParam::second_type;
  TwoJetMsg msg;
  std::vector<TwoJet> test_elements =
      TestFixture::tj_helper().make_test_two_jet_elements(NUM_TRIES);
  // ACTION/VERIFICATION
  for (const TwoJet &test_tj : test_elements) {
    proto::pack(test_tj, &msg);
    const TwoJet retrieved_two_jet = proto::unpack(msg);
    EXPECT_TRUE(test_tj.is_approx(retrieved_two_jet));
  }
}

template <typename Pair>
using TwoJetToProtoDeathTests = TwoJetToProtoCommonTests<Pair>;
TYPED_TEST_SUITE(TwoJetToProtoDeathTests, TwoJetTypePairs);

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(TwoJetToProtoDeathTests, TestPackNull) {
  // SETUP
  using TwoJet = typename TypeParam::first_type;
  std::vector<TwoJet> test_elements =
      TestFixture::tj_helper().make_test_two_jet_elements(NUM_TRIES);
  // ACTION/VERIFICATION
  for (const TwoJet &test_tj : test_elements) {
    EXPECT_THROW({ proto::pack(test_tj, nullptr); }, AssertException);
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

template <typename Pair>
class TwoJetLToProtoTests : public TwoJetToProtoTestsBase<Pair> {};

using TwoJetLTypePairs = ::testing::Types<
    std::pair<TwoJetLSE3, proto::TwoJetL_SE3>,
    std::pair<TwoJetLSO3, proto::TwoJetL_SO3>>;

TYPED_TEST_SUITE(TwoJetLToProtoTests, TwoJetLTypePairs);

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(TwoJetLToProtoTests, TestPack) {
  // SETUP
  using TwoJet = typename TypeParam::first_type;
  using TwoJetMsg = typename TypeParam::second_type;
  TwoJetMsg msg;
  std::vector<TwoJet> test_elements =
      TestFixture::tj_helper().make_test_two_jet_elements(NUM_TRIES);

  // ACTION/VERIFICATION
  for (const TwoJet &test_tj : test_elements) {
    proto::pack(test_tj, &msg);
    const TwoJet retrieved_tj = proto::unpack(msg);
    EXPECT_TRUE(
        test_tj.frame_from_ref().is_approx(retrieved_tj.frame_from_ref()));
    EXPECT_TRUE(
        retrieved_tj.d_frame_from_ref().isApprox(test_tj.d_frame_from_ref()));
    EXPECT_TRUE(
        retrieved_tj.d2_frame_from_ref().isApprox(test_tj.d2_frame_from_ref()));
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

template <typename Pair>
class TwoJetRToProtoTests : public TwoJetToProtoTestsBase<Pair> {};

using TwoJetRTypePairs = ::testing::Types<
    std::pair<TwoJetRSE3, proto::TwoJetR_SE3>,
    std::pair<TwoJetRSO3, proto::TwoJetR_SO3>>;

TYPED_TEST_SUITE(TwoJetRToProtoTests, TwoJetRTypePairs);

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(TwoJetRToProtoTests, TestPack) {
  // SETUP
  using TwoJet = typename TypeParam::first_type;
  using TwoJetMsg = typename TypeParam::second_type;
  TwoJetMsg msg;
  std::vector<TwoJet> test_elements =
      TestFixture::tj_helper().make_test_two_jet_elements(NUM_TRIES);
  // ACTION/VERIFICATION
  for (const TwoJet &test_tj : test_elements) {
    proto::pack(test_tj, &msg);
    const TwoJet retrieved_tj = proto::unpack(msg);
    EXPECT_TRUE(
        test_tj.ref_from_frame().is_approx(retrieved_tj.ref_from_frame()));
    EXPECT_TRUE(
        retrieved_tj.d_ref_from_frame().isApprox(test_tj.d_ref_from_frame()));
    EXPECT_TRUE(
        retrieved_tj.d2_ref_from_frame().isApprox(test_tj.d2_ref_from_frame()));
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim::curves
