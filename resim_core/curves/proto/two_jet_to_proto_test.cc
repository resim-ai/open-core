#include "resim_core/curves/proto/two_jet_to_proto.hh"

#include <gtest/gtest.h>

#include <vector>

#include "random"
#include "resim_core/curves/proto/two_jet.pb.h"
#include "resim_core/curves/proto/two_jetl_fse3_to_proto.hh"
#include "resim_core/curves/proto/two_jetl_fso3_to_proto.hh"
#include "resim_core/curves/proto/two_jetl_se3_to_proto.hh"
#include "resim_core/curves/proto/two_jetl_so3_to_proto.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/curves/two_jet_test_helpers.hh"
#include "resim_core/math/proto/matrix_to_proto.hh"
#include "resim_core/testing/random_matrix.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/proto/fse3_to_proto.hh"
#include "resim_core/transforms/proto/fso3_to_proto.hh"
#include "resim_core/transforms/proto/se3_to_proto.hh"
#include "resim_core/transforms/proto/so3_to_proto.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::curves {

namespace {
using TwoJetSE3 = curves::TwoJetL<transforms::SE3>;
using TwoJetSO3 = curves::TwoJetL<transforms::SO3>;
using TwoJetFSE3 = curves::TwoJetL<transforms::FSE3>;
using TwoJetFSO3 = curves::TwoJetL<transforms::FSO3>;

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
class TwoJetToProtoTests : public ::testing::Test {
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

using TwoJetTypePairs = ::testing::Types<
    std::pair<TwoJetSE3, proto::TwoJetL_SE3>,
    std::pair<TwoJetSO3, proto::TwoJetL_SO3>,
    std::pair<TwoJetFSO3, proto::TwoJetL_FSO3>,
    std::pair<TwoJetFSE3, proto::TwoJetL_FSE3>>;

TYPED_TEST_SUITE(TwoJetToProtoTests, TwoJetTypePairs);

TYPED_TEST(TwoJetToProtoTests, TestPack) {
  // SETUP
  using TwoJet = typename TypeParam::first_type;
  using Group = typename TwoJet::GroupType;
  using TwoJetMsg = typename TypeParam::second_type;
  TwoJetMsg msg;
  std::vector<TwoJet> test_elements =
      TestFixture::tj_helper().make_test_two_jet_elements(NUM_TRIES);
  // ACTION/VERIFICATION
  for (const TwoJet &test_tj : test_elements) {
    proto::pack(test_tj, &msg);
    const Group retrieved_group =
        transforms::proto::unpack(msg.frame_from_ref());
    EXPECT_TRUE(test_tj.frame_from_ref().is_approx(retrieved_group));
    // Now test the derivatives
    typename Group::TangentVector d_frame_from_ref;
    math::proto::unpack_matrix(msg.d_frame_from_ref(), InOut(d_frame_from_ref));
    EXPECT_TRUE(d_frame_from_ref.isApprox(test_tj.d_frame_from_ref()));
    typename Group::TangentVector d2_frame_from_ref;
    math::proto::unpack_matrix(
        msg.d2_frame_from_ref(),
        InOut(d2_frame_from_ref));
    EXPECT_TRUE(d2_frame_from_ref.isApprox(test_tj.d2_frame_from_ref()));
  }
}

TYPED_TEST(TwoJetToProtoTests, TestRoundTrip) {
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
using TwoJetToProtoDeathTests = TwoJetToProtoTests<Pair>;
TYPED_TEST_SUITE(TwoJetToProtoDeathTests, TwoJetTypePairs);

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(TwoJetToProtoDeathTests, TestPackNull) {
  // SETUP
  using TwoJet = typename TypeParam::first_type;
  std::vector<TwoJet> test_elements =
      TestFixture::tj_helper().make_test_two_jet_elements(NUM_TRIES);
  // ACTION/VERIFICATION
  for (const TwoJet &test_tj : test_elements) {
    EXPECT_DEATH(
        { proto::pack(test_tj, nullptr); },
        "Can't pack into invalid proto!");
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim::curves
